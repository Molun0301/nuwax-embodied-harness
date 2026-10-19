//! Real admitted Heightfield-v1 fixtures for scalar body transport and contact.
use super::*;
use crate::surface::body_query::{BodyGroundState,GroundSample};
use nuwax_map_chunks::heightfield_observations::HeightfieldColumn;
use nuwax_map_chunks::{ColumnGroundRecords,Domain,HeightfieldSurface,Receiver,Repository,
    Transfer,Writer};
use std::collections::{BTreeMap,BTreeSet};

const Q:f32=0.05;

#[test]
fn current_classified_ground_does_not_wait_for_static_membership_and_mixed_voxel_blocks() {
    let whole=fixed_go2_model();
    let map=scene(0,false).with_body_envelope(whole).unwrap();
    let from=([2.025,0.025,0.4],[0.0,0.0,0.0,1.0]);
    assert_eq!(map.query_ground([2.025,0.025],0.0).state,GroundQueryState::Unknown);
    let point=[2.025,0.025,0.025];
    let classified=map.with_recent_obstacle_points(1,1,0,&[0],[(0,point)]).unwrap();
    let owner=classified.body_query();let state=owner.start_ground_state(from).unwrap();
    owner.validate_motion_from_ground(from,from,&state,&[],&mut None).unwrap();
    assert!(classified.recent_obstacle_hints([[1.9,-0.1],[2.2,0.2]],from).unwrap().is_empty());
    for source in [
        map.with_recent_obstacle_points(1,1,0,&[],[(0,point)]).unwrap(),
        map.with_recent_obstacle_points(1,1,0,&[0],[(0,point),(1,[2.026,0.026,0.026])]).unwrap(),
        map.with_recent_obstacle_points(1,1,0,&[0],[(0,[2.025,0.025,0.425])]).unwrap(),
    ] {
        let owner=source.body_query();let state=owner.start_ground_state(from).unwrap();
        let mut witness=None;
        assert_eq!(owner.validate_motion_from_ground(from,from,&state,&[],&mut witness),
            Err(SurfaceError::InsufficientClearance));
        assert!(matches!(witness.unwrap().rejection,BodyRejection::Occupancy{source:OccupancySource::Recent,..}));
    }
}

#[test]
fn owner_bound_cache_reuses_exact_state_without_aliasing_height_or_source() {
    let map=scene(0,false);let owner=map.body_query();
    let pose=([0.025,0.025,0.425],[0.0,0.0,0.0,1.0]);
    let state=owner.start_ground_state(pose).unwrap();
    map.validate_body_pose_in_context(pose.0,pose.1,QuerySample::Ground(&state.sample),&mut None).unwrap();
    let (patches,poses)={let c=map.body_query_cache.lock().unwrap();(c.bound_ground_patches.len(),c.bound_poses.len())};
    assert!(patches>0 && poses==1);
    map.validate_body_pose_in_context(pose.0,pose.1,QuerySample::Ground(&state.sample),&mut None).unwrap();
    {let c=map.body_query_cache.lock().unwrap();assert_eq!((c.bound_ground_patches.len(),c.bound_poses.len()),(patches,poses));}
    let mut another=state.sample.clone();another.reference_origin[2]+=0.35;
    assert!(BoundPoseGroundKey::new(QuerySample::Ground(&state.sample))!=BoundPoseGroundKey::new(QuerySample::Ground(&another)));
    another=state.sample.clone();another.reference_sources=vec![NativeGroundSource::HeightfieldFine{key:GroundKey::HeightfieldV1([1,0,0])}];
    assert!(BoundPoseGroundKey::new(QuerySample::Ground(&state.sample))!=BoundPoseGroundKey::new(QuerySample::Ground(&another)));
    // An unchanged pose on a new recent source must not inherit the old permit.
    let blocked=map.with_recent_obstacle_points(1,1,0,&[],[(0,[0.025,0.025,0.425])]).unwrap();
    assert_eq!(blocked.validate_body_pose_in_context(pose.0,pose.1,QuerySample::Ground(&state.sample),&mut None),
        Err(SurfaceError::InsufficientClearance));
}

fn model()->SurfaceBodyEnvelope {
    SurfaceBodyEnvelope {fixed_collision:None,footprint_xy_m:vec![[-0.06,-0.06],[0.06,-0.06],[0.06,0.06],[-0.06,0.06]],
        z_bounds_m:[-0.10,0.10],body_to_support_z_m:0.4,max_step_up_m:0.21,max_step_down_m:0.21,
        max_cross_slope_deg:18.0,support_uncertainty_m:0.0}
}

fn fixed_go2_model()->SurfaceBodyEnvelope {
    use nuwax_types::{FixedCollisionGeometry,FixedCollisionPrimitive,FixedCollisionShape};
    let primitive=|name:&str,translation_m,shape|FixedCollisionPrimitive {link_name:name.into(),translation_m,
        rotation_row_major:[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0],shape};
    let mut result=model();
    // These are the existing URDF fixed collisions, not a trunk-bottom guess.
    // The outer rectangle conservatively contains the original whole-body XY.
    result.footprint_xy_m=vec![[-0.24,-0.164],[0.34,-0.164],[0.34,0.164],[-0.24,0.164]];
    result.z_bounds_m=[-0.448,0.057];
    result.fixed_collision=Some(Arc::new(FixedCollisionGeometry {model_base_link:"base".into(),
        excluded_moving_collision_count:24,unsupported_collision_count:0,primitives:vec![
            primitive("base",[0.0;3],FixedCollisionShape::Box{size_m:[0.3762,0.0935,0.114]}),
            primitive("Head_upper",[0.285,0.0,0.01],FixedCollisionShape::Cylinder{radius_m:0.05,length_m:0.09}),
            primitive("Head_lower",[0.293,0.0,-0.06],FixedCollisionShape::Sphere{radius_m:0.047}),
        ]}));
    result
}

fn scene(step:i32,high_member:bool)->SurfaceSnapshot {scene_with_roughness(step,high_member,0)}

fn scene_with_roughness(step:i32,high_member:bool,rough:i32)->SurfaceSnapshot {
    scene_with_counts(|x,y| {
        let z=if x>=0 {step}else{0};
        let mut counts=BTreeMap::from([(z,[1000,0,0,0,0,0,0,0])]);
        if high_member && x==0 && y==0 {counts.insert(8,[1,0,0,0,0,0,0,0]);}
        if rough>0 {counts.insert(z+rough,[1000,0,0,0,0,0,0,0]);}
        counts
    })
}

fn scene_with_counts(counts:impl Fn(i32,i32)->BTreeMap<i32,[u32;8]>)->SurfaceSnapshot {
    scene_with_raw_counts(counts,&[])
}

fn scene_with_raw_counts(counts:impl Fn(i32,i32)->BTreeMap<i32,[u32;8]>,extra:&[[i32;3]])->SurfaceSnapshot {
    scene_with_raw_counts_and_empty(counts,extra,None)
}

fn scene_with_raw_counts_and_empty(counts:impl Fn(i32,i32)->BTreeMap<i32,[u32;8]>,
    extra:&[[i32;3]],empty:Option<([i32;2],i32)>)->SurfaceSnapshot {
    scene_with_native_columns(|x,y|vec![HeightfieldColumn::from_counts([x,y],Q,counts(x,y)).unwrap().unwrap()],extra,empty)
}

fn scene_with_native_columns(columns:impl Fn(i32,i32)->Vec<HeightfieldColumn>,
    extra:&[[i32;3]],empty:Option<([i32;2],i32)>)->SurfaceSnapshot {
    let mut writer=Writer::new(Repository::temporary().unwrap(),Q,4*1024*1024).unwrap();
    let mut points=vec![];
    for x in -12..12 {for y in -8..8 {
        if let Some((_,z))=empty.filter(|(xy,_)|*xy==[x,y]) {
            writer.observe([[x,y,z]],20).unwrap();
            points.push(((x as f32+0.5)*Q,(y as f32+0.5)*Q,(z as f32+0.5)*Q));
            writer.replace_ground_column([x,y],&ColumnGroundRecords {
                classified_empty:true,legacy:vec![],heightfields:vec![]}).unwrap();
            continue;
        }
        let columns=columns(x,y);
        for (z,_) in columns.iter().flat_map(HeightfieldColumn::members) {
            writer.observe([[x,y,z]],20).unwrap();
            points.push(((x as f32+0.5)*Q,(y as f32+0.5)*Q,(z as f32+0.5)*Q));
        }
        let record=ColumnGroundRecords {classified_empty: false, legacy:vec![],heightfields:columns.into_iter().map(|column|
            HeightfieldSurface {column,samples:vec![]}).collect()};
        writer.replace_ground_column([x,y],&record).unwrap();
    }}
    writer.observe(extra.iter().copied(),20).unwrap();
    points.extend(extra.iter().map(|&[x,y,z]|((x as f32+0.5)*Q,(y as f32+0.5)*Q,(z as f32+0.5)*Q)));
    let root=writer.snapshot(Domain {session:"body-native-contract".into(),generation:1,
        frame_generation:0,producer_generation:1},1,20).unwrap();
    let mut receiver=Receiver::new(Repository::temporary().unwrap());
    receiver.receive(&Transfer {root:root.encode().unwrap(),included:BTreeMap::new()},&root.domain,
        |_,r|writer.repository().read_object(r.object,nuwax_map_chunks::MAX_BLOCK_BYTES)).unwrap();
    let mut observations=SurfaceObservations::new(Q,32).unwrap();
    let mut parts=vec![];
    for r in &root.blocks {
        let (block,records)=receiver.ground_parts(r).unwrap();
        observations.replace_block(r.key,block.iter().map(|(&local,e)|
            (nuwax_map_chunks::global_key(r.key,local).unwrap(),SurfaceObservation {
                observed_ns:e.observed_ns,hits:e.hits})).collect()).unwrap();
        parts.push((r.key,records));
    }
    for xy in parts.iter().map(|(k,_)|[k[0],k[1]]).collect::<BTreeSet<_>>() {
        let columns=parts.iter().filter(|(k,_)|k[..2]==xy).cloned().collect::<Vec<_>>();
        observations.replace_heightfield_columns(xy,&columns);
    }
    observations.mark_complete_geometry_source();
    let config=Config {world_frame:"map".into(),voxel_size:0.1,robot_height:0.2,surface_closing_radius:0.0,
        node_spacing_m:0.3,wall_clearance_m:0.0,wall_buffer_m:0.2,wall_buffer_weight:2.0,
        max_slope_deg:18.0,step_threshold_m:0.21,min_step_tread_depth_m:0.2,step_penalty_weight:2.0,
        goal_tolerance:0.1,viz_publish_hz:0.0};
    SurfaceSnapshot::from_points(&points,config,Q).unwrap().with_body_envelope(model()).unwrap()
        .with_observations(Some(Arc::new(observations)))
}

#[test]
fn fixed_ground_members_keep_rigid_hits_and_nonmembers_keep_whole_body_hits() {
    let q=[0.0,0.0,0.0,1.0];let pose=([0.025,0.025,0.4],q);
    let high_member=scene(0,true).with_body_envelope(fixed_go2_model()).unwrap();
    let low_plate=scene_with_raw_counts(|_,_|BTreeMap::from([(0,[1000,0,0,0,0,0,0,0])]),&[[0,0,3]])
        .with_body_envelope(fixed_go2_model()).unwrap();
    let outer_corner=scene_with_raw_counts(|_,_|BTreeMap::from([(0,[1000,0,0,0,0,0,0,0])]),&[[5,2,5]])
        .with_body_envelope(fixed_go2_model()).unwrap();
    for (name,map,key,is_member) in [("high_member",high_member,[0,0,8],true),
        ("nonmember_low_plate",low_plate,[0,0,3],false),("nonmember_outer_corner",outer_corner,[5,2,5],false)] {
        let record=map.observations().unwrap().ground_records([key[0],key[1]]).next().unwrap();
        assert_eq!(record.contains_member(key),is_member);
        let owner=map.body_query();let state=owner.start_ground_state(pose).unwrap();let mut witness=None;
        assert_eq!(owner.validate_motion_from_ground(pose,pose,&state,&[],&mut witness),
            Err(SurfaceError::InsufficientClearance),"{name}");
        assert!(witness.is_some(),"{name}: actual intersection retains witness");
    }
}

#[test]
fn fixed_geometry_change_with_same_outer_envelope_retires_pose_and_source_owner() {
    use nuwax_types::FixedCollisionShape;
    let mut model=fixed_go2_model();
    let fixed=Arc::make_mut(model.fixed_collision.as_mut().unwrap());
    fixed.primitives.truncate(1);
    fixed.primitives[0].shape=FixedCollisionShape::Box{size_m:[0.02,0.02,0.02]};
    let before=scene_with_counts(|x,y| {
        let mut counts=BTreeMap::from([(0,[1000,0,0,0,0,0,0,0])]);
        if x==1 && y==0 {counts.insert(8,[1,0,0,0,0,0,0,0]);}
        counts
    }).with_body_envelope(model.clone()).unwrap();
    let pose=([0.025,0.025,0.425],[0.0,0.0,0.0,1.0]);
    let owner=before.body_query();let ground=owner.start_ground_state(pose).unwrap();
    owner.validate_motion_from_ground(pose,pose,&ground,&[],&mut None).unwrap();
    Arc::make_mut(model.fixed_collision.as_mut().unwrap()).primitives[0].shape=
        FixedCollisionShape::Box{size_m:[0.20,0.02,0.02]};
    let after=before.with_body_envelope(model).unwrap();
    assert!(!owner.same_body_query_view(&after));
    assert!(!owner.same_ground_source(&after));
    assert!(!Arc::ptr_eq(&before.body_query_cache,&after.body_query_cache));
    let next=after.body_query();let state=next.start_ground_state(pose).unwrap();
    assert_eq!(next.validate_motion_from_ground(pose,pose,&state,&[],&mut None),
        Err(SurfaceError::InsufficientClearance));
}

#[test]
fn missing_or_partial_fixed_geometry_keeps_existing_contact_compatibility() {
    let mut full=fixed_go2_model();full.max_step_up_m=0.0;full.max_step_down_m=0.0;
    let mut missing=full.clone();missing.fixed_collision=None;
    let mut empty=full.clone();Arc::make_mut(empty.fixed_collision.as_mut().unwrap()).primitives.clear();
    let mut partial=full.clone();Arc::make_mut(partial.fixed_collision.as_mut().unwrap()).unsupported_collision_count=1;
    for (name,model,allowed) in [("missing",missing,false),("empty",empty,false),
        ("partial",partial,false),("real_go2",full,true)] {
        let map=scene_with_roughness(0,false,3).with_body_envelope(model).unwrap();
        let h=map.query_ground([0.025,0.025],0.0).height.unwrap();
        let pose=([0.025,0.025,h+0.4],[0.0,0.0,0.0,1.0]);
        let owner=map.body_query();let state=owner.start_ground_state(pose).unwrap();
        let result=owner.validate_motion_from_ground(pose,pose,&state,&[],&mut None);
        if allowed {result.unwrap_or_else(|error|panic!("{name}: {error:?}"));}
        else {assert_eq!(result,Err(SurfaceError::InsufficientClearance),
            "{name}: old compatibility result, not an invented missing-support gate or subset exemption");}
    }
}

#[test]
fn legacy_plane_and_heightfield_exact_members_share_finite_contact_semantics() {
    use nuwax_map_chunks::{LayerObservation,LayerSample};
    for high in [false,true] {
        let native=scene_with_roughness(0,high,3).with_body_envelope(fixed_go2_model()).unwrap();
        let mut observed=SurfaceObservations::new(Q,32).unwrap();
        let mut blocks=BTreeMap::<[i32;3],BTreeMap<[i32;3],SurfaceObservation>>::new();
        let mut layers=BTreeMap::<[i32;3],BTreeMap<u16,LayerObservation>>::new();
        for x in -12..12 {for y in -8..8 {
            let record=native.observations().unwrap().ground_records([x,y]).next().unwrap();
            let members:Vec<_>=record.members().map(|z|[x,y,z]).collect();
            for &key in &members {blocks.entry(nuwax_map_chunks::block_key(key)).or_default()
                .insert(key,SurfaceObservation{observed_ns:20,hits:2});}
            let xy=[x,y].map(|v|(f64::from(v)+0.5)*f64::from(Q));
            let height=record.height_at(xy).unwrap();
            layers.entry(nuwax_map_chunks::block_key(members[0])).or_default().insert(
                nuwax_map_chunks::local_key(members[0]),LayerObservation {
                    members:members.iter().copied().map(nuwax_map_chunks::local_key).collect(),
                    normal:[0.0,0.0,1.0],plane_origin:[xy[0],xy[1],height],
                    samples:members.iter().enumerate().map(|(i,key)|LayerSample {
                        source:Default::default(),source_sequence:1,source_timestamp_ns:20,point_index:i as u32,
                        point:[xy[0],xy[1],(f64::from(key[2])+0.5)*f64::from(Q)]}).collect(),
                });
        }}
        for (block,values) in blocks {observed.replace_block(block,values).unwrap();}
        for (block,records) in layers {observed.replace_ground_layers(block,Arc::new(records)).unwrap();}
        observed.mark_complete_geometry_source();
        let legacy=native.with_observations(Some(Arc::new(observed)));
        for (representation,map) in [("HeightfieldV1",native),("LegacyPlane",legacy)] {
            let pose=([0.025,0.025,0.4],[0.0,0.0,0.0,1.0]);
            let query=map.body_query();let ground=query.start_ground_state(pose).unwrap();
            let result=query.validate_motion_from_ground(pose,pose,&ground,&[],&mut None);
            if high {assert_eq!(result,Err(SurfaceError::InsufficientClearance),"{representation}");}
            else {result.unwrap_or_else(|error|panic!("{representation}: {error:?}"));}
        }
    }
}

#[test]
fn fixed_whole_footprint_crosses_twenty_cm_tread_and_rejects_above_robot_step() {
    for (step,allowed) in [(4,true),(7,false)] {
        let map=scene(step,false).with_body_envelope(fixed_go2_model()).unwrap();
        let from=([-0.325,0.025,0.4],[0.0,0.0,0.0,1.0]);
        let owner=map.body_query();let state=owner.start_ground_state(from).unwrap();
        let result=owner.project_motion(from,&state,[0.325,0.025],0.0,&mut None);
        if allowed {
            let projection=result.unwrap();
            owner.validate_projected_motion(from,&projection,&[],&mut None).unwrap();
            assert!((projection.pose().0[2]-from.0[2]-f64::from(step)*f64::from(Q)).abs()<1e-9);
        } else {assert!(matches!(result,Err(SurfaceError::UntraversableSegment)));}
    }
}

#[test]
fn fixed_sweep_keeps_roundoff_boundary_and_fine_only_tangent_candidates() {
    use nuwax_types::FixedCollisionShape;
    let mut model=fixed_go2_model();
    model.footprint_xy_m=vec![[-0.01,-0.01],[0.01,-0.01],[0.01,0.01],[-0.01,0.01]];
    model.z_bounds_m=[-0.01,0.01];
    let geometry=Arc::make_mut(model.fixed_collision.as_mut().unwrap());
    geometry.primitives.truncate(1);
    geometry.primitives[0].translation_m=[0.086,0.0,0.0];
    geometry.primitives[0].shape=FixedCollisionShape::Sphere{radius_m:0.346};
    let map=scene(0,false).with_body_envelope(model).unwrap();
    let xyz=[-0.23199999701976773,0.0,0.425];
    let yaw=|angle:f64|[0.0,0.0,(angle*0.5).sin(),(angle*0.5).cos()];
    // The exact tangent lies between ordinary pose samples. Its X is one ULP
    // above body_x + (local_translation + radius), at the next fine column.
    let from=(xyz,yaw(-0.17));let to=(xyz,yaw(0.23));
    let point=[4.0*f64::from(Q),0.0,xyz[2]];
    assert!(map.fixed_body_query((xyz,yaw(0.0))).unwrap().unwrap().intersects_point(point).unwrap());
    let recent=map.with_recent_obstacle_points(1,1,0,&[0],[(0,point)]).unwrap();
    let mut observed=map.observations().unwrap().as_ref().clone();
    let mut block=BTreeMap::new();
    for x in 0..12 {for y in 0..8 {
        block.insert([x,y,0],observed.get([x,y,0]).unwrap());
    }}
    block.insert([4,0,8],SurfaceObservation{observed_ns:20,hits:2});
    observed.replace_block([0,0,0],block).unwrap();observed.mark_complete_geometry_source();
    let fine_only=map.with_observations(Some(Arc::new(observed)));
    for (kind,source) in [("recent_actual_point",recent),("fine_only_voxel",fine_only)] {
        let owner=source.body_query();let ground=owner.start_ground_state(from).unwrap();
        owner.validate_motion_from_ground(from,from,&ground,&[],&mut None).unwrap();
        owner.validate_motion_from_ground(to,to,&ground,&[],&mut None).unwrap();
        let mut witness=None;
        assert_eq!(owner.validate_motion_from_ground(from,to,&ground,&[],&mut witness),
            Err(SurfaceError::InsufficientClearance),"{kind}: continuous tangent must not disappear before narrow phase");
        assert!(witness.is_some(),"{kind}");
    }
}

#[test]
fn native_directionless_ramp_and_discrete_step_keep_original_slope_and_tread_limits() {
    // True associated sub-bin counts yield source h. No normal or fitted plane
    // is fabricated by this fixture; the raw occupancy remains real members.
    let slope=|rise_per_column:i32|scene_with_counts(|x,_| {
        let sub=x*rise_per_column;
        let z=sub.div_euclid(8);let mut counts=[0;8];counts[sub.rem_euclid(8) as usize]=1000;
        BTreeMap::from([(z,counts)])
    });
    let gentle=slope(1);let steep=slope(4);let stair=scene(3,false);
    for (name,map,allowed) in [("gentle_7deg",gentle,true),("continuous_26deg",steep,false),("15cm_tread",stair,true)] {
        let owner=map.body_query();let xy=[-0.325,0.025];
        let h=map.query_ground(xy,0.0).height.unwrap();
        assert_eq!(map.query_ground(xy,0.0).normal,None);
        let from=([xy[0],xy[1],h+0.4],[0.0,0.0,0.0,1.0]);
        let state=owner.start_ground_state(from).unwrap();
        let result=owner.project_motion(from,&state,[0.325,0.025],0.0,&mut None);
        if allowed {
            let next=result.unwrap_or_else(|error|panic!("{name}: {error:?}"));
            owner.validate_projected_motion(from,&next,&[],&mut None).unwrap_or_else(|error|panic!("{name}: sweep {error:?}"));
            let end_support=map.support_position_from_body(next.pose().0,next.pose().1,0.4).unwrap();
            assert!((end_support[2]-next.ground().height()).abs()<1e-9);
        } else {assert!(matches!(result,Err(SurfaceError::UntraversableSegment)),"{name}: {result:?}");}
    }
}

#[test]
fn native_height_survives_absent_direction_and_sparse_real_members() {
    let map=scene(0,true);let observed=map.observations().unwrap();
    let record=observed.ground_records([0,0]).next().unwrap();
    assert!(record.is_heightfield());assert_eq!(record.direction(),None);assert_eq!(record.scatter(),None);
    assert_eq!(record.members().collect::<Vec<_>>(),vec![0,8]);
    assert!(!record.contains_member([0,0,4]));
    let ground=map.query_ground([0.025,0.025],0.0);
    assert_eq!(ground.state,GroundQueryState::Observed);
    assert_eq!(ground.height,record.height_at([0.025,0.025]));
    assert_eq!(ground.normal,None);
    assert_eq!(record.occupied_bounds(f64::from(Q)),[0.0,9.0*f64::from(Q)]);
}

#[test]
fn native_projection_samples_offset_support_xy_and_preserves_real_tilt() {
    let map=scene(3,false);let owner=map.body_query();
    let angle=0.5_f64;let q=[0.0,(angle*0.5).sin(),0.0,(angle*0.5).cos()];
    let from=([0.075,0.025,0.5],q);
    let state=owner.start_ground_state(from).unwrap();
    let support=map.support_position_from_body(from.0,q,0.4).unwrap();
    assert!(support[0]<0.0 && from.0[0]>0.0,"centre and actual support name different columns");
    assert_eq!(state.height(),map.query_ground([support[0],support[1]],0.0).height.unwrap());
    let next=owner.project_motion(from,&state,[0.275,0.025],0.0,&mut None).unwrap();
    assert_eq!(next.pose().1,q,"missing direction must preserve complete actual attitude");
    assert!((next.pose().0[2]-from.0[2]-3.0*f64::from(Q)).abs()<1e-10);
    assert_eq!(next.ground().query_state(),GroundQueryState::Observed);
    owner.validate_projected_motion(from,&next,&[],&mut None).unwrap();
}

#[test]
fn native_unknown_carries_height_source_and_pose_without_flattening() {
    let map=scene(0,false);let owner=map.body_query();
    let a=0.08_f64;let from=([-0.025,0.025,0.45],[(a*0.5).sin(),0.0,0.0,(a*0.5).cos()]);
    let state=owner.start_ground_state(from).unwrap();
    let projection=owner.project_motion(from,&state,[1.0,0.025],0.0,&mut None).unwrap();
    assert_eq!(projection.pose().0[2],from.0[2]);assert_eq!(projection.pose().1,from.1);
    assert_eq!(projection.ground().query_state(),GroundQueryState::Unknown);
    assert_eq!(projection.ground().height(),state.height());
    // The last known source follows the traversed columns, rather than being
    // overwritten by a fabricated unknown-layer key.
    assert!(projection.ground().ground_key().is_some());
    owner.validate_projected_motion(from,&projection,&[],&mut None).unwrap();
    let unknown=projection.ground().clone();
    let again=owner.project_motion(projection.pose(),&unknown,[1.1,0.025],0.0,&mut None).unwrap();
    assert_eq!(again.ground().ground_key(),unknown.ground_key());
    assert_eq!(again.ground().height(),unknown.height());
}

#[test]
fn native_rotation_and_near_zero_translation_preserve_body_z() {
    let map=scene(0,false);let owner=map.body_query();
    let from=([-0.025,0.025,0.45],[0.04,0.0,0.0,(1.0_f64-0.04*0.04).sqrt()]);
    let state=owner.start_ground_state(from).unwrap();
    for delta in [0.0,5e-10] {
        let next=owner.project_motion(from,&state,[from.0[0]+delta,from.0[1]],0.2,&mut None).unwrap();
        assert_eq!(next.pose().0[2],from.0[2]);
    }
}

#[test]
fn native_high_member_remains_a_real_body_collision() {
    let map=scene(0,true);let from=([0.025,0.025,0.4],[0.0,0.0,0.0,1.0]);
    let observed=map.observations().unwrap();
    let record=observed.ground_records([0,0]).next().unwrap();
    let footprint=PolygonQuery::new(&map.body_envelope().unwrap().footprint_xy_m);
    for sample in [QuerySample::LegacyNominal,QuerySample::Unanchored] {
        let row=map.measured_fine_ground_value_in_context::<FineGroundColumn>([0,0],f64::from(Q),Some(record),
            map.body_envelope().unwrap(),&footprint,from.0,[0.0,0.0,1.0],sample);
        assert!(row.contacts.as_slice().iter().any(|run|run[0]<=0 && 0<=run[1]));
        assert!(!row.contacts.as_slice().iter().any(|run|run[0]<=8 && 8<=run[1]));
    }
    let owner=map.body_query();let state=owner.start_ground_state(from).unwrap();let mut witness=None;
    assert_eq!(owner.validate_motion_from_ground(from,from,&state,&[],&mut witness),
        Err(SurfaceError::InsufficientClearance));
    assert!(witness.is_some());
}

#[test]
fn native_large_step_is_rejected_by_capability_without_direction() {
    let map=scene(7,false);let owner=map.body_query();
    let from=([-0.125,0.025,0.5],[0.0,0.0,0.0,1.0]);let state=owner.start_ground_state(from).unwrap();
    assert!(matches!(owner.project_motion(from,&state,[0.125,0.025],0.0,&mut None),
        Err(SurfaceError::UntraversableSegment)));
}

#[test]
fn native_small_state_hash_ignores_unused_xy_but_keeps_carried_height_and_source() {
    use std::hash::{Hash,Hasher};
    let hash=|value:&BodyGroundState| {let mut h=std::collections::hash_map::DefaultHasher::new();value.hash(&mut h);h.finish()};
    let state=BodyGroundState {sample:GroundSample {mode:crate::surface::body_query::GroundSampleMode::Compatibility,reference_origin:[0.025,0.025,0.05],reference_normal:None,
        query_state:GroundQueryState::Unknown,reference_sources:vec![NativeGroundSource::HeightfieldFine {
            key:GroundKey::HeightfieldV1([0,0,0])}]}};
    let mut moved=state.clone();moved.sample.reference_origin[0]=8.0;
    assert_eq!(state,moved);assert_eq!(hash(&state),hash(&moved));
    moved.sample.reference_origin[2]=0.15;assert_ne!(state,moved);
    moved=state.clone();moved.sample.reference_sources=vec![NativeGroundSource::HeightfieldFine {
        key:GroundKey::HeightfieldV1([1,0,0])}];assert_ne!(state,moved);
}

#[test]
fn native_five_ten_fifteen_centimetre_relief_keeps_scalar_height_without_plane_fit() {
    for relief in [1,2,3] {
        let map=scene_with_roughness(0,false,relief);
        let record=map.observations().unwrap().ground_records([0,0]).next().unwrap();
        assert_eq!(record.members().collect::<Vec<_>>(),vec![0,relief]);
        assert_eq!(record.direction(),None);
        let h=record.height_at([0.025,0.025]).unwrap();
        let owner=map.body_query();let from=([0.025,0.025,h+0.4],[0.0,0.0,0.0,1.0]);
        let state=owner.start_ground_state(from).unwrap();
        assert_eq!(state.height(),h);
        let next=owner.project_motion(from,&state,[0.075,0.025],0.0,&mut None).unwrap();
        assert_eq!(next.pose().1,from.1);assert!((next.pose().0[2]-from.0[2]).abs()<1e-12);
        owner.validate_projected_motion(from,&next,&[],&mut None).unwrap();
    }
}

#[test]
fn native_zero_step_relief_matrix_requires_real_rigid_contact_geometry() {
    let mut results=vec![];
    for relief in [1,2,3] {
        let mut whole=fixed_go2_model();
        whole.max_step_up_m=0.0;whole.max_step_down_m=0.0;
        let map=scene_with_roughness(0,false,relief).with_body_envelope(whole).unwrap();
        let h=map.query_ground([0.025,0.025],0.0).height.unwrap();
        let from=([0.025,0.025,h+0.4],[0.0,0.0,0.0,1.0]);
        let owner=map.body_query();let state=owner.start_ground_state(from).unwrap();
        let mut witness=None;
        let result=owner.validate_motion_from_ground(from,from,&state,&[],&mut witness);
        eprintln!("H3_ZERO_STEP_RELIEF cm={} h={h:?} result={result:?} witness={witness:?}",relief*5);
        results.push(result);
    }
    assert!(results.iter().all(Result::is_ok),
        "rough member relief is not a vertical step; resolving leg contact versus rigid collision requires actual fixed primitives");
}

#[test]
fn native_unknown_keeps_last_known_step_height_within_same_segment() {
    let map=scene(3,false);let owner=map.body_query();
    let from=([-0.125,0.025,0.5],[0.0,0.0,0.0,1.0]);
    let state=owner.start_ground_state(from).unwrap();
    let next=owner.project_motion(from,&state,[1.0,0.025],0.0,&mut None).unwrap();
    assert_eq!(next.ground().query_state(),GroundQueryState::Unknown);
    assert!((next.ground().height()-state.height()-3.0*f64::from(Q)).abs()<1e-10);
    assert!((next.pose().0[2]-from.0[2]-3.0*f64::from(Q)).abs()<1e-10,
        "unknown endpoint carries the last known height; it cannot erase an earlier known step");
    owner.validate_projected_motion(from,&next,&[],&mut None).unwrap();
    let carried=owner.project_motion(next.pose(),next.ground(),[1.1,0.025],0.0,&mut None).unwrap();
    assert_eq!(carried.pose().0[2],next.pose().0[2]);
}

#[test]
fn owner_known_height_checks_actual_support_while_unknown_and_small_bias_remain_valid() {
    let native=scene(0,false);
    for map in [native.clone(),native.with_observations(None)] {
        let owner=map.body_query();let q=[0.0,0.0,0.0,1.0];
        let high=([0.025,0.025,2.0],q);
        let state=owner.start_ground_state(high).unwrap();
        assert_ne!(state.query_state(),GroundQueryState::Unknown);
        assert!(state.height()<0.1,"initial association must still return the real low ground");
        assert_eq!(owner.validate_motion_from_ground(high,high,&state,&[],&mut None),
            Err(SurfaceError::UntraversableSegment),"source h minus itself cannot validate an airborne actual body");
        for bias in [-0.05,0.0,0.05] {
            let actual=([0.025,0.025,state.height()+0.4+bias],q);
            let ground=owner.start_ground_state(actual).unwrap();
            owner.validate_motion_from_ground(actual,actual,&ground,&[],&mut None).unwrap();
        }
        let blind=([100.0,100.0,2.0],q);
        let unknown=owner.start_ground_state(blind).unwrap();
        assert_eq!(unknown.query_state(),GroundQueryState::Unknown);
        owner.validate_motion_from_ground(blind,blind,&unknown,&[],&mut None).unwrap();
    }
}

fn legacy_direction_ramp(fine_observations:bool)->SurfaceSnapshot {
    let fine=0.01_f32;
    let keys:Vec<_>=(-60..61).flat_map(|x|(-50..71).map(move|y|
        [x,y,((f64::from(x)+0.5)*0.08-0.5).floor() as i32])).collect();
    let points:Vec<_>=keys.iter().map(|p|((p[0] as f32+0.5)*fine,
        (p[1] as f32+0.5)*fine,(p[2] as f32+0.5)*fine)).collect();
    let config=Config {world_frame:"map".into(),voxel_size:0.1,robot_height:0.55,surface_closing_radius:0.0,
        node_spacing_m:0.3,wall_clearance_m:0.0,wall_buffer_m:0.1,wall_buffer_weight:2.0,
        max_slope_deg:18.0,step_threshold_m:0.15,min_step_tread_depth_m:0.2,step_penalty_weight:1.0,
        goal_tolerance:0.1,viz_publish_hz:0.0};
    let mut envelope=model();envelope.footprint_xy_m=vec![[-0.3,-0.16],[0.3,-0.16],[0.3,0.16],[-0.3,0.16]];
    envelope.z_bounds_m=[-0.15,0.15];envelope.max_step_up_m=0.15;envelope.max_step_down_m=0.15;
    let map=SurfaceSnapshot::from_points(&points,config,fine).unwrap().with_body_envelope(envelope).unwrap();
    if !fine_observations {return map;}
    let mut blocks=BTreeMap::<[i32;3],BTreeMap<[i32;3],SurfaceObservation>>::new();
    for key in keys {blocks.entry(key.map(|v|v.div_euclid(32))).or_default()
        .insert(key,SurfaceObservation {observed_ns:1,hits:2});}
    let mut observations=SurfaceObservations::new(fine,32).unwrap();
    for (block,values) in blocks {observations.replace_block(block,values).unwrap();}
    observations.mark_complete_geometry_source();map.with_observations(Some(Arc::new(observations)))
}

#[test]
fn owner_legacy_direction_comes_from_original_contact_field_and_native_none_stays_none() {
    for fine in [false,true] {
        let map=legacy_direction_ramp(fine);
        let start=map.body_pose_on_surface([0.0,0.2],0.0,[0.0,0.2,0.0]).unwrap();
        let support=map.support_position_from_body(start.0,start.1,0.4).unwrap();
        let direct=map.direct_ground_query([support[0],support[1]],support[2]);
        assert_eq!(direct.state,GroundQueryState::LegacyGeometry);
        assert_eq!(direct.normal,Some([0.0,0.0,1.0]),"legacy scalar placeholder is not the contact direction");
        let owner=map.body_query();let mut ground=owner.start_ground_state(start).unwrap();
        let direction=ground.sample.reference_normal.expect("original legacy contact field has a direction");
        assert!(direction[0].abs()>0.03);
        assert_eq!(ground.height(),direct.height.unwrap(),"direction recovery cannot replace scalar h");
        let initial=rotation(start.1).unwrap()[2];
        assert!(initial[0].abs()>0.03,"fixed input must contain real slope attitude");
        let mut actual=start;
        for yaw in [0.4,0.8,1.2] {
            let next=owner.project_motion(actual,&ground,[start.0[0],start.0[1]],yaw,&mut None).unwrap();
            assert_eq!(next.pose().0,start.0);
            let normal=rotation(next.pose().1).unwrap()[2];
            assert!((0..3).all(|axis|(normal[axis]-initial[axis]).abs()<1e-5),
                "legacy slope turn must preserve its world ground normal: fine={fine}, yaw={yaw}, before={initial:?}, after={normal:?}");
            owner.validate_projected_motion(actual,&next,&[],&mut None).unwrap();
            actual=next.pose();ground=next.ground().clone();
        }
    }
    let native=scene_with_roughness(0,false,3);let owner=native.body_query();
    let actual=([0.025,0.025,0.5],[0.0,0.05_f64.sin(),0.0,0.05_f64.cos()]);
    let state=owner.start_ground_state(actual).unwrap();
    assert_eq!(state.query_state(),GroundQueryState::Observed);
    assert_eq!(state.sample.reference_normal,None,"native missing direction must not enter legacy fitting");
    let next=owner.project_motion(actual,&state,[0.075,0.025],0.0,&mut None).unwrap();
    assert_eq!(next.ground().sample.reference_normal,None);
    assert_eq!(next.pose().1,actual.1);
}


#[test]
fn classified_empty_low_occupancy_is_unknown_not_a_fabricated_drop() {
    let map=scene_with_raw_counts_and_empty(|_,_|BTreeMap::from([(0,[0,1000,0,0,0,0,0,0])]),
        &[],Some(([1,0],-10)));
    let q=f64::from(Q);
    let a=[0.5*q,0.5*q];let b=[1.5*q,0.5*q];
    let observations=map.observations().unwrap();
    assert!(observations.has_measured_ground_layers());
    assert!(observations.classified_empty_ground([1,0]));
    assert!(observations.get([1,0,-10]).is_some());
    assert!(observations.ground_records([1,0]).next().is_none());
    let h=map.query_ground(a,0.0).height.unwrap();
    assert_eq!(h,1.5*q/8.0);
    assert_eq!(map.query_ground(b,h).state,GroundQueryState::Unknown);
    let raw_top=-9.0*q;
    assert!(h-raw_top>map.body_envelope().unwrap().max_step_down_m);
    assert!(map.read_fine_contact_bands(1,0).is_empty());
    assert!(map.known_terrain_transition_sample(b,h).is_none());
    let from=([a[0],a[1],h+0.4],[0.0,0.0,0.0,1.0]);
    let to=([b[0],b[1],from.0[2]],from.1);
    assert!(raw_top<from.0[2]+map.body_envelope().unwrap().z_bounds_m[0]);
    assert_eq!(map.validate_known_reference_motion([a[0],a[1],h],[b[0],b[1],h]),Ok(()));
    let owner=map.body_query();let state=owner.start_ground_state(from).unwrap();
    let mut witness=None;
    let projected=owner.project_motion(from,&state,b,0.0,&mut witness).unwrap();
    assert_eq!(projected.pose(),to);
    assert_eq!(projected.ground().query_state(),GroundQueryState::Unknown);
    assert_eq!(projected.ground().height(),h);
    owner.validate_projected_motion(from,&projected,&[],&mut witness).unwrap();
    assert!(witness.is_none());
    assert!(map.observations().unwrap().get([1,0,-10]).is_some());
    eprintln!("H3_EMPTY_UNKNOWN_DROP h={h:?} raw_top={raw_top:?} from={from:?} to={to:?} witness={witness:?}");
}

#[test]
fn p0_unanchored_admitted_heightfield_matches_original_core_cold_and_warm() {
    // Independently admitted Writer/Receiver owners; no hand-built HF descriptor.
    let old=scene(0,false);let new=scene(0,false);
    let pose=([0.025,0.025,0.425],[0.0,0.0,0.0,1.0]);
    assert!(old.observations.as_ref().unwrap().has_measured_ground_layers());
    assert!(!Arc::ptr_eq(&old.observations.as_ref().unwrap().legacy_reference_cache,
        &new.observations.as_ref().unwrap().legacy_reference_cache));
    for phase in ["cold","warm"] {
        let mut a=None;let mut b=None;
        let ar=old.check_body_pose_in_context_impl::<false>(pose.0,pose.1,QuerySample::Unanchored,&mut a);
        let br=new.check_body_pose_in_context_impl::<true>(pose.0,pose.1,QuerySample::Unanchored,&mut b);
        assert_eq!(ar,Ok(()));assert_eq!(br,ar);assert_eq!(a,None);assert_eq!(b,None);
        eprintln!("P0_NATIVE phase={phase} actual_height={:?} result={ar:?}",old.query_ground([0.025,0.025],0.0).height);
    }
    let view=new.body_query();
    view.validate_unanchored_motion(pose,([0.075,0.025,0.425],pose.1),&[],&mut None).unwrap();
}

// C1-c1 uses admitted records. Explicit navigation callbacks below are physical
// edge contract inputs, not a claim that the PCT producer emitted these traces.
fn c1c1_native<'a>(map:&'a SurfaceSnapshot,xy:[i32;2])->NavigationGroundSample<'a> {
    let record=map.observations().unwrap().ground_records(xy).next().unwrap();
    NavigationGroundSample::KnownNative {record,reference_xy:xy.map(|v|(f64::from(v)+0.5)*f64::from(Q))}
}

#[test]
fn c1c1_navigation_selects_actual_support_column_and_preserves_bias() {
    let map=scene(3,false);let owner=map.body_query();
    let angle=0.5_f64;let q=[0.0,(angle*0.5).sin(),0.0,(angle*0.5).cos()];
    let selected=c1c1_native(&map,[1,0]);let decode=|_|selected;
    let navigation=BodyNavigationSegment::from_recorded(&map,[0.075,0.025],[0.275,0.025],&[],&decode);
    // A real prior state on the upper side must not be subtracted from the new
    // lower initial support. Resolve t=0 without moving the supplied actual.
    let previous=owner.start_ground_state(([0.275,0.025,0.65],q)).unwrap();
    let saved=previous.clone();
    for bias in [0.0,0.04] {
        let from=([0.075,0.025,0.5+bias],q);
        let support=map.support_position_from_body(from.0,q,0.4).unwrap();
        assert!(support[0]<0.0 && from.0[0]>0.0);
        let next=owner.project_motion_with_navigation(from,&previous,[0.275,0.025],0.0,&navigation,&mut None).unwrap();
        assert_eq!(next.pose().1,q);
        assert!((next.pose().0[2]-from.0[2]-3.0*f64::from(Q)).abs()<1e-10);
        let support=map.support_position_from_body(next.pose().0,next.pose().1,0.4).unwrap();
        let xy=[support[0],support[1]];let column=xy.map(|v|(v/f64::from(Q)).floor() as i32);
        let actual=map.observations().unwrap().ground_records(column).next().unwrap();
        assert_eq!(next.ground().height(),actual.height_at(xy).unwrap());
        assert_eq!(next.ground().ground_key(),Some(actual.key()));
        assert!(actual.members().all(|z|actual.contains_member([column[0],column[1],z])));
        owner.validate_projected_motion(from,&next,&[],&mut None).unwrap();
        eprintln!("C1C1_SUPPORT bias={bias:?} actual={from:?} support={support:?} key={:?} next={:?}",actual.key(),next.pose());
    }
    let from=([0.075,0.025,0.5],q);
    let stationary=BodyNavigationSegment::from_recorded(&map,[0.075,0.025],[0.075,0.025],&[],&decode);
    let turn=owner.project_motion_with_navigation(from,&previous,[0.075,0.025],0.2,&stationary,&mut None).unwrap();
    assert_eq!(turn.pose().0,from.0);owner.validate_projected_motion(from,&turn,&[],&mut None).unwrap();
    assert_eq!(previous,saved);
}

#[test]
fn c1c1_scalar_direction_and_existing_cache_dependencies_are_distinct() {
    use crate::surface::body_query::GroundSampleMode;
    let map=scene(0,false);let owner=map.body_query();let pose=([0.025,0.025,0.425],[0.,0.,0.,1.]);
    let mut scalar=owner.start_ground_state(pose).unwrap().sample;
    scalar.mode=GroundSampleMode::NavigationScalar;
    let h=scalar.reference_origin[2];let without=scalar.clone();
    // Internal optional-direction contract. Current admitted HF direction is
    // None; this does not fabricate a new direction-producing capture fixture.
    scalar.reference_normal=Some([-0.25/1.0625_f64.sqrt(),0.,1./1.0625_f64.sqrt()]);
    assert_eq!(scalar.height_at([0.045,0.025]),h);
    assert!(GroundHeightKey::new(&scalar)==GroundHeightKey::new(&without));
    assert!(BoundPoseGroundKey::new(QuerySample::Ground(&scalar))!=BoundPoseGroundKey::new(QuerySample::Ground(&without)));
    assert_ne!(BodyGroundState{sample:scalar.clone()},BodyGroundState{sample:without.clone()});
    let mut plane=scalar.clone();plane.mode=GroundSampleMode::NavigationPlane;
    assert!((plane.height_at([0.045,0.025])-h-0.005).abs()<1e-12);
    assert!(GroundHeightKey::new(&plane)!=GroundHeightKey::new(&scalar));
    let mut old=plane.clone();old.mode=GroundSampleMode::Compatibility;old.query_state=GroundQueryState::Unknown;
    assert_eq!(old.height_at([0.045,0.025]),plane.height_at([0.045,0.025]));
    let mut same_h_other_key=without.clone();
    same_h_other_key.reference_sources=vec![NativeGroundSource::HeightfieldFine {
        key:map.observations().unwrap().ground_records([1,0]).next().unwrap().key()}];
    assert!(BoundPoseGroundKey::new(QuerySample::Ground(&without))!=BoundPoseGroundKey::new(QuerySample::Ground(&same_h_other_key)));
    for sample in [&without,&scalar,&without,&scalar] {
        map.validate_body_pose_in_context(pose.0,pose.1,QuerySample::Ground(sample),&mut None).unwrap();
    }
    let cache=map.body_query_cache.lock().unwrap();
    assert!(cache.bound_poses.contains_key(&([pose.0[0],pose.0[1],pose.0[2],pose.1[0],pose.1[1],pose.1[2],pose.1[3]].map(f64::to_bits),BoundPoseGroundKey::new(QuerySample::Ground(&without)).unwrap())));
    assert!(cache.bound_poses.contains_key(&([pose.0[0],pose.0[1],pose.0[2],pose.1[0],pose.1[1],pose.1[2],pose.1[3]].map(f64::to_bits),BoundPoseGroundKey::new(QuerySample::Ground(&scalar)).unwrap())));
}

#[test]
fn c1c1_navigation_gap_carry_and_unanchored_physical_response_stay_separate() {
    let map=scene(3,false);let owner=map.body_query();let from=([-0.125,0.025,0.5],[0.,0.,0.,1.]);
    let initial=owner.start_ground_state(from).unwrap();let original=initial.clone();
    let low=c1c1_native(&map,[-3,0]);let upper=c1c1_native(&map,[5,0]);
    let high=map.observations().unwrap().ground_records([5,0]).next().unwrap().height_at([0.275,0.025]).unwrap();
    let decode=|t|if t<0.5 {low}else{NavigationGroundSample::Carried{height:high}};
    let navigation=BodyNavigationSegment::from_recorded(&map,[-0.125,0.025],[1.0,0.025],&[0.5],&decode);
    let next=owner.project_motion_with_navigation(from,&initial,[1.0,0.025],0.0,&navigation,&mut None).unwrap();
    assert_eq!(next.ground().query_state(),GroundQueryState::Unknown);assert_eq!(next.ground().height(),high);
    assert!((next.pose().0[2]-from.0[2]-3.0*f64::from(Q)).abs()<1e-10);
    owner.validate_projected_motion(from,&next,&[],&mut None).unwrap();
    let returning=|t|if t<1.0 {NavigationGroundSample::Carried{height:high}}else{upper};
    let edge=BodyNavigationSegment::from_recorded(&map,[1.0,0.025],[0.275,0.025],&[1.0],&returning);
    let back=owner.project_motion_with_navigation(next.pose(),next.ground(),[0.275,0.025],0.0,&edge,&mut None).unwrap();
    assert_eq!(back.pose().0[2],next.pose().0[2]);assert_eq!(back.ground().query_state(),GroundQueryState::Observed);
    owner.validate_projected_motion(next.pose(),&back,&[],&mut None).unwrap();
    let unanchored=|_|NavigationGroundSample::Unanchored;
    let edge=BodyNavigationSegment::from_recorded(&map,[-0.125,0.025],[1.0,0.025],&[],&unanchored);
    let physical=owner.project_motion_with_navigation(from,&initial,[1.0,0.025],0.0,&edge,&mut None).unwrap();
    assert_eq!(physical.pose(),next.pose());assert_eq!(physical.ground().height(),high);
    assert!(matches!(unanchored(1.0),NavigationGroundSample::Unanchored));assert_eq!(initial,original);
}

#[test]
fn c1c1_same_pose_navigation_changes_survive_physical_trace() {
    let map=scene_with_native_columns(|x,y|[0,1].map(|z|
        HeightfieldColumn::from_counts([x,y],Q,BTreeMap::from([(z,[1000,0,0,0,0,0,0,0])])).unwrap().unwrap()).to_vec(),&[],None);
    let records:Vec<_>=map.observations().unwrap().ground_records([0,0]).collect();
    assert_eq!(records.len(),2);assert_ne!(records[0].key(),records[1].key());
    let low=NavigationGroundSample::KnownNative{record:records[0],reference_xy:[0.025,0.025]};
    let high=NavigationGroundSample::KnownNative{record:records[1],reference_xy:[0.025,0.025]};
    let decode=|t|if (0.25..0.75).contains(&t) {low}else{high};
    let edge=BodyNavigationSegment::from_recorded(&map,[0.025,0.025],[0.025,0.025],&[0.25,0.75],&decode);
    let actual=([0.025,0.025,0.425],[0.,0.,0.,1.]);let owner=map.body_query();
    let state=owner.start_ground_state(actual).unwrap();let original=state.clone();
    let initial=map.navigation_initial_support(actual,&state,&edge).unwrap();
    let reference=map.navigation_ground_motion_reference(actual,actual,&initial,&edge).unwrap();
    assert_eq!(reference.at(0.0).reference_sources,reference.at(1.0).reference_sources);
    assert_ne!(reference.at(0.0).reference_sources,reference.at(0.5).reference_sources);
    let binding=crate::surface::body_query::BodyQueryReference::Ground(Arc::new(reference));
    let context=QuerySegment::for_reference(&binding);assert!(!context.constant_for_test());
    let mut boundaries=Vec::new();context.append_changes(&mut boundaries);assert_eq!(boundaries,vec![0.25,0.75]);
    let mut witness=None;
    assert_eq!(owner.validate_motion_with_navigation(actual,actual,&state,&edge,&[],&mut witness),
        Err(SurfaceError::UntraversableSegment));
    assert_eq!(witness,None);assert_eq!(state,original);
    // Equal physical samples still carry a navigation boundary into the same
    // event input; this synthetic segment tests the adapter contract only.
    let constant=|_|low;let edge=BodyNavigationSegment::from_recorded(&map,[0.025,0.025],[0.025,0.025],&[0.5],&constant);
    let initial=map.navigation_initial_support(actual,&state,&edge).unwrap();
    let reference=map.navigation_ground_motion_reference(actual,actual,&initial,&edge).unwrap();
    assert!(reference.changes.iter().any(|(t,_)|*t==0.5));
}

#[test]
fn c1c1_projected_reference_is_reused_by_complete_contact_sweep() {
    let raw=scene_with_raw_counts(|_,_|BTreeMap::from([(0,[1000,0,0,0,0,0,0,0])]),&[[0,0,8]]);
    let member=scene(0,true);let recent=scene(0,false).with_recent_obstacle_points(1,1,0,&[],[(0,[0.025,0.025,0.425])]).unwrap();
    for (name,map,expected_source) in [("raw",raw,OccupancySource::Fine),("member",member,OccupancySource::Fine),
        ("recent",recent,OccupancySource::Recent)] {
        let map=map.with_body_envelope(fixed_go2_model()).unwrap();let owner=map.body_query();
        let from=([-0.45,0.025,0.425],[0.,0.,0.,1.]);let to=[0.45,0.025];let state=owner.start_ground_state(from).unwrap();
        let h=state.height();let decode=|_|NavigationGroundSample::Carried{height:h};
        let edge=BodyNavigationSegment::from_recorded(&map,[from.0[0],from.0[1]],to,&[],&decode);
        let projection=owner.project_motion_with_navigation(from,&state,to,0.0,&edge,&mut None).unwrap();
        owner.validate_motion_from_ground(from,from,&state,&[],&mut None).unwrap();
        let reference=Arc::clone(&projection.reference);let mut witness=None;
        assert_eq!(owner.validate_projected_motion(from,&projection,&[],&mut witness),Err(SurfaceError::InsufficientClearance),"{name}");
        assert!(Arc::ptr_eq(&reference,&projection.reference));
        let rejected=witness.unwrap();
        assert!(matches!(rejected.rejection,BodyRejection::Occupancy{source,..} if source==expected_source),"{name}: {rejected:?}");
        assert!(rejected.pose.0[0]>from.0[0] && rejected.pose.0[0]<to[0],"{name}: {rejected:?}");
        eprintln!("C1C1_SWEEP {name} {rejected:?}");
    }
}

#[test]
fn c1c1_recent_refresh_keeps_navigation_but_retires_physical_permission() {
    let map=scene(0,false);let owner=map.body_query();let from=([0.025,0.025,0.425],[0.,0.,0.,1.]);
    let state=owner.start_ground_state(from).unwrap();let saved=state.clone();let sample=c1c1_native(&map,[0,0]);let decode=|_|sample;
    let edge=BodyNavigationSegment::from_recorded(&map,[0.025,0.025],[0.075,0.025],&[],&decode);
    let to=([0.075,0.025,0.435],from.1);
    let next=owner.validate_motion_with_navigation(from,to,&state,&edge,&[],&mut None).unwrap();
    assert_eq!(next.height(),state.height());assert_eq!(to.0[2],0.435);
    let blocked=map.with_recent_obstacle_points(2,2,0,&[],[(0,[0.075,0.025,0.435])]).unwrap();let current=blocked.body_query();
    assert!(current.same_ground_source(&map));let mut witness=None;
    assert_eq!(current.validate_motion_with_navigation(from,to,&state,&edge,&[],&mut witness),Err(SurfaceError::InsufficientClearance));
    assert!(matches!(witness.unwrap().rejection,BodyRejection::Occupancy{source:OccupancySource::Recent,..}));
    let replaced=map.fork_for_planning();let mut witness=None;
    assert!(!replaced.body_query().same_ground_source(&map));
    assert_eq!(replaced.body_query().validate_motion_with_navigation(from,to,&state,&edge,&[],&mut witness),Err(SurfaceError::InvalidPose));
    assert_eq!(witness,None);assert_eq!(state,saved);
}
