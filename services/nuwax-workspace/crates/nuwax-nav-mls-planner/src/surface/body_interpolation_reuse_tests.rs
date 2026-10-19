use super::*;
use std::cell::Cell;
use std::collections::{BTreeMap,BTreeSet};

type Body=([f64;3],[f64;4]);
std::thread_local! {static EVENT_GEOMETRY_BUILDS:Cell<usize>=const {Cell::new(0)};}
pub(super) fn record_event_geometry_build() {
    EVENT_GEOMETRY_BUILDS.with(|count|count.set(count.get()+1));
}
fn reset_builds() {EVENT_GEOMETRY_BUILDS.with(|count|count.set(0));}
fn builds()->usize {EVENT_GEOMETRY_BUILDS.with(Cell::get)}
fn envelope(polygon:bool)->SurfaceBodyEnvelope {
    SurfaceBodyEnvelope {
        fixed_collision: None,
        footprint_xy_m:if polygon {vec![[-0.3,-0.12],[0.18,-0.18],[0.32,0.0],[0.18,0.18],[-0.3,0.12]]}
            else {vec![[-0.3,-0.16],[0.3,-0.16],[0.3,0.16],[-0.3,0.16]]},
        z_bounds_m:[-0.10,0.10],body_to_support_z_m:0.4,
        max_step_up_m:0.15,max_step_down_m:0.15,max_cross_slope_deg:18.0,support_uncertainty_m:0.0,
    }
}
fn config()->Config {
    Config {world_frame:"map".into(),voxel_size:0.1,robot_height:0.1,
        surface_closing_radius:0.0,node_spacing_m:0.3,wall_clearance_m:0.0,
        wall_buffer_m:0.0,wall_buffer_weight:0.0,max_slope_deg:18.0,step_threshold_m:0.15,
        min_step_tread_depth_m:0.2,step_penalty_weight:2.0,goal_tolerance:0.1,viz_publish_hz:0.0}
}
fn grid_map(polygon:bool)->SurfaceSnapshot {
    SurfaceSnapshot::from_points(&[(0.025,0.025,-0.025)],config(),0.05).unwrap()
        .with_body_envelope(envelope(polygon)).unwrap()
}
fn yaw_q(yaw:f64)->[f64;4] {[0.0,0.0,(yaw*0.5).sin(),(yaw*0.5).cos()]}
fn norm(q:[f64;4])->[f64;4] {let n=q.iter().map(|v|v*v).sum::<f64>().sqrt();q.map(|v|v/n)}
fn fractions_bits(value:Result<Vec<f64>,SurfaceError>)->Result<Vec<u64>,SurfaceError> {
    value.map(|values|values.into_iter().map(f64::to_bits).collect())
}

#[test]
fn interpolation_grid_matches_frozen_complete_fractions() {
    let tilting=norm([0.13,-0.09,0.22,0.96]);
    let attitudes=[([0.,0.,0.,1.],yaw_q(0.08)),([0.,0.,0.,1.],yaw_q(1e-12)),
        ([0.,0.,0.,1.],yaw_q(std::f64::consts::PI)),(tilting,yaw_q(-2.8)),
        (tilting,norm([0.130001,-0.09,0.220001,0.96])),
        ([0.0,-0.0,0.0,2.0],[0.2,-0.3,0.5,1.3]),
        (tilting,tilting.map(|v|-v)),(tilting,tilting),
        ([0.0,-0.0,0.0,1.0],[-0.0,0.0,-0.0,-1.0])];
    for polygon in [false,true] {
        let map=grid_map(polygon);
        for size in [f64::from(0.01_f32),f64::from(0.05_f32),f64::from(0.1_f32)] {
            for base in [[0.0,-0.0,0.4],[-0.1,0.1,0.42],[2.0,-1.0,0.5]] {
                for delta in [[0.0,0.0,0.0],[0.021,-0.013,0.003],[-0.1,0.2,-0.04]] {
                    for (qa,qb) in attitudes {
                        let from=(base,qa);let to=(std::array::from_fn(|i|base[i]+delta[i]),qb);
                        let expected=fractions_bits(frozen_support_grid(&map,from,to,size));
                        let actual=fractions_bits(map.body_support_grid_fractions(from,to,size));
                        assert_eq!(actual,expected,"polygon={polygon} size={size:?} from={from:?} to={to:?}");
                    }
                }
            }
        }
    }
}

fn event_bits(event:&RotationEvents)->Vec<u64> {
    let mut out=Vec::new();
    out.extend(event.from.0.map(f64::to_bits));out.extend(event.from.1.map(f64::to_bits));
    out.extend(event.to.0.map(f64::to_bits));out.extend(event.to.1.map(f64::to_bits));
    for (r,n,p) in event.samples {
        out.extend(r.into_iter().flatten().map(f64::to_bits));out.push(n.to_bits());out.extend(p.map(f64::to_bits));
    }
    out.extend([event.minimum_upward_z.to_bits(),event.normal_delta_bound.to_bits()]);
    out.extend(event.ground_normal.map(f64::to_bits));
    out.push(event.vertices.len() as u64);
    for (v,frames) in &event.vertices {
        out.extend(v.map(f64::to_bits));out.extend(frames.iter().flatten().copied().map(f64::to_bits));
    }
    out.push(event.faces.len() as u64);
    for (n,h,frames) in &event.faces {
        out.extend(n.map(f64::to_bits));out.push(h.to_bits());out.extend(frames.iter().flatten().copied().map(f64::to_bits));
    }
    out.push(event.edges.len() as u64);
    for (v,e,frames,crosses) in &event.edges {
        out.extend(v.map(f64::to_bits));out.extend(e.map(f64::to_bits));
        out.extend(frames.iter().flatten().copied().map(f64::to_bits));
        out.extend(crosses.iter().flatten().copied().map(f64::to_bits));
    }
    for bounds in [&event.vertex_bounds,&event.face_bounds,&event.edge_bounds] {
        out.push(bounds.len() as u64);
        for (low,high) in bounds {out.extend(low.map(f64::to_bits));out.extend(high.map(f64::to_bits));}
    }
    out
}
#[test]
fn interpolation_full_contact_geometry_matches_frozen_constructor() {
    for polygon in [false,true] {
        let model=envelope(polygon);
        for (qa,qb) in [([0.,0.,0.,1.],yaw_q(0.08)),(norm([0.1,-0.1,0.4,0.8]),yaw_q(-2.8)),
            ([0.0,-0.0,0.0,2.0],[0.2,-0.3,0.5,1.3]),([0.,0.,0.,1.],yaw_q(1e-12))] {
            let from=([-0.1,0.05,0.4],qa);let to=([0.15,-0.07,0.43],qb);
            let original=frozen_rotation_events(&model,from,to);
            let actual=RotationEvents::new(&model,from,to);
            assert_eq!(event_bits(&actual),event_bits(&original));
            assert!(actual.root_equations.borrow().is_empty()&&actual.frames.borrow().is_empty());
            for (low,high) in [([-0.05,-0.05,0.30],[0.05,0.05,0.40]),
                ([0.30,-0.01,0.40],[0.35,0.01,0.45]),([4.0,4.0,4.0],[4.1,4.1,4.1])] {
                let bits=|intervals:Vec<[f64;2]>|intervals.into_iter().map(|v|v.map(f64::to_bits)).collect::<Vec<_>>();
                assert_eq!(bits(actual.contacts(&model,low,high,[0.0,1.0],&[0.25,0.75])),
                    bits(original.contacts(&model,low,high,[0.0,1.0],&[0.25,0.75])));
            }
        }
    }
}

// Structural red before production: a rotating support grid used to build the
// entire body-feature geometry once, even though none of those fields were read.
#[test]
fn interpolation_support_grid_skips_unused_body_geometry() {
    let map=grid_map(false);
    let from=([0.02,-0.01,0.4],[0.,0.,0.,1.]);let to=([0.04,-0.02,0.42],yaw_q(0.3));
    reset_builds();
    let actual=map.body_support_grid_fractions(from,to,0.05).unwrap();
    assert!(!actual.is_empty());
    assert_eq!(builds(),0,"support-grid consumes only interpolation, not body feature bounds");
}
#[test]
fn interpolation_real_contacts_keep_geometry_and_translation_stays_unbuilt() {
    let map=grid_map(false);
    let from=([0.02,-0.01,0.4],[0.,0.,0.,1.]);let translated=([0.04,-0.02,0.42],from.1);
    reset_builds();map.body_support_grid_fractions(from,translated,0.05).unwrap();
    assert_eq!(builds(),0,"the original pure-translation branch already skipped rotation geometry");
    reset_builds();
    map.validate_body_contact_events(from,(translated.0,yaw_q(0.3)),|_|Ok(())).unwrap();
    assert_eq!(builds(),1,"the real rotating occupied-volume event pass must still build full geometry");
}

fn outcome(f:impl FnOnce()->Result<Vec<f64>,SurfaceError>)->String {
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)) {
        Ok(result)=>format!("return:{:?}",fractions_bits(result)),
        Err(payload)=>format!("panic:{}",payload.downcast_ref::<String>().map(String::as_str)
            .or_else(||payload.downcast_ref::<&str>().copied()).unwrap_or("non-string payload")),
    }
}
#[test]
fn interpolation_invalid_inputs_preserve_original_failure_classification() {
    let map=grid_map(false);let q=[0.,0.,0.,1.];
    for (from,to) in [
        (([0.,0.,0.4],[0.;4]),([0.1,0.,0.4],q)),
        (([0.,0.,0.4],[f64::NAN,0.,0.,1.]),([0.1,0.,0.4],q)),
        (([0.,0.,0.4],q),([0.1,0.,0.4],[0.,0.,f64::INFINITY,1.])),
        (([f64::NAN,0.,0.4],q),([0.1,0.,0.4],q)),
        (([0.,0.,0.4],q),([f64::INFINITY,0.,0.4],q)),
        (([0.,0.,0.4],[0.,0.,0.,1e300]),([0.1,0.,0.4],[0.,0.,1e300,1e300])),
        (([0.,0.,0.4],[0.,0.,0.,1e-300]),([0.1,0.,0.4],[0.,0.,1e-300,1e-300])),
    ] {
        let expected=outcome(||frozen_support_grid(&map,from,to,0.05));
        let actual=outcome(||map.body_support_grid_fractions(from,to,0.05));
        assert_eq!(actual,expected,"from={from:?} to={to:?}");
    }
    let no_model=SurfaceSnapshot::from_points(&[(0.025,0.025,-0.025)],config(),0.05).unwrap();
    let from=([0.,0.,0.4],q);let to=([0.1,0.,0.4],yaw_q(0.1));
    assert_eq!(fractions_bits(no_model.body_support_grid_fractions(from,to,0.05)),
        fractions_bits(frozen_support_grid(&no_model,from,to,0.05)));
}

fn actual_scene(kind:usize)->SurfaceSnapshot {
    let source=0.05_f32;let fine=f64::from(source);let edge=nuwax_map_chunks::BLOCK_SIDE;
    let mut keys=BTreeSet::new();
    for x in -18_i32..=18 {for y in -12_i32..=12 {
        if kind==2&&(-1..=0).contains(&x)&&(-1..=0).contains(&y) {continue;}
        let z=if kind==1 {x.div_euclid(8)-1}else{-1};keys.insert([x,y,z]);
        if kind==7 {keys.insert([x,y,9]);}
    }}
    if kind==3 {for y in -2..=2 {for z in 4..=9 {keys.insert([0,y,z]);}}}
    let points:Vec<_>=keys.iter().map(|key|(((f64::from(key[0])+0.5)*fine) as f32,
        ((f64::from(key[1])+0.5)*fine) as f32,((f64::from(key[2])+0.5)*fine) as f32)).collect();
    let mut blocks=BTreeMap::<[i32;3],BTreeMap<[i32;3],SurfaceObservation>>::new();
    for &key in &keys {blocks.entry(key.map(|v|v.div_euclid(edge))).or_default()
        .insert(key,SurfaceObservation{observed_ns:0,hits:0});}
    let mut observed=SurfaceObservations::new(source,edge).unwrap();
    for (block,values) in blocks {observed.replace_block(block,values).unwrap();}
    if kind==6 {
        use nuwax_map_chunks::{LayerObservation,LayerObservations,LayerSample,block_key,local_key};
        let mut layers=BTreeMap::<[i32;3],LayerObservations>::new();
        for &key in &keys {
            let local=local_key(key);let point=key.map(|v|(f64::from(v)+0.5)*fine);
            layers.entry(block_key(key)).or_default().insert(local,LayerObservation {
                members:vec![local],normal:[0.,0.,1.],plane_origin:point,
                samples:vec![LayerSample{source:Default::default(),point,source_sequence:1,
                    source_timestamp_ns:1,point_index:u32::from(local)}],
            });
        }
        for (block,layers) in layers {observed.replace_ground_layers(block,Arc::new(layers)).unwrap();}
    }
    observed.mark_complete_geometry_source();
    let mut map=SurfaceSnapshot::from_points(&points,config(),source).unwrap()
        .with_body_envelope(envelope(kind%2==1)).unwrap().with_observations(Some(Arc::new(observed)));
    if kind==4 {map=map.with_observations(None);}
    if kind==5 {map=map.with_recent_obstacle_points(1,100,0,&[],[(0,[0.0,0.0,0.4])]).unwrap();}
    map
}
fn witness_bits(witness:Option<BodyFailureWitness>)->String {
    witness.map_or_else(||"none".into(),|w|match w.rejection {
        BodyRejection::Occupancy{cell,source,z_bounds,ground_height}=>format!("{:?}/{:?}/{cell:?}/{source:?}/{:?}/{:?}",
            w.pose.0.map(f64::to_bits),w.pose.1.map(f64::to_bits),z_bounds.map(f64::to_bits),ground_height.map(f64::to_bits)),
    })
}
fn pose_bits(pose:Body)->([u64;3],[u64;4]) {(pose.0.map(f64::to_bits),pose.1.map(f64::to_bits))}
// Identical source runs against tests-only and candidate binaries. This is a real
// public pose/projection/sweep trace, not a frozen helper pretending to be a sweep.
#[test]
#[ignore="explicit serial before/after real-output comparison"]
fn interpolation_actual_motion_and_witness_trace() {
    let q=[0.,0.,0.,1.];
    let boundary=[[-1.0,-0.8],[1.0,-0.8],[1.0,0.8],[-1.0,0.8]];
    let concave=[[-1.0,-0.8],[1.0,-0.8],[1.0,0.8],[0.1,0.8],[0.1,-0.1],[-0.1,-0.1],[-0.1,0.8],[-1.0,0.8]];
    // Explicitly qualify one clear motion, one interior obstacle witness and
    // one independent concave-boundary rejection before collecting the trace.
    let from=([-0.4,0.0,0.4],q);let to=([0.4,0.0,0.4],yaw_q(0.3));
    let clear=actual_scene(0);
    assert_eq!(clear.validate_body_motion(from,to,&[]),Ok(()));
    assert_eq!(clear.validate_body_motion(from,to,&concave),Err(SurfaceError::InvalidBoundary));
    let blocked=actual_scene(3);
    assert_eq!(blocked.validate_body_pose(from.0,from.1),Ok(()));
    assert_eq!(blocked.validate_body_pose(to.0,to.1),Ok(()));
    let mut witness=None;
    assert!(blocked.validate_body_motion_with_witness(from,to,&[],&mut witness).is_err());
    assert!(witness.is_some(),"interior occupancy must produce an actual first rejection witness");
    for kind in 0..8 {for warm in [false,true] {
        let map=actual_scene(kind).fork_for_planning();
        let motions=[(([-0.4,0.,0.4],q),([0.4,0.,0.4],yaw_q(0.3))),
            (([0.0,0.0,0.4],q),([0.0,0.0,0.4],yaw_q(1.4))),
            (([-0.1,0.05,0.4],norm([0.05,-0.03,0.0,1.0])),([0.12,-0.04,0.42],norm([0.06,-0.02,0.1,1.0])))];
        for (index,(from,to)) in motions.into_iter().enumerate() {
            if warm {let _=map.validate_body_pose(from.0,from.1);let _=map.validate_body_motion(from,to,&[]);}
            let mut witness=None;let result=map.validate_body_pose_with_witness(from.0,from.1,&mut witness);
            eprintln!("INTERPOLATION_TRACE kind={kind} warm={warm} case={index} query=pose input={:?} result={result:?} witness={}",pose_bits(from),witness_bits(witness));
            let mut witness=None;let target_yaw=rotation(to.1).unwrap()[0][1].atan2(rotation(to.1).unwrap()[0][0]);
            let result=map.body_pose_after_motion_with_witness(from,[to.0[0],to.0[1]],target_yaw,&mut witness).map(pose_bits);
            eprintln!("INTERPOLATION_TRACE kind={kind} warm={warm} case={index} query=project input={:?}/{:?} result={result:?} witness={}",pose_bits(from),pose_bits(to),witness_bits(witness));
            for (bounded,bounds) in [&[][..],boundary.as_slice(),concave.as_slice()].into_iter().enumerate() {
                let mut witness=None;let result=map.validate_body_motion_with_witness(from,to,bounds,&mut witness);
                eprintln!("INTERPOLATION_TRACE kind={kind} warm={warm} case={index} query=motion bounded={bounded} input={:?}/{:?} result={result:?} witness={}",pose_bits(from),pose_bits(to),witness_bits(witness));
            }
        }
    }}
}

// Frozen before-proposal implementations. Do not rewrite them to share the candidate helper.
fn frozen_rotation_events(model:&SurfaceBodyEnvelope,from:([f64;3],[f64;4]),to:([f64;3],[f64;4]))->RotationEvents {
        let norm=|q:[f64;4]| {let n=q.iter().map(|v|v*v).sum::<f64>().sqrt();q.map(|v|v/n)};
        let qa=norm(from.1);let mut qb=norm(to.1);
        if qa.iter().zip(qb).map(|(a,b)|a*b).sum::<f64>()<0.0 {qb=qb.map(|v|-v);}
        let samples:[([[f64;3];3],f64,[f64;3]);4]=std::array::from_fn(|i| {
            let t=i as f64/3.0;
            let q=std::array::from_fn(|j|qa[j]*(1.0-t)+qb[j]*t);
            let n=q.iter().map(|v|v*v).sum::<f64>();
            let r=rotation(q).expect("已验证四元数插值").map(|a|a.map(|v|v*n));
            (r,n,std::array::from_fn(|j|from.0[j]*(1.0-t)+to.0[j]*t))
        });
        // 以中间姿态覆盖整段旋转。单位四元数弦长等于2*sin(转角/4)，
        // 乘特征半径即可界定其相对中间姿态的最大位移，微小真实转角也保留。
        let middle=rotation(std::array::from_fn(|i|qa[i]+qb[i])).expect("已验证最短插值");
        let angular_bound=qa.iter().zip(qb).map(|(a,b)|(a-b).powi(2)).sum::<f64>().sqrt();
        let displacement:[f64;3]=std::array::from_fn(|i|to.0[i]-from.0[i]);
        let sweep_bounds=|vertices:&[[f64;3]]| {
            let padding=vertices.iter().map(|v|dot(*v,*v).sqrt()).fold(0.0,f64::max)*angular_bound;
            let mut min=[f64::INFINITY;3];let mut max=[f64::NEG_INFINITY;3];
            for &v in vertices {let point=world(middle,from.0,v);for i in 0..3 {
                min[i]=min[i].min(point[i]+displacement[i].min(0.0)-padding);
                max[i]=max[i].max(point[i]+displacement[i].max(0.0)+padding);
            }}
            (min,max)
        };
        let local_vertices:Vec<_>=model.footprint_xy_m.iter().flat_map(|p|model.z_bounds_m.map(|z|[p[0],p[1],z])).collect();
        let vertex_bounds=local_vertices.iter().map(|&v|sweep_bounds(&[v])).collect();
        let vertices=local_vertices.into_iter().map(|v|(v,samples.map(|(r,_,_)|world(r,[0.0;3],v)))).collect();
        let mut face_bounds:Vec<_>=model.z_bounds_m.into_iter().map(|z|sweep_bounds(
            &model.footprint_xy_m.iter().map(|p|[p[0],p[1],z]).collect::<Vec<_>>())).collect();
        let mut faces=vec![([0.0,0.0,1.0],model.z_bounds_m[0]),([0.0,0.0,1.0],model.z_bounds_m[1])];
        let mut edges=Vec::new();
        for (a,b) in model.footprint_xy_m.iter().zip(model.footprint_xy_m.iter().cycle().skip(1)) {
            let e=[b[0]-a[0],b[1]-a[1],0.0];let n=[e[1],-e[0],0.0];
            faces.push((n,n[0]*a[0]+n[1]*a[1]));
            face_bounds.push(sweep_bounds(&[a,b].into_iter().flat_map(|p|
                model.z_bounds_m.map(|z|[p[0],p[1],z])).collect::<Vec<_>>()));
            for z in model.z_bounds_m {edges.push(([a[0],a[1],z],e));}
            edges.push(([a[0],a[1],model.z_bounds_m[0]],[0.0,0.0,model.z_bounds_m[1]-model.z_bounds_m[0]]));
        }
        // 四帧特征变换只属于本次机体运动，与查询哪个占据体素无关。
        let faces=faces.into_iter().map(|(n,h)|(n,h,samples.map(|(r,_,_)|world(r,[0.0;3],n)))).collect();
        let edge_bounds=edges.iter().map(|&(v,e)|sweep_bounds(&[v,std::array::from_fn(|i|v[i]+e[i])])).collect();
        let edges=edges.into_iter().map(|(v,e)|(v,e,samples.map(|(r,_,_)|world(r,[0.0;3],e)),
            samples.map(|(r,_,_)|world(r,[0.0;3],cross(v,e))))).collect();
        RotationEvents {samples,vertices,faces,edges,minimum_upward_z:minimum_upward_z(qa,qb),
            ground_normal:rotation(qa).expect("已验证实际姿态")[2],normal_delta_bound:2.0*angular_bound,
            root_equations:Default::default(),frames:Default::default(),
            vertex_bounds,face_bounds,edge_bounds,from:(from.0,qa),to:(to.0,qb)}
    }

fn frozen_support_grid(map:&SurfaceSnapshot,from:([f64;3],[f64;4]),to:([f64;3],[f64;4]),size:f64)
        -> Result<Vec<f64>,SurfaceError> {
        let model=map.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        if rotation(from.1)?==rotation(to.1)? {
            let a=map.support_position_from_body(from.0,from.1,model.body_to_support_z_m)?;
            let b=map.support_position_from_body(to.0,to.1,model.body_to_support_z_m)?;
            return reference_grid_fractions(a,b,size);
        }
        let events=frozen_rotation_events(model,from,to);
        let point=[0.0,0.0,-model.body_to_support_z_m];
        let middle=rotation(std::array::from_fn(|i|events.from.1[i]+events.to.1[i]))?;
        let centre=world(middle,from.0,point);
        let padding=model.body_to_support_z_m*events.from.1.iter().zip(events.to.1)
            .map(|(a,b)|(a-b).powi(2)).sum::<f64>().sqrt();
        let mut times=vec![0.0,1.0];
        for axis in 0..3 {
            let displacement=to.0[axis]-from.0[axis];
            let first=((centre[axis]+displacement.min(0.0)-padding)/size).floor() as i32+1;
            let last=((centre[axis]+displacement.max(0.0)+padding)/size).floor() as i32;
            for index in first..=last {
                let boundary=f64::from(index)*size;
                cubic_event_roots(events.samples.map(|(r,n,p)|world(r,[0.0;3],point)[axis]+(p[axis]-boundary)*n),[0.0,1.0],&mut times);
            }
        }
        Ok(grid_interval_samples(times))
    }
