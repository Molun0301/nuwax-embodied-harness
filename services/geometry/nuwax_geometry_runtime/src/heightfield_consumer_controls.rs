//! Real capture/Reader controls for the unified native-body consumer candidate.
//! Intended inclusion: a cfg(test) child of pointcloud::fine_cloud.
//! No producer logic, Reader assembly, native counts, or dependency is copied.
use super::*;
use crate::admitted_cloud::AcceptedMappingCloud;
use crate::lcm_runtime::mapping_observation::RegisteredObservation;
use nalgebra::Isometry3;
use nuwax_codec::lcm::schema;
use nuwax_map_chunks::{ColumnGroundRecords, Domain, LayerSource, Receiver, Repository, SealedMap};
use nuwax_nav_mls_planner::{
    mls_planner::Config, SurfaceBodyEnvelope, SurfaceError, SurfaceSnapshot,
};
use nuwax_nav_mls_planner::surface::{BodyRejection, GroundQueryState, OccupancySource};
use nuwax_nav_stack::{surface_config::SurfaceNavigationConfig, surface_map::SurfaceMapCache};
use nuwax_nav_types_messages::RobotGeometry;
use std::collections::{BTreeMap, BTreeSet};

const Q: f32 = 0.05;
type Body = ([f64; 3], [f64; 4]);
fn q() -> f64 { f64::from(Q) }
fn hbin(bin: i32) -> f64 { (f64::from(bin) + 0.5) * (q() / 8.0) }
fn field(z: impl Fn(i32, i32) -> Vec<f64>) -> Vec<Point3<f64>> {
    (-4..=4).flat_map(|x| (-4..=4).flat_map({ let z=&z; move |y| {
        z(x,y).into_iter().map(move |h| Point3::new(
            (f64::from(x)+0.5)*q(), (f64::from(y)+0.5)*q(), h))
    }})).collect()
}
fn admitted(points: Vec<Point3<f64>>, stamp: u64, sequence: u64) -> AcceptedMappingCloud {
    let indices=(0..points.len()).map(|i| i as u32*3+4).collect();
    AcceptedMappingCloud::auxiliary(RegisteredObservation::for_test_source(
        stamp, sequence, LayerSource::Auxiliary {
            sensor_id:"h3-capture-consumer-control".into(), clock_id:"device".into(),
            runtime_generation:1, session_generation:1,
        }, points, indices, Isometry3::identity()))
}
fn ingest(fine:&mut FineCloud, source:&AcceptedMappingCloud, first:usize) {
    let suffix=&source.map_points()[first..];
    assert!(fine.accumulate(suffix,None,source.source().timestamp_ns(),Some(source)).unwrap());
    // This is the actual owner-index output. Auxiliary does not manufacture a LiDAR scan.
    assert!(fine.latest_unclassified_scan().is_none());
}
fn assert_indices(fine:&FineCloud, source:&AcceptedMappingCloud, retained:impl Iterator<Item=usize>) {
    let identity=source.source();
    let expected:Vec<_>=retained.map(|i|identity.original_index(i).unwrap()).collect();
    assert_eq!(fine.latest_ground_indices.as_ref(),expected.as_slice());
}
fn records(fine:&mut FineCloud) -> BTreeMap<[i32;2],ColumnGroundRecords> {
    // Existing bounded test observer reads the original writer, not rebuilt counts.
    super::owner_test_observations::snapshot(fine).1
}
fn assert_occupancy(fine:&mut FineCloud, points:impl Iterator<Item=Point3<f64>>) {
    let expected:BTreeSet<_>=points.map(|p| {
        let (x,y,z)=voxel_key([p.x,p.y,p.z],Q).unwrap();[x,y,z]
    }).collect();
    let actual=super::owner_test_observations::snapshot(fine).0.into_iter()
        .map(|(key,_)|key).collect::<BTreeSet<_>>();
    assert_eq!(actual,expected,"producer occupancy must be the actual admitted returns");
}
fn assert_current_samples(fine:&mut FineCloud, source:&AcceptedMappingCloud) {
    let identity=source.source();
    let columns=records(fine);
    let mut n=0;
    for sample in columns.values().flat_map(|c|&c.heightfields).flat_map(|f|&f.samples)
        .filter(|s|s.source_timestamp_ns==identity.timestamp_ns()
            && s.source_sequence==identity.sequence().unwrap()) {
        n+=1;
        assert_eq!(sample.source.as_ref(),identity.layer_source().as_ref());
        assert!(sample.point_index>=4 && (sample.point_index-4)%3==0);
        let i=((sample.point_index-4)/3) as usize;
        assert_eq!(identity.original_index(i),Some(sample.point_index));
        assert_eq!(sample.point,<[f64;3]>::from(source.map_points()[i].coords));
    }
    assert!(n>0,"bounded diagnostics must preserve some original samples");
}
fn fixed_model(max_step:f64) -> SurfaceBodyEnvelope {
    // Exact B Go2 fixture. Parsing an existing public RobotGeometry type avoids
    // a new geometry-runtime dependency on nuwax-types.
    let robot=RobotGeometry::from_runtime_json(r#"{
      "footprint_xy_m":[[-0.24,-0.164],[0.34,-0.164],[0.34,0.164],[-0.24,0.164]],
      "z_min_m":-0.448,"z_max_m":0.057,"source":"robot_default_urdf_collision",
      "self_collision":{"model_base_link":"base","excluded_moving_collision_count":24,
        "unsupported_collision_count":0,"primitives":[
          {"link_name":"base","shape":"box","translation_m":[0,0,0],
           "rotation_row_major":[1,0,0,0,1,0,0,0,1],"size_m":[0.3762,0.0935,0.114]},
          {"link_name":"Head_upper","shape":"cylinder","translation_m":[0.285,0,0.01],
           "rotation_row_major":[1,0,0,0,1,0,0,0,1],"radius_m":0.05,"length_m":0.09},
          {"link_name":"Head_lower","shape":"sphere","translation_m":[0.293,0,-0.06],
           "rotation_row_major":[1,0,0,0,1,0,0,0,1],"radius_m":0.047}]}}
    "#).unwrap();
    SurfaceBodyEnvelope {fixed_collision:robot.self_collision,footprint_xy_m:robot.footprint_xy_m,
        z_bounds_m:[robot.z_min_m,robot.z_max_m],body_to_support_z_m:0.4,
        max_step_up_m:max_step,max_step_down_m:max_step,max_cross_slope_deg:18.0,support_uncertainty_m:0.0}
}
fn config(max_step:f64) -> SurfaceNavigationConfig {
    let mut c=SurfaceNavigationConfig::new("map".into(),0.4,0.328,0.2,0.1,0.21,18.0,0.2).unwrap();
    // Exact B scene geometry config; zero-step control changes the body limits only,
    // as does native_zero_step_relief_matrix_requires_real_rigid_contact_geometry.
    c.geometry=Config {world_frame:"map".into(),voxel_size:0.1,robot_height:0.2,
        surface_closing_radius:0.0,node_spacing_m:0.3,wall_clearance_m:0.0,
        wall_buffer_m:0.2,wall_buffer_weight:2.0,max_slope_deg:18.0,
        step_threshold_m:0.21,min_step_tread_depth_m:0.2,step_penalty_weight:2.0,
        goal_tolerance:0.1,viz_publish_hz:0.0};
    c.body_envelope=Some(fixed_model(max_step));
    c
}
fn message(root:&SealedMap) -> schema::NavigationMap3D {
    let manifest=root.manifest();
    let revision=i64::try_from(manifest.revision).unwrap();
    let stamp=i64::try_from(manifest.source_timestamp_ns).unwrap();
    let transfer = crate::lcm_runtime::prepare_navigation_map_transfer_for_test(root).unwrap();
    assert!(transfer.included.values().map(Vec::len).sum::<usize>() <= 128 * 1024);
    schema::NavigationMap3D {
        schema_version:schema::NAVIGATION_MAP_3D_SCHEMA_VERSION,
        producer_runtime_generation:manifest.domain.producer_generation as i64,
        committed_map:schema::CommittedMapIdentity {
            session_id:manifest.domain.session.clone(),session_generation:manifest.domain.generation as i64,
            revision,commit_boot_nonce:[0x63;16],commit_sequence:revision,
            source_timestamp_ns:stamp,frame_id:"map".into(),
        },
        map_frame_generation:manifest.domain.frame_generation as i64,
        cloud_revision:revision,cloud_occupancy_revision:revision,source_timestamp_ns:stamp,
        source_voxel_size:manifest.voxel_size_m,commit_boot_nonce:[0x63;16],commit_sequence:revision,
        cleared:false,virtual_obstacles:None,
        fine:Some(transfer),
        cloud:schema::PointCloud2 {
            header:schema::Header {seq:revision as i32,stamp_sec:(stamp/1_000_000_000) as i32,
                stamp_nsec:(stamp%1_000_000_000) as i32,frame_id:"map".into()},
            height:1,width:0,point_step:0,row_step:0,fields:vec![],data:vec![],
            ..Default::default()
        },
    }
}
fn reader(fine:&mut FineCloud, max_step:f64, stamp:u64) -> Arc<SurfaceSnapshot> {
    let before=records(fine);
    let sealed=fine.seal(Domain {session:"h3-capture-consumer-control".into(),generation:1,
        frame_generation:1,producer_generation:1},1,stamp).unwrap();
    assert_eq!(sealed.manifest().version,5);
    let message=message(&sealed);
    schema::validate_navigation_map_3d(&message).unwrap();
    // The production publication deliberately has a bounded inline payload.
    // Fetch only missing immutable objects through the original Receiver and
    // source Repository, then hand that same admitted receiver to the Reader.
    // This is in-process Store coverage; outer RPC framing is tested separately.
    let transfer=message.fine.as_ref().unwrap();
    let mut receiver=Receiver::with_cache_budget(Repository::temporary().unwrap(),32*1024*1024);
    let mut fetched=Vec::new();
    let admitted=receiver.receive(transfer,&sealed.manifest().domain,|root,reference| {
        assert_eq!(root,sealed.manifest().as_ref());
        assert!(!transfer.included.contains_key(&reference.object));
        fetched.push(reference.object);
        sealed.repository.read_object(reference.object,nuwax_map_chunks::MAX_BLOCK_BYTES)
    }).unwrap();
    assert_eq!(admitted.as_ref(),sealed.manifest().as_ref());
    let wanted:BTreeSet<_>=sealed.manifest().blocks.iter()
        .filter(|r|!transfer.included.contains_key(&r.object)).map(|r|r.object).collect();
    assert_eq!(fetched.iter().copied().collect::<BTreeSet<_>>(),wanted);
    assert_eq!(fetched.len(),wanted.len());
    eprintln!("CAPTURE_READER_TRANSFER blocks={} inline_objects={} inline_bytes={} fetched_objects={}",
        sealed.manifest().blocks.len(),transfer.included.len(),
        transfer.included.values().map(Vec::len).sum::<usize>(),fetched.len());
    let mut cache=SurfaceMapCache::configured_with_admitted_fine_source(&config(max_step),receiver);
    let map=cache.build(&message).unwrap();
    let again=cache.build(&message).unwrap();
    assert!(Arc::ptr_eq(&map,&again),"same Reader publication returns its original Arc");
    let observed=map.observations().expect("configured Reader must enable original observations");
    assert!(observed.is_complete_geometry_source());
    assert!(Arc::ptr_eq(observed,again.observations().unwrap()));
    for (xy,column) in before {
        assert!(column.legacy.is_empty());
        let got:Vec<_>=observed.ground_records(xy).collect();
        assert_eq!(got.len(),column.heightfields.len());
        for original in column.heightfields {
            let members:Vec<_>=original.column.members().map(|(z,_)|z).collect();
            let received=got.iter().find(|r|r.members().eq(members.iter().copied())).unwrap();
            assert!(received.is_heightfield());
            assert_eq!(received.height_at(xy.map(|v|(f64::from(v)+0.5)*q())),Some(original.column.height()));
            assert_eq!(received.occupied_bounds(q()),original.column.occupied_bounds());
            assert_eq!(received.direction(),None);
            assert_eq!(received.scatter(),None);
        }
    }
    // Do not call with_observations/from_points or clone the snapshot here.
    map
}
fn allow_static(map:&SurfaceSnapshot, pose:Body) {
    let query=map.body_query();let ground=query.start_ground_state(pose).unwrap();
    assert_eq!(ground.query_state(),GroundQueryState::Observed);
    query.validate_motion_from_ground(pose,pose,&ground,&[],&mut None).unwrap();
}
fn reject_static(map:&SurfaceSnapshot, pose:Body, wanted:BTreeSet<[i32;3]>) {
    let query=map.body_query();let ground=query.start_ground_state(pose).unwrap();
    assert_eq!(ground.query_state(),GroundQueryState::Observed,
        "negative must reach collision checking from a real known floor");
    let mut witness=None;
    assert_eq!(query.validate_motion_from_ground(pose,pose,&ground,&[],&mut witness),
        Err(SurfaceError::InsufficientClearance));
    match witness.expect("must be an actual intersection, not a generic failure").rejection {
        BodyRejection::Occupancy {cell,source,..} => {
            assert!(wanted.contains(&cell),"unexpected rejection voxel {cell:?}");
            assert!(matches!(source,OccupancySource::Fine|OccupancySource::FineFringe));
        }
    }
}

#[test]
fn capture_reader_thick_bands_use_actual_height_and_finite_zero_step_contact() {
    for (thickness,last_member,height_bin) in [(0.05,1,5),(0.10,2,9),(0.15,3,13)] {
        let source=admitted(field(|_,_|(0..25).map(|k|0.01+thickness*f64::from(k)/24.0).collect()),20,0);
        let mut fine=FineCloud::fresh().unwrap();
        ingest(&mut fine,&source,0);
        assert_occupancy(&mut fine,source.map_points().iter().copied());
        assert_indices(&fine,&source,0..source.map_points().len());
        assert_eq!(fine.latest_ground_indices.len(),2025);
        assert_current_samples(&mut fine,&source);
        let columns=records(&mut fine);
        assert_eq!(columns.len(),81);
        for c in columns.values() {
            assert!(c.legacy.is_empty());assert_eq!(c.heightfields.len(),1);
            let h=&c.heightfields[0].column;
            assert_eq!(h.members().map(|(z,_)|z).collect::<Vec<_>>(),(0..=last_member).collect::<Vec<_>>());
            assert_eq!(h.height(),hbin(height_bin),"producer expectation before any consumer");
        }
        let map=reader(&mut fine,0.0,20);
        let h=map.query_ground([0.025,0.025],0.0).height.unwrap();
        assert_eq!(h,hbin(height_bin));
        allow_static(&map,([0.025,0.025,h+0.4],[0.0,0.0,0.0,1.0]));
    }
}

#[test]
fn capture_reader_original_short_vertical_suffix_keeps_rigid_positive_and_negative() {
    // Unchanged original source and retained suffix from FineCloud's historical test.
    let mut points:Vec<_>=(0..8).flat_map(|x|(0..8).map(move|y|
        Point3::new(0.008+f64::from(x)*0.004,0.008+f64::from(y)*0.004,0.01))).collect();
    points.extend((0..8).flat_map(|y|(0..8).map(move|z|
        Point3::new(0.28,0.008+f64::from(y)*0.02,0.21+f64::from(z)*0.02))));
    let source=admitted(points,10,0);let mut fine=FineCloud::fresh().unwrap();
    ingest(&mut fine,&source,8);
    assert_occupancy(&mut fine,source.map_points()[8..].iter().copied());
    assert_indices(&fine,&source,8..128);assert_eq!(fine.latest_ground_indices.len(),120);
    assert_current_samples(&mut fine,&source);
    let columns=records(&mut fine);
    assert_eq!(columns.keys().copied().collect::<BTreeSet<_>>(),BTreeSet::from([[0,0],[5,0],[5,1],[5,2]]));
    assert_eq!(columns[&[0,0]].heightfields[0].column.height(),hbin(1));
    for xy in [[5,0],[5,1],[5,2]] {
        assert_eq!(columns[&xy].heightfields.len(),1);
        let c=&columns[&xy].heightfields[0].column;
        assert_eq!(c.members().map(|(z,_)|z).collect::<Vec<_>>(),vec![4,5,6]);
        assert_eq!(c.height(),hbin(44));
    }
    let map=reader(&mut fine,0.21,10);
    // Same finite source and body position. Choose x=0.0 explicitly: at yaw pi,
    // whole-body max X is .24, below the obstacle voxel's .25 lower face.
    // At yaw zero, original Head_lower sphere center is (.293,.025,h+.34),
    // and actual source return (.28,.028,.35) lies strictly inside r=.047.
    let xyz=[0.0,0.025,hbin(1)+0.4];
    allow_static(&map,(xyz,[0.0,0.0,1.0,0.0]));
    let actual_top=source.map_points()[64+8+7];
    let lower_head_center=[xyz[0]+0.293,xyz[1],xyz[2]-0.06];
    let separation_squared=(actual_top.x-lower_head_center[0]).powi(2)
        +(actual_top.y-lower_head_center[1]).powi(2)+(actual_top.z-lower_head_center[2]).powi(2);
    assert!(separation_squared<0.047_f64.powi(2),"original source point lies inside original sphere");
    // The low-floor reference need not select this elevated source column as
    // contact. A lower whole-body obstacle may short-circuit before the head hit.
    // Do not pretend the witness proves which fixed primitive was checked first.
    let occupied_short_structure=(0..3).flat_map(|y|(4..=6).map(move|z|[5,y,z])).collect();
    reject_static(&map,(xyz,[0.0,0.0,0.0,1.0]),occupied_short_structure);
}

#[test]
fn capture_reader_mixed_voxel_and_actual_trunk_return_remain_obstacles() {
    let baseline=admitted(field(|_,_|vec![0.01]),19,0);
    for (name,extra) in [("mixed_original",0.041),("actual_trunk",0.410)] {
        let mut fine=FineCloud::fresh().unwrap();ingest(&mut fine,&baseline,0);
        let source=admitted(field(|x,y|if x==0&&y==0 {vec![0.010,extra]}else{vec![0.01]}),20,1);
        ingest(&mut fine,&source,0);
        assert_occupancy(&mut fine,baseline.map_points().iter().chain(source.map_points()).copied());
        if name=="mixed_original" {
            assert_indices(&fine,&source,(0..82).filter(|i|*i!=40&&*i!=41));
            assert_eq!(fine.latest_ground_indices.len(),80);
            assert!(fine.writer.ground_column([0,0]).unwrap().heightfields.is_empty());
        } else {
            assert_indices(&fine,&source,0..82);
            let c=fine.writer.ground_column([0,0]).unwrap();
            assert_eq!(c.heightfields.len(),2,"real capture yields TWO surfaces, not B's hand-grouped high member");
            assert!(c.heightfields.iter().any(|f|f.column.members().map(|(z,_)|z).eq([8])));
        }
        assert_current_samples(&mut fine,&source);
        let key=if name=="mixed_original" {[0,0,0]}else{[0,0,8]};
        assert!(!fine.writer.region(key,key,1).unwrap().is_empty());
        let map=reader(&mut fine,0.21,20);
        // Actual support lies on neighboring known low floor; no Unknown gate
        // or elevated-support error can masquerade as the collision negative.
        let pose=([-0.025,0.025,hbin(1)+0.4],[0.0,0.0,0.0,1.0]);
        if name=="actual_trunk" {
            let point=source.map_points()[41];
            assert!((point.x-pose.0[0]).abs()<0.3762/2.0
                &&(point.y-pose.0[1]).abs()<0.0935/2.0
                &&(point.z-pose.0[2]).abs()<0.114/2.0,
                "the actual extra return lies strictly inside the unchanged base box");
        }
        reject_static(&map,pose,BTreeSet::from([key]));
    }
}
