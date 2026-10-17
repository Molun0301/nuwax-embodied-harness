use super::*;

#[test]
fn recent_surface_box_top_without_map_commit_remains_collision_and_floor_noise_is_clear() {
    let map=scene(&[],false);let q=[0.,0.,0.,1.];let pose=([0.05,0.05,0.5],q);
    let floor=map.with_recent_obstacle_points(1,100,0,&[0],[(0,[0.05,0.05,0.04])]).unwrap();
    assert!(map.same_geometry(&floor));assert!(!map.same_body_query_view(&floor));
    floor.validate_body_motion(pose,pose,&[]).unwrap();
    // A horizontal box/table top can itself be classified as a plane. It must
    // survive even when no persistent geometry commit has happened yet.
    let blocked=floor.with_recent_obstacle_points(2,200,0,&[0],[(0,[0.05,0.05,0.55])]).unwrap();
    let mut witness=None;
    assert_eq!(blocked.validate_body_motion_with_witness(pose,pose,&[],&mut witness),Err(SurfaceError::InsufficientClearance));
    assert!(matches!(witness.unwrap().rejection,BodyRejection::Occupancy{source:OccupancySource::Recent,..}));
    let cleared=blocked.with_recent_obstacle_points(3,300,0,&[],[]).unwrap();
    cleared.validate_body_motion(pose,pose,&[]).unwrap();
    let transferred=map.with_recent_obstacles_from(&blocked);
    assert!(transferred.validate_body_pose(pose.0,pose.1).is_err());
    transferred.clear_recent_obstacles().validate_body_pose(pose.0,pose.1).unwrap();
}

#[test]
fn recent_obstacles_cover_intermediate_translation_rotation_and_mixed_voxels() {
    let map=scene(&[],false);let q=[0.,0.,0.,1.];
    let from=([-1.,0.05,0.5],q);let to=([1.,0.05,0.5],q);
    let obstacle=map.with_recent_obstacle_points(1,100,0,&[0],[(0,[0.,0.05,0.55])]).unwrap();
    obstacle.validate_body_pose(from.0,from.1).unwrap();obstacle.validate_body_pose(to.0,to.1).unwrap();
    assert!(obstacle.validate_body_motion(from,to,&[]).is_err());
    let half=std::f64::consts::FRAC_PI_4;
    let turn=([0.05,0.05,0.5],[0.,0.,half.sin(),half.cos()]);
    let rotation=map.with_recent_obstacle_points(1,100,0,&[0],[(0,[0.05,0.45,0.55])]).unwrap();
    rotation.validate_body_pose([0.05,0.05,0.5],q).unwrap();
    assert!(rotation.validate_body_motion(([0.05,0.05,0.5],q),turn,&[]).is_err());
    let mixed=map.with_recent_obstacle_points(1,100,0,&[0],[(0,[0.01,0.01,0.55]),(1,[0.02,0.02,0.56])]).unwrap();
    assert!(mixed.recent_obstacles.as_ref().unwrap().frames[0].surface_points.is_empty());
    assert!(mixed.validate_body_pose([0.05,0.05,0.5],q).is_err());
}

#[test]
fn recent_retention_is_bounded_source_time_and_never_deletes_persistent_wall() {
    let pose=([0.05,0.05,0.5],[0.,0.,0.,1.]);
    let mut map=scene(&[],false);
    for seq in 1..=12 {map=map.with_recent_obstacle_points(seq,seq*100,10_000,&[],[(0,[3.,3.,0.5])]).unwrap();}
    assert_eq!(map.recent_obstacles.as_ref().unwrap().frames.len(),8);
    assert!(map.with_recent_obstacle_points(12,1300,0,&[],[]).is_err());
    let fresh=map.with_recent_obstacle_points(13,20_000,500,&[],[]).unwrap();
    assert_eq!(fresh.recent_obstacles.as_ref().unwrap().frames.len(),1);
    let wall=scene(&[(0.05,0.05,0.55)],false).with_recent_obstacles_from(&fresh).clear_recent_obstacles();
    assert!(wall.validate_body_pose(pose.0,pose.1).is_err());
}

#[test]
fn half_meter_lidar_sphere_blind_start_has_bounded_ground_projection() {
    // Lidar is 0.4m above the floor. A 0.5m spherical blind radius removes
    // floor returns in a 0.3m XY disk, not a fabricated 0.5m horizontal hole.
    let points:Vec<_>=(-30..30).flat_map(|x|(-30..30).filter_map(move|y| {
        let px=(x as f32+0.5)*0.05;let py=(y as f32+0.5)*0.05;
        ((px*px+py*py+0.4*0.4).sqrt()>=0.5).then_some((px,py,0.025))
    })).collect();
    let mut cfg=scene(&[],false).config().clone();cfg.voxel_size=0.05;cfg.wall_clearance_m=0.;
    let map=SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(model()).unwrap();
    let query=map.query_ground([0.025,0.025],0.05);
    assert_eq!(query.state,GroundQueryState::Estimated,"{query:?}");
    assert!(!query.sources.is_empty());assert!((query.height.unwrap()-0.05).abs()<1e-6);
    let from=([0.025,0.025,0.45],[0.,0.,0.,1.]);
    let next=map.body_pose_after_motion(from,[0.045,0.025],0.).unwrap();
    assert!((next.0[2]-from.0[2]).abs()<1e-6);
    map.validate_body_motion(from,next,&[]).unwrap();
    let unknown=map.query_ground([5.,5.],0.05);
    assert_eq!(unknown.state,GroundQueryState::Unknown);
    assert_eq!(map.body_pose_after_motion(([5.,5.,0.45],from.1),[5.01,5.],0.).unwrap(),([5.01,5.,0.45],from.1));
    assert_eq!(map.query_ground([5.01,5.],0.05).state,GroundQueryState::Unknown);
}

#[test]
fn live_floor_keeps_existing_foot_contact_but_independent_upper_surface_collides() {
    let mut envelope=model();envelope.z_bounds_m=[-0.448,0.057];
    let map=scene(&[],false).with_body_envelope(envelope).unwrap();
    let pose=([0.05,0.05,0.5],[0.,0.,0.,1.]);
    map.validate_body_pose(pose.0,pose.1).unwrap();
    let floor=map.with_recent_obstacle_points(1,100,0,&[0],[(0,[0.05,0.05,0.075])]).unwrap();
    floor.validate_body_motion(pose,pose,&[]).unwrap();
    let upper=floor.with_recent_obstacle_points(2,200,0,&[0],[(0,[0.05,0.05,0.45])]).unwrap();
    assert!(upper.validate_body_motion(pose,pose,&[]).is_err());
}

#[test]
fn new_recent_source_updates_guard_view_without_changing_terrain() {
    let map=scene(&[],false).with_recent_obstacle_points(1,100,0,&[0],[(0,[0.05,0.05,0.075])]).unwrap();
    let next=map.with_recent_obstacle_points(2,200,0,&[0],[(0,[0.05,0.05,0.075])]).unwrap();
    assert!(map.same_geometry(&next));assert!(!map.same_body_query_view(&next));
    assert!(!Arc::ptr_eq(&map.body_query_cache,&next.body_query_cache),"latest raw surface returns must be checked without an O(cloud) equality pass");
    assert_eq!(next.recent_obstacles.as_ref().unwrap().frames[0].timestamp,200);
    let changed=next.with_recent_obstacle_points(3,300,0,&[0],[(0,[0.05,0.05,0.45])]).unwrap();
    assert!(!next.same_body_query_view(&changed));
}

#[test]
fn local_recent_guide_hints_preserve_surface_and_volume_without_exporting_floor() {
    let mut envelope=model();envelope.z_bounds_m=[-0.448,0.057];
    let map=scene(&[],false).with_body_envelope(envelope).unwrap()
        .with_recent_obstacle_points(1,100,0,&[0,1],[(0,[0.05,0.05,0.075]),
            (1,[0.15,0.05,0.45]),(2,[0.25,0.05,0.45]),(3,[5.,5.,0.45])]).unwrap();
    let hints=map.recent_obstacle_hints([[-0.5,-0.5],[0.5,0.5]],([0.05,0.05,0.5],[0.,0.,0.,1.])).unwrap();
    assert_eq!(hints.len(),2);
    assert!(hints.iter().any(|&(a,b)|a==[0.15,0.05,0.45]&&a==b));
    assert!(hints.iter().any(|&(a,b)|a[0]<b[0]&&a[2]<b[2]));
    assert!(map.recent_obstacle_hints([[f64::NEG_INFINITY,0.],[1.,1.]],([0.;3],[0.,0.,0.,1.])).is_err());
}

#[test]
fn recent_guide_keeps_obstacles_above_and_below_the_starting_body() {
    let map=scene(&[],false).with_recent_obstacle_points(1,100,0,&[],
        [(0,[0.25,0.05,0.15]),(1,[0.25,0.15,1.05])]).unwrap();
    let start=([0.05,0.05,0.5],[0.,0.,0.,1.]);
    let hints=map.recent_obstacle_hints([[-0.5,-0.5],[0.5,0.5]],start).unwrap();
    assert_eq!(hints.len(),2,"参考沿地形升降，不能用起点机身高度截去后续障碍");
    for (_,point) in [(0,[0.25,0.05,0.15]),(1,[0.25,0.15,1.05])] {
        assert!(hints.iter().any(|(a,b)|(0..3).all(|i|a[i]<=point[i]&&point[i]<=b[i])));
    }
}

#[test]
fn recent_hint_range_uses_nonempty_columns_at_fine_source_resolution() {
    for quantum in [0.05_f32,0.01] {
        let source=scene(&[],false);let cfg=source.config().clone();
        let map=SurfaceSnapshot::from_points(&[(0.025,0.025,0.005)],cfg,quantum).unwrap()
            .with_body_envelope(model()).unwrap()
            .with_recent_obstacle_points(1,100,0,&[],[(0,[1.0,0.0,0.5]),(1,[20.,0.,0.5])]).unwrap();
        let hints=map.recent_obstacle_hints([[-6.5,-6.5],[6.5,6.5]],([0.,0.,0.5],[0.,0.,0.,1.])).unwrap();
        assert_eq!(hints.len(),1,"fine={quantum} must not silently lose obstacles due to empty-grid area");
        assert!(hints[0].0[0]<2.);
    }
}

#[test]
fn known_current_height_mismatch_is_distinct_from_blind_ground_and_normal_height_noise() {
    let map=scene(&[],false);let q=[0.,0.,0.,1.];
    for z in [0.45,0.5,0.55] {
        assert_eq!(map.body_ground_relation([0.05,0.05,z],q).unwrap(),BodyGroundRelation::KnownCompatible);
    }
    assert_eq!(map.body_ground_relation([0.05,0.05,2.0],q).unwrap(),BodyGroundRelation::KnownIncompatible);
    assert_eq!(map.body_ground_relation([100.,100.,0.5],q).unwrap(),BodyGroundRelation::Unknown);
    assert_eq!(map.body_ground_relation([0.05,0.05,0.5],q).unwrap(),BodyGroundRelation::KnownCompatible);
}
