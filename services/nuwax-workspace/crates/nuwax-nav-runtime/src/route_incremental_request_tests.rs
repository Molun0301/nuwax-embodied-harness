// 经过真实 publication 接收与请求构造，确认 Arc 列列表完整转交。
#[test]
fn runtime_incremental_route_request_retains_coalesced_basis_and_deleted_columns() {
    let mut config=SurfaceFollowerConfig::new(0.4);
    config.local_planning_enabled=false;
    let mut follower=RouteFollower::new(config).unwrap();
    let first=selective_scene(&[[100,100,5]]);
    let second=selective_scene(&[[110,100,5]]);
    let third=Arc::new(second.with_recent_obstacle_points(3,3_000_000_000,0,&[],[]).unwrap());
    follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&first))).unwrap();
    let mut publication=selective_update(&first,Arc::clone(&second),&[[100,100],[110,100]]);
    publication.append(SurfaceMapUpdate {snapshot:Arc::clone(&third),planar_reference:None,
        change_basis:Some(Arc::clone(&second)),changes:Arc::new(SurfaceGeometryChanges::Empty)});
    let changes=Arc::clone(&publication.changes);
    follower.update_execution_map(ExecutionMap::SurfaceUpdate(publication)).unwrap();
    let pose=comparison_pose(0.5,1);
    let route=Arc::new(Path3D {poses:vec![pose.clone()]});
    let start_ground=third.body_query().start_ground_state(body(&pose)).map_err(|error|error.to_string());
    let request=follower.planning_request(&third,&pose,&route,false,start_ground);
    assert!(Arc::ptr_eq(request.change_basis.as_ref().unwrap(),&first));
    assert!(Arc::ptr_eq(&request.changes,&changes),"请求共享完整变更列表，不复制 fine 列");
    assert!(Arc::ptr_eq(&request.source,&third));
    let nuwax_nav_pct::SurfaceDelta::Columns {columns,..}=
        crate::surface_changes::pct_surface_delta(request.change_basis.as_deref(),&request.changes)
        else {panic!("完整合并变化应保留 Columns");};
    assert_eq!(columns,&[[100,100],[110,100]],"已删除旧列仍必须交给增量核");
    follower.invalidate_execution_map();
    assert!(follower.change_basis.is_none());
    assert!(matches!(follower.changes.as_ref(),SurfaceGeometryChanges::Unknown));
}
