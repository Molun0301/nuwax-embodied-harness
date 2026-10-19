// 请求/缓存接线测试；A 的增量算法及局部窗口回退保持原实现。
fn incremental_complete_source(base:&SurfaceSnapshot,removed:&[[i32;2]],added:&[[i32;3]])
    ->Arc<SurfaceSnapshot> {
    let mut observations=SurfaceObservations::new(base.source_voxel_size_m(),32).unwrap();
    let mut blocks=std::collections::BTreeMap::<_,std::collections::BTreeMap<_,_>>::new();
    for key in base.cell_keys().map(|(x,y,z)|[x,y,z])
        .filter(|key|!removed.contains(&[key[0],key[1]])).chain(added.iter().copied()) {
        blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,
            SurfaceObservation {observed_ns:1_000_000_000,hits:2});
    }
    for (key,values) in blocks {observations.replace_block(key,values).unwrap();}
    observations.mark_complete_geometry_source();
    Arc::new(base.with_observations(Some(Arc::new(observations))))
}
fn incremental_column_changes(source:&SurfaceSnapshot,columns:Vec<[i32;2]>)->Arc<SurfaceGeometryChanges> {
    Arc::new(SurfaceGeometryChanges::Columns {fine_size:source.source_voxel_size_m(),fine:columns,
        planner_size:source.config().voxel_size,coarse:Vec::new()})
}
fn incremental_facts(map:&nuwax_nav_pct::Tomogram)->Vec<([i32;2],i32,[u64;6])> {
    let mut cells:Vec<_>=map.cells.iter().map(|cell|(cell.column,cell.layer,
        [cell.ground,cell.ground_interval[0],cell.ground_interval[1],cell.ceiling,
            cell.clearance,cell.cost].map(f64::to_bits))).collect();
    cells.sort();cells
}

#[test]
fn runtime_incremental_delta_borrows_complete_columns_including_deletion() {
    use nuwax_nav_pct::{SurfaceDelta,Tomogram};
    let (_,request,_)=fixture();
    let source=incremental_complete_source(&request.source,&[],&[]);
    let old=Tomogram::from_surface(&source).unwrap();
    let before=incremental_facts(&old);
    let deleted=old.cells[old.cells.len()/2].column;
    let added=[100,100];
    let next=incremental_complete_source(&source,&[deleted],&[[added[0],added[1],0]]);
    let changes=incremental_column_changes(&next,vec![deleted,added]);
    let SurfaceGeometryChanges::Columns {fine,..}=changes.as_ref() else {unreachable!()};
    let delta=pct_surface_delta(Some(&source),&changes);
    let SurfaceDelta::Columns {basis,resolution,columns}=delta else {panic!("完整列必须传给 A");};
    assert!(std::ptr::eq(basis,source.as_ref()));
    assert_eq!(resolution.to_bits(),f64::from(next.source_voxel_size_m()).to_bits());
    assert_eq!(columns.as_ptr(),fine.as_ptr(),"只借用原完整列列表，不能复制或丢失删除列");
    assert_eq!(columns,&[deleted,added]);
    let (updated,stats)=old.updated_from_surface(&next,delta).unwrap();
    assert!(!stats.rebuilt,"完整全局基准允许 A 的真正增量路径");
    assert_eq!(stats.columns_prepared,2);
    assert!(!updated.cells.iter().any(|cell|cell.column==deleted));
    assert!(updated.cells.iter().any(|cell|cell.column==added));
    assert_eq!(incremental_facts(&updated),incremental_facts(&Tomogram::from_surface(&next).unwrap()));
    assert_eq!(incremental_facts(&old),before,"旧不可变图不能被修改");
}

#[test]
fn runtime_incremental_empty_keeps_static_geometry_and_unknown_or_unlinked_basis_rebuilds() {
    use nuwax_nav_pct::{SurfaceDelta,Tomogram};
    let (_,request,_)=fixture();
    let source=incremental_complete_source(&request.source,&[],&[]);
    let old=Tomogram::from_surface(&source).unwrap();
    let recent=source.with_recent_obstacle_points(2,2_000_000_000,0,&[],[(0,[1.0,1.0,0.6])]).unwrap();
    let delta=pct_surface_delta(Some(&source),&SurfaceGeometryChanges::Empty);
    assert!(matches!(delta,SurfaceDelta::Unchanged {..}));
    let (unchanged,stats)=old.updated_from_surface(&recent,delta).unwrap();
    assert!(!stats.rebuilt);
    assert_eq!(stats.columns_prepared,0);
    assert_eq!(incremental_facts(&unchanged),incremental_facts(&old));
    let deleted=old.cells[old.cells.len()/2].column;
    let next=incremental_complete_source(&source,&[deleted],&[]);
    let columns=incremental_column_changes(&next,vec![deleted]);
    let unrelated=incremental_complete_source(&source,&[],&[[100,100,0]]);
    for (basis,changes) in [(None,columns.as_ref()),
        (Some(source.as_ref()),&SurfaceGeometryChanges::Unknown),
        (Some(unrelated.as_ref()),columns.as_ref())] {
        let (updated,stats)=old.updated_from_surface(&next,pct_surface_delta(basis,changes)).unwrap();
        assert!(stats.rebuilt,"缺基准、Unknown 或错误变化基准都必须冷建");
        assert_eq!(incremental_facts(&updated),incremental_facts(&Tomogram::from_surface(&next).unwrap()));
    }
}

#[test]
fn runtime_incremental_query_parameter_change_rebuilds_even_with_complete_columns() {
    let (_,request,_)=fixture();
    let source=incremental_complete_source(&request.source,&[],&[]);
    let old=nuwax_nav_pct::Tomogram::from_surface(&source).unwrap();
    let mut model=source.body_envelope().unwrap().clone();
    model.max_step_up_m*=0.5;
    let changed=source.with_body_envelope(model).unwrap();
    let columns=incremental_column_changes(&changed,Vec::new());
    let (updated,stats)=old.updated_from_surface(&changed,
        pct_surface_delta(Some(&source),&columns)).unwrap();
    assert!(stats.rebuilt,"参数变化不能因列列表完整而继承旧查询语义");
    assert_eq!(incremental_facts(&updated),
        incremental_facts(&nuwax_nav_pct::Tomogram::from_surface(&changed).unwrap()));
}

#[test]
fn runtime_incremental_local_owner_preserves_window_and_old_snapshot_on_source_update() {
    let (mut kernel,mut request,_)=fixture();
    request.scan.horizon_m=0.5;
    request.source=incremental_complete_source(&request.source,&[],&[[100,100,0]]);
    let old=kernel.prepare_local_map(&request,&request.source).unwrap();
    let (_,center,_,radius)=kernel.local_map.as_ref().unwrap().clone();
    assert!(!old.cells.iter().any(|cell|cell.column==[100,100]),"原局部窗口不含远处列");
    let before=incremental_facts(&old);
    let deleted=old.cells[old.cells.len()/2].column;
    let basis=Arc::clone(&request.source);
    request.source=incremental_complete_source(&basis,&[deleted],&[[100,100,0]]);
    request.change_basis=Some(basis);
    request.changes=incremental_column_changes(&request.source,vec![deleted]);
    request.pose.pose.position.x+=0.1;
    let updated=kernel.prepare_local_map(&request,&request.source).unwrap();
    assert!(!Arc::ptr_eq(&old,&updated));
    assert_eq!(kernel.local_map.as_ref().unwrap().1,center,
        "来源变化通过 A 的原窗口回退，不能把局部图无意变成全局图或移动窗口");
    assert_eq!(kernel.local_map.as_ref().unwrap().3.to_bits(),radius.to_bits());
    assert!(!updated.cells.iter().any(|cell|cell.column==deleted||cell.column==[100,100]));
    let cold=nuwax_nav_pct::Tomogram::local_from_surface(&request.source,[center.x,center.y],radius).unwrap();
    assert_eq!(incremental_facts(&updated),incremental_facts(&cold));
    assert_eq!(incremental_facts(&old),before);
    let reused=kernel.prepare_local_map(&request,&request.source).unwrap();
    assert!(Arc::ptr_eq(&updated,&reused),"同一已更新来源的重复请求继续复用原 Arc");
}

#[test]
fn runtime_incremental_local_window_parameter_change_rebuilds_and_cancel_keeps_cache() {
    let (mut kernel,mut request,_)=fixture();
    request.scan.horizon_m=0.5;
    let old=kernel.prepare_local_map(&request,&request.source).unwrap();
    let old_radius=kernel.local_map.as_ref().unwrap().3;
    request.pose.pose.position.x+=0.1;
    request.scan.horizon_m=1.5;
    let updated=kernel.prepare_local_map(&request,&request.source).unwrap();
    assert!(!Arc::ptr_eq(&old,&updated),"horizon 改变不能复用旧窗口");
    assert_eq!(kernel.local_map.as_ref().unwrap().1,request.pose.pose.position);
    assert!(kernel.local_map.as_ref().unwrap().3>old_radius);
    let retained_source=Arc::clone(&request.source);
    request.change_basis=Some(Arc::clone(&request.source));
    request.source=incremental_complete_source(&request.source,&[],&[[100,100,0]]);
    request.changes=incremental_column_changes(&request.source,vec![[100,100]]);
    let flag=Arc::new(std::sync::atomic::AtomicBool::new(true));
    let token=nuwax_nav_pct::PlanningCancellation::from_flag(flag);
    let result=nuwax_nav_pct::with_planning_cancellation(&token,||kernel.prepare_local_map(&request,&request.source));
    assert!(result.is_err());
    assert!(Arc::ptr_eq(&updated,&kernel.local_map.as_ref().unwrap().2),"取消不能替换原完整缓存");
    assert!(Arc::ptr_eq(&retained_source,&kernel.local_map.as_ref().unwrap().0));
}
