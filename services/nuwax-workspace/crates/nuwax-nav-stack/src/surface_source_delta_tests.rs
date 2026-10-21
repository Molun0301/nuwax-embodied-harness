// 同一生产地图拥有方保留完整细列变化与失败恢复证据。
use super::*;
use std::collections::BTreeSet;
const A:[i32;3]=[71,11,47];
const RELATED:[i32;3]=[71,25,47];
fn configured()->SurfaceNavigationConfig {
    let mut config=config();
    config.apply_calibration(Some(nuwax_nav_types_messages::RobotGeometry {self_collision:None,
        footprint_xy_m:vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
        z_min_m:-0.05,z_max_m:0.2,source:"原生来源变化回归".into()}),None).unwrap();config
}
struct FineFixture {
    repository: nuwax_map_chunks::Repository,
    writer: nuwax_map_chunks::Writer,
    cells: BTreeSet<[i32; 3]>,
    previous: BTreeSet<[i32; 3]>,
    revision: i64,
}
impl FineFixture {
    fn new() -> Self {
        let repository=nuwax_map_chunks::Repository::temporary().unwrap();
        Self { writer:nuwax_map_chunks::Writer::new(repository.clone(),0.05,4*1024*1024).unwrap(),
            repository, cells:(0..80).flat_map(|x|(0..60).map(move|y|[x,y,40])).collect(),
            previous:BTreeSet::new(),revision:0 }
    }
    fn next(&mut self) -> schema::NavigationMap3D {
        self.revision+=1;
        let mut message=map_message(self.revision,"selective-cache-fixture");
        self.writer.remove(self.previous.difference(&self.cells).copied()).unwrap();
        self.writer.observe(self.cells.iter().copied(),message.source_timestamp_ns as u64).unwrap();
        message.source_voxel_size=0.05;
        message.cloud.width=0;message.cloud.row_step=0;message.cloud.data.clear();
        let root=self.writer.snapshot(schema::navigation_map_3d_domain(&message),self.revision as u64,
            message.source_timestamp_ns as u64).unwrap();
        let included=root.blocks.iter().map(|block|(block.object,self.repository
            .read_object(block.object,nuwax_map_chunks::MAX_BLOCK_BYTES).unwrap())).collect();
        message.fine=Some(nuwax_map_chunks::Transfer {root:root.encode().unwrap(),included});
        self.previous=self.cells.clone();
        schema::validate_navigation_map_3d(&message).unwrap();
        message
    }
}

#[test]
fn selective_refresh_fine_delta_accumulates_all_blocks_and_last_layer_deletions() {
    let configured=configured();
    let mut cache=SurfaceGeometryCache::configured(&configured);
    let mut fixture=FineFixture::new();
    fixture.cells.extend([[-1,-1,70],[33,33,70],[33,33,100]]);
    let first=cache.build_with_changes(&fixture.next()).unwrap();
    assert!(matches!(first.changes.as_ref(),SurfaceGeometryChanges::Unknown));
    fixture.cells.remove(&[-1,-1,70]);
    fixture.cells.remove(&[33,33,70]);fixture.cells.remove(&[33,33,100]);
    fixture.cells.remove(&[40,30,40]);fixture.cells.insert([65,1,70]);
    let second=cache.build_with_changes(&fixture.next()).unwrap();
    assert!(nuwax_nav_runtime::native_map::same_source_geometry(second.change_basis.as_ref().unwrap(),&first.snapshot));
    let SurfaceGeometryChanges::Columns {fine,coarse,..}=second.changes.as_ref() else {panic!("complete compatible delta required")};
    assert_eq!(fine.iter().copied().collect::<BTreeSet<_>>(),BTreeSet::from([[-1,-1],[33,33],[40,30],[65,1]]));
    assert!(coarse.contains(&[-1,-1]),"removed final fine column must remove its coarse column");
    assert!(second.snapshot.observations().unwrap().get([-1,-1,70]).is_none());
    let metadata=cache.build_with_changes(&fixture.next()).unwrap();
    assert!(matches!(metadata.changes.as_ref(),SurfaceGeometryChanges::Empty));
    assert!(nuwax_nav_runtime::native_map::same_source_geometry(&second.snapshot,&metadata.snapshot));
    eprintln!("SELECTIVE_MULTIBLOCK fine_columns={fine:?} coarse_columns={coarse:?} deletion_and_metadata=true");
}

#[test]
fn selective_refresh_fine_only_delta_survives_unchanged_coarse_snapshot() {
    let configured=configured();
    let mut cache=SurfaceGeometryCache::configured(&configured);
    let mut fixture=FineFixture::new();
    let first=cache.build_with_changes(&fixture.next()).unwrap();
    fixture.cells.remove(&[40,30,40]);
    let second=cache.build_with_changes(&fixture.next()).unwrap();
    assert!(first.snapshot.same_geometry(&second.snapshot));
    assert!(!nuwax_nav_runtime::native_map::same_source_geometry(&first.snapshot,&second.snapshot));
    let SurfaceGeometryChanges::Columns {fine,coarse,..}=second.changes.as_ref() else {panic!("fine-only scope lost")};
    assert_eq!(fine,&vec![[40,30]]);assert!(coarse.is_empty());
}

#[test]
fn selective_refresh_failed_physical_advance_is_unknown_once_then_recovers_known() {
    let configured=configured();
    let mut cache=SurfaceGeometryCache::configured(&configured);
    let mut fixture=FineFixture::new();
    let first=cache.build_with_changes(&fixture.next()).unwrap();
    let physical_before=cache.physical_snapshot().unwrap();
    fixture.cells.insert(A);
    let mut broken=fixture.next();
    let mut mask=virtual_mask(&broken,false);
    mask.info.origin.position[0]=f64::MAX;
    broken.virtual_obstacles=Some(mask);
    // This is a legal wire message but the mask's finite extent is not
    // representable by the derived map. It fails after physical advances.
    schema::validate_navigation_map_3d(&broken).unwrap();
    assert!(cache.build_with_changes(&broken).is_err());
    assert!(!Arc::ptr_eq(&physical_before,&cache.physical_snapshot().unwrap()));
    let recovered=cache.build_with_changes(&fixture.next()).unwrap();
    assert!(nuwax_nav_runtime::native_map::same_source_geometry(recovered.change_basis.as_ref().unwrap(),&first.snapshot));
    assert!(matches!(recovered.changes.as_ref(),SurfaceGeometryChanges::Unknown));
    let metadata=cache.build_with_changes(&fixture.next()).unwrap();
    assert!(matches!(metadata.changes.as_ref(),SurfaceGeometryChanges::Empty));
    fixture.cells.insert(RELATED);
    let known=cache.build_with_changes(&fixture.next()).unwrap();
    let SurfaceGeometryChanges::Columns {fine,..}=known.changes.as_ref() else {panic!("successful recovery must restore known basis")};
    assert_eq!(fine,&vec![[RELATED[0],RELATED[1]]]);
    eprintln!("SELECTIVE_BASIS fail_after_physical=true recovered_unknown=true next_metadata_empty=true next_geometry_known=true");
}
