// Tests-only: append as a nested module inside surface_dataflow::tests.
// These are pure shared-predicate / original-codec tests, NOT a real failure or
// owner scheduling qualification. Real F1/Nav entry tests cover those separately.
use super::*;
use crate::surface_map::SurfaceMapRetryDependency;
use nuwax_map_chunks::{Manifest, Repository, Transfer, Writer, MAX_BLOCK_BYTES};

struct DependencyFixture {
    failed: schema::NavigationMap3D,
    root: Manifest,
    source: Arc<SurfaceSnapshot>,
}
impl DependencyFixture {
    fn new(masked: bool) -> Self {
        let mut first = map_message(1, "dependency-guards");
        first.cloud.width = 0;
        first.cloud.row_step = 0;
        first.cloud.data.clear();
        if masked { first.virtual_obstacles = Some(virtual_mask(&first, false)); }
        let repository = Repository::temporary().unwrap();
        let mut writer = Writer::new(repository.clone(), first.source_voxel_size, 4 * 1024 * 1024).unwrap();
        let cells: Vec<_> = (0..40).flat_map(|x| (0..30).map(move |y| [x, y, 20])).collect();
        writer.observe(cells.iter().copied(), first.source_timestamp_ns as u64).unwrap();
        let first_root = writer.snapshot(schema::navigation_map_3d_domain(&first), 1,
            first.source_timestamp_ns as u64).unwrap();
        let included = first_root.blocks.iter().map(|block|
            (block.object, repository.read_object(block.object, MAX_BLOCK_BYTES).unwrap())).collect();
        first.fine = Some(Transfer { root: first_root.encode().unwrap(), included });
        schema::validate_navigation_map_3d(&first).unwrap();
        let mut cache = SurfaceGeometryCache::configured(&config());
        let source = cache.build(&first).unwrap();
        let mut failed = first.clone();
        failed.cloud_revision = 2;
        failed.cloud.header.seq = 2;
        // A new authentic observation changes immutable IDs while geometry is
        // retained. All fixture mutation happens outside any timing assertion.
        let stamp = first.source_timestamp_ns + 1;
        writer.observe(cells, stamp as u64).unwrap();
        let root = writer.snapshot(schema::navigation_map_3d_domain(&failed), 2, stamp as u64).unwrap();
        failed.source_timestamp_ns = stamp;
        failed.committed_map.source_timestamp_ns = stamp;
        failed.cloud.header.stamp_sec = (stamp / 1_000_000_000) as i32;
        failed.cloud.header.stamp_nsec = (stamp % 1_000_000_000) as i32;
        if let Some(mask) = failed.virtual_obstacles.as_mut() {
            mask.header.stamp_sec = failed.cloud.header.stamp_sec;
            mask.header.stamp_nsec = failed.cloud.header.stamp_nsec;
        }
        failed.fine = Some(Transfer { root: root.encode().unwrap(), included: Default::default() });
        schema::validate_navigation_map_3d(&failed).unwrap();
        assert!(root.blocks.len() >= 2);
        assert!(!first_root.blocks.iter().any(|old| old.object == root.blocks[0].object));
        Self { failed, root: (*root).clone(), source }
    }
    fn dependency(&self) -> SurfaceMapRetryDependency {
        let missing = &self.root.blocks[0];
        SurfaceMapRetryDependency::new(missing.object, missing.key, &self.root,
            &self.failed, Some(Arc::clone(&self.source)))
    }
    fn candidate(&self, root: &Manifest) -> schema::NavigationMap3D {
        let mut message = self.failed.clone();
        message.cloud_revision = root.revision as i64;
        message.cloud.header.seq = root.revision as i32;
        message.fine = Some(Transfer { root: root.encode().unwrap(), included: Default::default() });
        schema::validate_navigation_map_3d(&message).unwrap();
        message
    }
}

#[test]
fn dependency_same_key_and_same_object_remains_blocked() {
    let fixture = DependencyFixture::new(false);
    let dependency = fixture.dependency();
    let mut root = fixture.root.clone();
    root.revision += 1;
    let message = fixture.candidate(&root);
    assert!(dependency.still_blocked(&message, Some(&root)));
}

#[test]
fn dependency_old_key_absent_still_finds_same_object_elsewhere() {
    let fixture = DependencyFixture::new(false);
    let dependency = fixture.dependency();
    let mut root = fixture.root.clone();
    root.revision += 1;
    root.geometry_revision = root.revision;
    let mut moved = root.blocks[0].clone();
    moved.key = [10, 0, 0];
    moved.block_revision = root.revision;
    moved.geometry_revision = root.revision;
    root.blocks = vec![moved];
    let message = fixture.candidate(&root);
    assert!(root.blocks.binary_search_by_key(&fixture.root.blocks[0].key, |b| b.key).is_err());
    assert!(dependency.still_blocked(&message, Some(&root)), "old-key miss cannot prove object absence");
}

#[test]
fn dependency_replaced_old_key_still_finds_duplicate_object_reference() {
    let fixture = DependencyFixture::new(false);
    let dependency = fixture.dependency();
    let mut root = fixture.root.clone();
    root.revision += 1;
    root.geometry_revision = root.revision;
    let mut replacement = fixture.root.blocks[1].clone();
    replacement.key = fixture.root.blocks[0].key;
    let mut duplicate = fixture.root.blocks[0].clone();
    duplicate.key = [10, 0, 0];
    for block in [&mut replacement, &mut duplicate] {
        block.block_revision = root.revision; block.geometry_revision = root.revision;
    }
    root.blocks = vec![replacement, duplicate];
    let message = fixture.candidate(&root);
    assert_ne!(root.blocks[0].object, fixture.root.blocks[0].object);
    assert!(dependency.still_blocked(&message, Some(&root)), "replacement needs the metadata fallback");
    // Same object at multiple keys is structurally legal, with equal point count.
    let mut two = root.blocks[1].clone();
    two.key = [11, 0, 0];
    root.blocks.push(two);
    let message = fixture.candidate(&root);
    assert!(dependency.still_blocked(&message, Some(&root)));
}

#[test]
fn dependency_removed_object_or_supplied_bytes_allows_original_validation() {
    let fixture = DependencyFixture::new(false);
    let dependency = fixture.dependency();
    let missing = fixture.root.blocks[0].object;
    let mut root = fixture.root.clone();
    root.revision += 1;
    root.geometry_revision = root.revision;
    root.blocks.retain(|block| block.object != missing);
    let message = fixture.candidate(&root);
    assert!(!dependency.still_blocked(&message, Some(&root)));
    let mut supplied = fixture.failed.clone();
    supplied.fine.as_mut().unwrap().included.insert(missing, vec![0]);
    assert!(!dependency.still_blocked(&supplied, Some(&fixture.root)));
    assert!(schema::validate_navigation_map_3d(&supplied).is_err(),
        "presence permits an attempt, never accepts malformed bytes");
    let mut receiver = nuwax_map_chunks::Receiver::new(Repository::temporary().unwrap());
    assert!(receiver.receive(supplied.fine.as_ref().unwrap(), &fixture.root.domain,
        |_, _| panic!("malformed included object must fail before fetch")).is_err());
    assert!(receiver.current().is_none());
}

#[test]
fn dependency_present_mask_requires_actual_layout_and_cells() {
    let fixture = DependencyFixture::new(true);
    let dependency = fixture.dependency();
    assert!(dependency.still_blocked(&fixture.failed, Some(&fixture.root)));
    let mut clock_only = fixture.failed.clone();
    clock_only.committed_map.revision += 1;
    clock_only.committed_map.commit_sequence += 1;
    clock_only.virtual_obstacles.as_mut().unwrap().header.seq += 1;
    clock_only.virtual_obstacles.as_mut().unwrap().info.map_load_time_sec += 1;
    schema::validate_navigation_map_3d(&clock_only).unwrap();
    assert!(dependency.still_blocked(&clock_only, Some(&fixture.root)));
    let mut changed_cell = fixture.failed.clone();
    changed_cell.virtual_obstacles.as_mut().unwrap().data[0] = 100;
    schema::validate_navigation_map_3d(&changed_cell).unwrap();
    assert!(!dependency.still_blocked(&changed_cell, Some(&fixture.root)));
    let mut changed_layout = fixture.failed.clone();
    changed_layout.virtual_obstacles.as_mut().unwrap().info.origin.position[0] += 0.1;
    schema::validate_navigation_map_3d(&changed_layout).unwrap();
    assert!(!dependency.still_blocked(&changed_layout, Some(&fixture.root)));
    let mut removed_mask = fixture.failed.clone();
    removed_mask.virtual_obstacles = None;
    assert!(!dependency.still_blocked(&removed_mask, Some(&fixture.root)));
}

#[test]
fn dependency_codec_old_and_root_returning_apis_preserve_admission() {
    let fixture = DependencyFixture::new(false);
    let encoded = schema::encode_navigation_map_3d_body(&fixture.failed).unwrap();
    let old = schema::decode_navigation_map_3d_body(&encoded).unwrap();
    let (message, root) = schema::decode_navigation_map_3d_body_with_root(&encoded).unwrap();
    assert_eq!(old, message);
    assert_eq!(root.as_ref(), Some(&fixture.root));
    // Keep the admitted root/envelope exact; replace only the trailing generic
    // cloud body with a shape rejected by the final navigation-cloud check.
    let cloud = schema::encode_point_cloud2_body(&fixture.failed.cloud).unwrap();
    assert!(encoded.ends_with(&cloud));
    let mut bad_cloud = fixture.failed.cloud.clone();
    bad_cloud.point_step = -1;
    let mut malformed = encoded[..encoded.len() - cloud.len()].to_vec();
    malformed.extend(schema::encode_point_cloud2_body(&bad_cloud).unwrap());
    for error in [schema::decode_navigation_map_3d_body(&malformed).unwrap_err(),
        schema::decode_navigation_map_3d_body_with_root(&malformed).unwrap_err()] {
        assert!(error.to_string().contains("explicit empty cloud"), "wrong failure boundary: {error}");
    }
    let mut bad = fixture.failed.clone();
    bad.fine.as_mut().unwrap().root = b"{}".to_vec();
    assert!(schema::validate_navigation_map_3d(&bad).is_err());
}
