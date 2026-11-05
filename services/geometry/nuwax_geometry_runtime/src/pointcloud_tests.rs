use super::*;
#[test]
fn dense_delta_returns_only_the_unread_surviving_tail() {
    let mut map = CloudMap::new(41, None, None).unwrap();
    let old: Vec<_> = (0..8)
        .map(|i| Point3::new(i as f64 * 0.1, 0., 0.5))
        .collect();
    map.dense.append(&old).unwrap();
    map.dense_snapshot = map.dense.snapshot(100, 1, "41");
    let first = map
        .dense_chunk(
            proto::DenseCloudChunkRequest {
                expected_epoch: String::new(),
                offset_points: 0,
                max_points: 100,
                allow_delta: false,
            },
            "map",
        )
        .unwrap();
    assert_eq!(first.total_points, 8);
    let keys = [
        voxel_key([old[1].x, old[1].y, old[1].z], VOXEL_SIZE).unwrap(),
        voxel_key([old[5].x, old[5].y, old[5].z], VOXEL_SIZE).unwrap(),
    ];
    map.dense.remove_voxels(&keys, VOXEL_SIZE).unwrap();
    let added = Point3::new(3., 4., 0.5);
    map.dense.append(&[added]).unwrap();
    map.dense_snapshot = map.dense.snapshot(200, 2, "41");
    let delta = map
        .dense_chunk(
            proto::DenseCloudChunkRequest {
                expected_epoch: first.epoch.clone(),
                offset_points: 8,
                max_points: 100,
                allow_delta: true,
            },
            "map",
        )
        .unwrap();
    assert_eq!(delta.base_epoch, first.epoch);
    assert_eq!(delta.base_offset_points, 8);
    assert_eq!(delta.offset_points, 6);
    assert_eq!(delta.total_points, 7);
    assert_eq!(delta.removed_ranges, vec![1, 1, 5, 1]);
    assert_eq!(
        delta.xyz_f32_le,
        [added.x, added.y, added.z]
            .into_iter()
            .flat_map(|v| (v as f32).to_le_bytes())
            .collect::<Vec<_>>()
    );
}
#[test]
fn cloud_shm_cleanup_only_unlinks_its_owned_inode() {
    let path =
        std::env::temp_dir().join(format!("geometry-cloud-cleanup-{}", uuid::Uuid::new_v4()));
    let writer = CloudMapShmWriter::new(&path).unwrap();
    assert!(path.exists());
    drop(writer);
    assert!(!path.exists());
    let writer = CloudMapShmWriter::new(&path).unwrap();
    let replacement = path.with_extension("replacement");
    std::fs::write(&replacement, b"replacement").unwrap();
    std::fs::rename(&replacement, &path).unwrap();
    drop(writer);
    assert_eq!(std::fs::read(&path).unwrap(), b"replacement");
    std::fs::remove_file(path).unwrap();
}
