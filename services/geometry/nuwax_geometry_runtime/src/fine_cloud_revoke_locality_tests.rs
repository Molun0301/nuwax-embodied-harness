use super::*;
use nuwax_map_chunks::{block_key, Key, RegionReader};

fn domain() -> nuwax_map_chunks::Domain {
    nuwax_map_chunks::Domain {
        session: "fine-revoke-locality".into(),
        generation: 1,
        frame_generation: 1,
        producer_generation: 1,
    }
}

#[test]
fn scattered_free_cells_revoke_over_two_million_voxels_by_two_blocks() {
    let mut fine = FineCloud::fresh().unwrap();
    let side = 128;
    let count = (side * side * side) as usize;
    assert!(count > 2_000_000);
    fine.writer
        .seed_unknown(
            (0..side).flat_map(|x| (0..side).flat_map(move |y| (0..side).map(move |z| [x, y, z]))),
        )
        .unwrap();
    let old = fine.seal(domain(), 1, 1).unwrap();
    assert_eq!(
        old.manifest()
            .blocks
            .iter()
            .map(|block| block.points as usize)
            .sum::<usize>(),
        count
    );
    let frame = crate::dynamic_backend::test_free_cells(
        0.15,
        [([0; 3], 0), ([42; 3], 0)],
        nalgebra::Isometry3::identity(),
    );
    let (low, high) = frame.clear_bounds().unwrap();
    let low = voxel_key(low, fine.voxel_size()).unwrap();
    let high = voxel_key(high, fine.voxel_size()).unwrap();
    assert_eq!(
        fine.writer
            .region([low.0, low.1, low.2], [high.0, high.1, high.2], 2_000_000),
        Err("region exceeds point budget".into()),
        "同一真实大图必须触发原整体查询故障"
    );

    // 有原点的零时间种子只查询当前分类；不能读取整图，也不能重记已有命中。
    let observed_key = [64; 3];
    let unknown_key = [65; 3];
    fine.writer.observe([observed_key], 1).unwrap();
    let seed_points = [observed_key, unknown_key].map(|key| {
        Point3::from(super::super::voxel_center_f64(
            (key[0], key[1], key[2]),
            fine.voxel_size(),
        ))
    });
    fine.accumulate_inner(&seed_points, Some([0.0; 3]), 0, None)
        .unwrap();
    assert_eq!(fine.last_revoke_read_blocks(), 0);
    assert_eq!(fine.last_revoke_block_lookups(), 0);
    assert!(fine.removed_keys().is_empty());
    assert!(fine.shared_frame().is_some_and(|frame| !frame.has_clears()));
    assert!(seed_points.iter().all(|point| !fine.is_dynamic(point)));
    assert_eq!(
        fine.writer.region(observed_key, observed_key, 1).unwrap()[0].1,
        nuwax_map_chunks::Evidence {
            observed_ns: 1,
            hits: 1
        }
    );
    assert_eq!(
        fine.writer.region(unknown_key, unknown_key, 1).unwrap()[0].1,
        nuwax_map_chunks::Evidence {
            observed_ns: 0,
            hits: 0
        }
    );

    fine.accumulate_inner(&[], Some([0.0; 3]), 2, Some(frame))
        .unwrap();
    assert_eq!(
        fine.last_revoke_read_blocks(),
        2,
        "中间62个精细块不属于新增free范围"
    );
    assert_eq!(fine.last_revoke_block_lookups(), 16);
    assert_eq!(fine.removed_keys().len(), 35);
    assert_eq!(fine.revoke_index_memory_bytes(), 0);
    let new = fine.seal(domain(), 2, 2).unwrap();
    assert_eq!(
        new.manifest()
            .blocks
            .iter()
            .map(|block| block.points as usize)
            .sum::<usize>(),
        count - 35
    );
    let mut reader = RegionReader::new(old.repository.clone(), 4 * MIB);
    assert_eq!(
        reader
            .region(old.manifest(), [0; 3], [2; 3], 27)
            .unwrap()
            .len(),
        27,
        "已发布旧读者仍能读取撤销前的全部点"
    );
    assert!(reader
        .region(new.manifest(), [0; 3], [2; 3], 27)
        .unwrap()
        .is_empty());
    println!("FINE_REVOKE_LOCALITY voxels={count} blocks=64 read_blocks=2 block_lookups=16 removed=35 persistent_index_bytes=0");
}

#[test]
fn rotated_negative_free_cells_match_full_scan_and_read_only_intersections() {
    let transform = nalgebra::Isometry3::from_parts(
        nalgebra::Translation3::new(-1.7, 2.4, -0.03),
        nalgebra::UnitQuaternion::from_euler_angles(0.11, -0.09, 0.37),
    );
    let frame = crate::dynamic_backend::test_free_cells(
        0.15,
        [([-2, 0, -1], 0), ([12, -8, 4], 1)],
        transform,
    );
    let mut fine = FineCloud::fresh().unwrap();
    let size = fine.voxel_size();
    let regions: Vec<_> = frame
        .clear_regions()
        .unwrap()
        .map(|(low, high)| {
            let low = voxel_key(low, size).unwrap();
            let high = voxel_key(high, size).unwrap();
            ([low.0, low.1, low.2], [high.0, high.1, high.2])
        })
        .collect();
    let mut keys = BTreeSet::new();
    for &(low, high) in &regions {
        for x in low[0] - 1..=high[0] + 1 {
            for y in low[1] - 1..=high[1] + 1 {
                for z in low[2] - 1..=high[2] + 1 {
                    keys.insert([x, y, z]);
                }
            }
        }
    }
    let first_x = keys.first().unwrap()[0];
    keys.extend((1000..4000).map(|y| [first_x, y, 0]));
    let expected_removed: BTreeSet<_> = keys
        .iter()
        .copied()
        .filter(|key| {
            frame.clears(super::super::voxel_center_f64(
                (key[0], key[1], key[2]),
                size,
            ))
        })
        .collect();
    assert!(!expected_removed.is_empty());
    let existing_blocks: BTreeSet<_> = keys.iter().copied().map(block_key).collect();
    // 独立遍历夹具的完整块集合，按块的闭区间相交关系确定应访问集合。
    let expected_blocks: BTreeSet<_> = existing_blocks
        .iter()
        .copied()
        .filter(|block| {
            regions.iter().any(|(low, high)| {
                (0..3).all(|axis| {
                    block[axis] * 32 <= high[axis] && block[axis] * 32 + 31 >= low[axis]
                })
            })
        })
        .collect();
    fine.writer.seed_unknown(keys.iter().copied()).unwrap();
    fine.writer.flush().unwrap();
    fine.accumulate_inner(&[], Some([0.0; 3]), 1, Some(frame))
        .unwrap();
    assert_eq!(fine.last_revoke_read_blocks(), expected_blocks.len());
    assert!(fine.last_revoke_read_blocks() < existing_blocks.len());
    assert_eq!(
        fine.removed_keys()
            .iter()
            .map(|key| [key.0, key.1, key.2])
            .collect::<BTreeSet<_>>(),
        expected_removed
    );
    let actual: BTreeSet<Key> = fine
        .writer
        .region([i32::MIN; 3], [i32::MAX; 3], keys.len())
        .unwrap()
        .into_iter()
        .map(|(key, _)| key)
        .collect();
    assert_eq!(
        actual,
        keys.difference(&expected_removed).copied().collect()
    );
    println!(
        "FINE_REVOKE_ROTATED blocks={} read_blocks={} block_lookups={} removed={}",
        existing_blocks.len(),
        fine.last_revoke_read_blocks(),
        fine.last_revoke_block_lookups(),
        expected_removed.len()
    );
    fine.accumulate_inner(
        &[],
        Some([0.0; 3]),
        2,
        Some(crate::dynamic_backend::MappedFrame::unfiltered()),
    )
    .unwrap();
    assert_eq!(fine.last_revoke_read_blocks(), 0);
    assert_eq!(fine.last_revoke_block_lookups(), 0);
}
