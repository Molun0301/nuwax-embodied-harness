use super::*;

#[test]
fn sparse_clear_regions_probe_only_local_grid_keys_on_large_maps() {
    let frame = test_free_cells(
        0.15,
        [([0, 0, 0], 0), ([6000, 0, 0], 0)],
        Isometry3::identity(),
    );
    let expected: AHashSet<_> = [(1, 1, 1), (18001, 1, 1)].into_iter().collect();
    let mut small_queries = 0;
    let small = frame
        .candidate_grid_keys(0.05, |key| {
            small_queries += 1;
            expected.contains(&key)
        })
        .unwrap();
    let mut large = expected.clone();
    large.extend((0..100_000).map(|i| (1_000_000 + i, 17, 19)));
    let mut large_queries = 0;
    let actual = frame
        .candidate_grid_keys(0.05, |key| {
            large_queries += 1;
            large.contains(&key)
        })
        .unwrap();
    assert_eq!(small, expected);
    assert_eq!(actual, expected);
    assert_eq!(
        small_queries, large_queries,
        "远处历史体素不能增加候选格查询次数"
    );
    assert!(small_queries < 400, "两端空单元之间的900米不应被枚举");
    let empty = test_free_cells(0.15, [], Isometry3::identity());
    assert!(empty
        .candidate_grid_keys(0.05, |_| panic!("无新增空单元不应查询地图"))
        .unwrap()
        .is_empty());
}

#[test]
fn transformed_regions_cover_the_f32_rounding_preimage_at_negative_coordinates() {
    let transform = Isometry3::from_parts(
        nalgebra::Translation3::new(-1.7, 21.3, 0.4),
        nalgebra::UnitQuaternion::from_euler_angles(0.04, -0.07, 0.7),
    );
    let frame = test_free_cells(0.15, [([-500_000, 100_000, -20], 0)], transform);
    // 这个双精度证据坐标略低于单元面，但f32舍入后实际落在本次新增空单元内。
    let point = transform * Point3::new(-75_000.001, 15_000.075, -2.925);
    let xyz = [point.x, point.y, point.z];
    assert!(frame.clears(xyz));
    let key = crate::dynamic_scene::cell(xyz).unwrap();
    let candidates = frame
        .candidate_grid_keys(crate::dynamic_scene::CELL_SIZE, |candidate| {
            candidate == key
        })
        .unwrap();
    assert!(
        candidates.contains(&key),
        "空间定位必须覆盖精确clears接受的舍入原像"
    );
    assert!(frame
        .clear_regions()
        .unwrap()
        .any(|(low, high)| (0..3).all(|axis| xyz[axis] >= low[axis] && xyz[axis] <= high[axis])));
}

#[test]
fn corrected_frame_and_sidecar_keep_exact_static_points_and_free_cell_centers() {
    use crate::dynamic_scene::tests::{corridor, ORIGIN};
    let transform = Isometry3::from_parts(
        nalgebra::Translation3::new(-12.31, 2.13, 0.17),
        nalgebra::UnitQuaternion::from_euler_angles(0.07, -0.03, 0.41),
    );
    let mut map = OnlineDufo::default();
    let mut checked = 0;
    for index in 1..=12 {
        let (points, _) = corridor((index < 5).then_some(0.0), (index % 3) as f64);
        let (mut frame, indices) = map.observe(&points, ORIGIN, index, transform).unwrap();
        let corrected: Vec<_> = points.iter().map(|p| transform * p).collect();
        frame.bind_points(&corrected, &indices);
        let decoded = MappedFrame::decode(&frame.encode().unwrap()).unwrap();
        let keep: AHashSet<_> = indices.iter().copied().collect();
        for (i, p) in corrected.iter().enumerate() {
            assert_eq!(
                decoded.is_dynamic([p.x, p.y, p.z]),
                !keep.contains(&(i as u32))
            );
        }
        let MappedFrame::Dufo(dufo) = &frame;
        let mut cells = AHashMap::new();
        for &(key, depth) in dufo.free.keys() {
            let center = transform
                * Point3::from(
                    key.map(|v| (v as f64 + (1u32 << depth) as f64 * 0.5) * dufo.resolution),
                );
            let p = [center.x, center.y, center.z];
            assert!(
                frame.clears(p) && decoded.clears(p),
                "校正及跨进程编码必须保留历史空单元身份"
            );
            let cell = crate::dynamic_scene::cell(p).unwrap();
            cells.insert(cell, 1);
            checked += 1;
        }
        // 按生产入口每帧查询一次，仍逐一检查全部原空单元，避免把空间查询重复为平方级。
        let candidates = decoded.potential_window_clear_cells(&cells).unwrap();
        for cell in cells.keys() {
            assert!(candidates.contains(cell));
        }
    }
    assert!(checked > 0, "用真实新增空体素检验校正坐标，不能空集合通过");
}
