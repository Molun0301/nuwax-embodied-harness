use super::*;

#[test]
fn slanted_free_observations_remove_the_same_residue_as_three_dimensional_rays() {
    use nalgebra::point;
    use nuwax_mapping::{
        cost_mapper::SimplePointCloud2, pointclouds::occupancy::build_occupancy_auto,
    };
    let observe = |timestamp, points| {
        build_occupancy_auto(
            &SimplePointCloud2::with_metadata(points, point![0.025, 0.025, 0.0], "map", timestamp),
            0.05,
        )
        .unwrap()
    };
    let first = observe(1.0, vec![point![1.025, 0.025, 0.0]]);
    let mut persistent = PersistentOccupancyGrid::from_observation(&first).unwrap();
    persistent
        .merge_observation(&observe(2.0, vec![point![1.025, 0.025, 0.0]]))
        .unwrap();
    let at = |persistent: &PersistentOccupancyGrid| {
        let grid = &persistent.grid;
        let x = ((1.025 - grid.origin[0]) / grid.resolution).floor() as u32;
        let y = ((0.025 - grid.origin[1]) / grid.resolution).floor() as u32;
        grid.data[grid.index(y, x)]
    };
    assert_eq!(at(&persistent), 100);
    // 终点不在水平切片内，但射线真实穿过旧障碍格；同帧重投不能重复投票。
    for timestamp in [3.0, 4.0] {
        let free = observe(timestamp, vec![point![4.025, 0.025, 0.2]]);
        persistent.merge_observation(&free).unwrap();
        persistent.merge_observation(&free).unwrap();
        assert_eq!(at(&persistent), 100);
    }
    persistent
        .merge_observation(&observe(5.0, vec![point![4.025, 0.025, 0.2]]))
        .unwrap();
    assert_eq!(at(&persistent), 0);
}

#[test]
fn planar_observations_confirm_hits_and_clears_without_reusing_timestamps() {
    let mut scan = OccupancyGrid::with_frame(3, 1, 0.05, "map");
    scan.data = vec![0, 100, -1];
    scan.ts = 1.0;
    let mut map = PersistentOccupancyGrid::from_observation(&scan).unwrap();
    assert_eq!(map.grid.data, [0, -1, -1]);
    map.merge_observation(&scan).unwrap();
    assert_eq!(map.grid.data[1], -1);
    scan.ts = 2.0;
    map.merge_observation(&scan).unwrap();
    assert_eq!(map.grid.data, [0, 100, -1]);
    scan.data[1] = 0;
    for ts in [3.0, 4.0] {
        scan.ts = ts;
        map.merge_observation(&scan).unwrap();
        assert_eq!(map.grid.data[1], 100);
    }
    scan.ts = 3.5;
    map.merge_observation(&scan).unwrap();
    assert_eq!(map.grid.data[1], 100);
    scan.ts = 5.0;
    scan.data.fill(-1);
    map.merge_observation(&scan).unwrap();
    assert_eq!(map.grid.data[1], 100);
    scan.ts = 6.0;
    scan.data[1] = 0;
    map.merge_observation(&scan).unwrap();
    assert_eq!(map.grid.data, [0, 0, -1]);
    let restored = PersistentOccupancyGrid::from_grid(map.grid.clone()).unwrap();
    assert_eq!(restored.grid.data, map.grid.data);
}

#[test]
fn planar_restored_map_accepts_new_source_clock_without_reusing_pending_votes() {
    let mut saved = OccupancyGrid::with_frame(2, 1, 0.05, "map");
    saved.data = vec![100, -1];
    saved.ts = 10000.0;
    let mut restored = PersistentOccupancyGrid::from_grid(saved.clone()).unwrap();
    let mut observation = saved;
    observation.data = vec![0, 100];
    for timestamp in [1.0, 2.0] {
        observation.ts = timestamp;
        restored.merge_observation(&observation).unwrap();
    }
    assert_eq!(restored.grid.data, [100, 100]);
    observation.ts = 3.0;
    restored.merge_observation(&observation).unwrap();
    assert_eq!(restored.grid.data, [0, 100]);
}

#[test]
fn planar_missing_returns_do_not_expand_or_erase_history() {
    let mut old = OccupancyGrid::with_frame(2, 1, 0.05, "map");
    old.origin[0] = 100.0;
    old.data = vec![0, 100];
    old.ts = 1.0;
    let mut map = PersistentOccupancyGrid::from_grid(old.clone()).unwrap();
    let mut absent = OccupancyGrid::with_frame(1, 1, 0.05, "map");
    absent.ts = 2.0;
    map.merge_observation(&absent).unwrap();
    assert_eq!(map.grid.origin, old.origin);
    assert_eq!(map.grid.data, old.data);
    assert_eq!(map.grid.ts, 2.0);
}

#[test]
fn long_thin_grid_keeps_projection_memory_bounded() {
    let target = OccupancyGrid::with_frame(40_000, 1, 0.05, "map");
    let source = OccupancyGrid::with_frame(20_000, 1, 0.05, "map");
    let persistent = PersistentOccupancyGrid::from_grid(target.clone()).unwrap();
    let projection = GridProjection::new(&target, &source);
    assert!(projection.rows.is_empty() && projection.columns.is_empty());
    for col in [0, 1, 16_383, 16_384, 19_999] {
        assert_eq!(
            projection.project(0, col),
            persistent.project_cell(&source, 0, col)
        );
    }
}

#[test]
fn cached_axes_match_original_cell_projection_including_boundaries() {
    for origin in [[-20.05, 3.15, 0.0], [0.0, 0.0, 0.0], [1e6, -1e6, 0.0]] {
        let mut target = OccupancyGrid::with_frame(127, 139, 0.05, "map");
        target.origin = origin;
        let persistent = PersistentOccupancyGrid::from_grid(target.clone()).unwrap();
        for shift in [-2.0, -1e-7, 0.0, 0.5, 1.0, 8.0] {
            let mut source = OccupancyGrid::with_frame(31, 37, 0.05, "map");
            source.origin = [origin[0] + shift * 0.05, origin[1] - shift * 0.05, 0.0];
            let projection = GridProjection::new(&target, &source);
            for row in 0..source.height {
                for col in 0..source.width {
                    assert_eq!(
                        projection.project(row, col),
                        persistent.project_cell(&source, row, col)
                    );
                }
            }
        }
    }
}

#[test]
fn in_place_merge_matches_cloned_staging_over_one_observation_sequence() {
    // 旧做法：先克隆整个持久图再合并，只留在测试里作对照。
    fn staged(current: Option<&PersistentOccupancyGrid>, latest: &OccupancyGrid) -> PersistentOccupancyGrid {
        let mut staged = match current {
            Some(current) => current.clone(),
            None => {
                let mut unknown = latest.clone();
                unknown.data.fill(-1);
                PersistentOccupancyGrid::from_grid(unknown).unwrap()
            }
        };
        staged.merge_observation(latest).unwrap();
        staged
    }
    let mut seed = 0x2545_f491_4f6c_dd1du64;
    let mut next = move |bound: u64| {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        seed % bound
    };
    let mut reference: Option<PersistentOccupancyGrid> = None;
    let mut in_place: Option<PersistentOccupancyGrid> = None;
    let mut ts = 0.0;
    for step in 0..400 {
        let mut latest = OccupancyGrid::with_frame(
            5 + next(20) as u32, 5 + next(20) as u32, 0.05, "map");
        // 原点在格线上左右移动，覆盖持久图扩张。
        latest.origin = [(next(60) as f64 - 30.) * 0.05, (next(60) as f64 - 30.) * 0.05, 0.];
        for value in &mut latest.data {
            *value = [-1, 0, 30, 65, 100][next(5) as usize];
        }
        if step % 37 == 5 {
            latest.data.fill(-1);
        }
        // 偶尔重复或倒退源时间，覆盖同帧不重复投票。
        ts += [1.0, 1.0, 1.0, 0.0, -0.5][next(5) as usize];
        latest.ts = ts;
        reference = Some(staged(reference.as_ref(), &latest));
        PersistentOccupancyGrid::merge_or_create(&mut in_place, &latest).unwrap();
        let (a, b) = (reference.as_ref().unwrap(), in_place.as_ref().unwrap());
        assert_eq!((a.grid.width, a.grid.height, a.grid.origin, a.grid.ts),
            (b.grid.width, b.grid.height, b.grid.origin, b.grid.ts), "第{step}步栅格范围");
        assert_eq!(a.grid.data, b.grid.data, "第{step}步占据值");
        assert_eq!(a.pending_values, b.pending_values, "第{step}步待确认值");
        assert_eq!(a.pending_counts, b.pending_counts, "第{step}步待确认计数");
        assert_eq!(a.last_observation_timestamp, b.last_observation_timestamp, "第{step}步时间水位");
    }
    let last = in_place.unwrap();
    assert!(last.pending_counts.iter().any(|count| *count > 0), "序列须留下未确认的票");
    assert!(last.grid.width > 25 && last.grid.height > 25, "序列须让持久图扩张");
}
