use super::*;

fn scene(clearance: f32) -> SurfaceSnapshot { scene_sized(clearance, 24, 16) }

fn scene_sized(clearance: f32, width: i32, height: i32) -> SurfaceSnapshot {
    let config = Config { world_frame: "map".into(), voxel_size: 0.1, robot_height: 0.3,
        surface_closing_radius: 0.0, node_spacing_m: 0.3, wall_clearance_m: clearance,
        wall_buffer_m: 0.1, wall_buffer_weight: 1.0, step_threshold_m: 0.2,
        max_slope_deg: 25.0, min_step_tread_depth_m: 0.2, step_penalty_weight: 1.0,
        goal_tolerance: 0.1, viz_publish_hz: 0.0 };
    let points: Vec<_> = (0..width).flat_map(|x| (0..height).map(move |y|
        ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.05))).collect();
    SurfaceSnapshot::from_points(&points, config, 0.1).unwrap()
}

fn wide_mask(line: bool) -> Arc<VirtualObstacleMask> {
    let mut blocked = vec![false; 60 * 60];
    if line { for y in 0..60 { blocked[y * 60 + 32] = true; } }
    Arc::new(VirtualObstacleMask::new([-2.0, -2.0], 0.1, 60, 60, &blocked).unwrap())
}

// Frozen old graph derivation, independent of the candidate's graph/region branch.
fn cold_reference(input: &SurfaceSnapshot, mask: Arc<VirtualObstacleMask>) -> SurfaceSnapshot {
    let mut graph = input.data.physical_graph.as_ref().clone();
    let excluded: AHashSet<_> = graph.cells.ids().filter(|&id| !mask.permits_cell(
        graph.cells.coord(id), f64::from(input.config().voxel_size),
        f64::from(input.config().wall_clearance_m))).collect();
    for (id, edges) in graph.cells.iter_edges_mut() {
        edges.retain(|edge| !excluded.contains(&id) && !excluded.contains(&edge.dest));
    }
    SurfaceSnapshot {
        recent_obstacles:input.recent_obstacles.clone(),
        ground_evidence: input.ground_evidence,
        observations: input.observations.clone(),
        body_query_cache: Default::default(),
        navigation_measurements: None,
        data: Arc::new(SurfaceData {
            frontier_cache: Default::default(),
            body_cache: Default::default(),
            body_envelope: input.data.body_envelope.clone(),
            height_bounds: Arc::clone(&input.data.height_bounds),
            graph: Arc::new(graph),
            physical_graph: Arc::clone(&input.data.physical_graph),
            occupied_columns: Arc::clone(&input.data.occupied_columns),
            occupied_voxels: input.data.occupied_voxels,
            virtual_obstacles: Some(mask), virtual_blocked_cells: Arc::new(excluded),
            config: input.config().clone(), source_voxel_size_m: input.data.source_voxel_size_m,
            profile_cells: input.data.profile_cells,
            candidate_step_cells: input.data.candidate_step_cells,
        }),
    }
}

fn edge_bits(map: &SurfaceSnapshot) -> Vec<(VoxelKey, Vec<(VoxelKey, [u32; 3])>)> {
    let mut result: Vec<_> = map.data.graph.cells.ids().map(|id| {
        let mut edges: Vec<_> = map.data.graph.cells.neighbors(id).iter().map(|edge|
            (map.data.graph.cells.coord(edge.dest),
             [edge.base_cost.to_bits(), edge.rise.to_bits(), edge.cost.to_bits()])).collect();
        edges.sort_unstable(); (map.data.graph.cells.coord(id), edges)
    }).collect();
    result.sort_unstable(); result
}

fn equivalent(actual: &SurfaceSnapshot, expected: &SurfaceSnapshot) {
    assert_eq!(edge_bits(actual), edge_bits(expected));
    assert_eq!(actual.data.virtual_blocked_cells, expected.data.virtual_blocked_cells);
    for (from, to) in [((3, 8, 0), (20, 8, 0)), ((3, 5, 0), (7, 10, 0))] {
        let start = actual.cell_position(from); let goal = actual.cell_position(to);
        assert_eq!(
            actual.test_connected(start, goal),
            expected.test_connected(start, goal));
        }
}

#[test]
fn empty_exclusion_reuses_physical_graph_without_edge_writes() {
    let input = scene(0.1); let mask = wide_mask(false);
    let actual = input.with_virtual_mask(mask.clone()).unwrap();
    assert!(actual.data.virtual_blocked_cells.is_empty());
    assert!(Arc::ptr_eq(&actual.data.graph, &input.data.physical_graph));
    assert!(!Arc::ptr_eq(&actual.data, &input.data));
    assert!(Arc::ptr_eq(actual.data.virtual_obstacles.as_ref().unwrap(), &mask));
    for id in input.data.graph.cells.ids() {
        assert_eq!(actual.data.graph.cells.neighbors(id).as_ptr(),
            input.data.graph.cells.neighbors(id).as_ptr());
    }
    let repeated = actual.with_virtual_mask(wide_mask(false)).unwrap();
    assert!(Arc::ptr_eq(&repeated.data.graph, &input.data.physical_graph));
    let extended_mask = Arc::new(VirtualObstacleMask::new([-3.0, -3.0], 0.1,
        80, 80, &vec![false; 80 * 80]).unwrap());
    let extended = actual.with_virtual_mask(extended_mask.clone()).unwrap();
    assert!(Arc::ptr_eq(&extended.data.graph, &actual.data.graph));
    assert!(!Arc::ptr_eq(&extended.data, &actual.data));
    // Both masks permit all current graph cells, but their finite domains differ.
    assert!(!actual.data.virtual_obstacles.as_ref().unwrap().permits_cell((-25, 0, 0), 0.1, 0.1));
    assert!(extended.data.virtual_obstacles.as_ref().unwrap().permits_cell((-25, 0, 0), 0.1, 0.1));
    assert!(Arc::ptr_eq(extended.data.virtual_obstacles.as_ref().unwrap(), &extended_mask));
    equivalent(&actual, &cold_reference(&input, mask));
    eprintln!("empty exclusion structural work: {} slots, {} live edges; graph/edge storage reused",
        input.data.graph.cells.slot_capacity(),
        input.data.graph.cells.ids().map(|id| input.data.graph.cells.neighbors(id).len()).sum::<usize>());
}

#[test]
fn empty_exclusion_after_masked_source_restores_physical_edges() {
    let physical = scene(0.1);
    let blocked = physical.with_virtual_mask(wide_mask(true)).unwrap();
    assert!(!blocked.data.virtual_blocked_cells.is_empty());
    assert!(!Arc::ptr_eq(&blocked.data.graph, &physical.data.physical_graph));
    let retained = edge_bits(&blocked);
    let mask = wide_mask(false); let restored = blocked.with_virtual_mask(mask.clone()).unwrap();
    assert!(restored.data.virtual_blocked_cells.is_empty());
    assert!(Arc::ptr_eq(&restored.data.graph, &physical.data.physical_graph));
    assert_eq!(edge_bits(&restored), edge_bits(&physical));
    assert_eq!(edge_bits(&blocked), retained);
    equivalent(&restored, &cold_reference(&blocked, mask));
}

#[test]
fn empty_exclusion_body_query_shares_unchanged_geometry() {
    let physical = scene(0.0);
    let query = physical.clone();
    assert!(Arc::ptr_eq(&query.data.graph, &physical.data.graph));
    let mask = wide_mask(false); let actual = query.with_virtual_mask(mask.clone()).unwrap();
    assert!(actual.data.virtual_blocked_cells.is_empty());
    assert!(Arc::ptr_eq(&actual.data.graph, &query.data.physical_graph));
    equivalent(&actual, &cold_reference(&query, mask));
}

#[test]
fn all_free_finite_extent_and_clearance_still_exclude_boundary_cells() {
    let input = scene(0.1);
    let mask = Arc::new(VirtualObstacleMask::new([0.0, 0.0], 0.1, 24, 16,
        &vec![false; 24 * 16]).unwrap());
    let actual = input.with_virtual_mask(mask.clone()).unwrap();
    assert!(!actual.data.virtual_blocked_cells.is_empty());
    assert!(!Arc::ptr_eq(&actual.data.graph, &input.data.physical_graph));
    assert!(!actual.cell_is_clear((0, 8, 0)));
    assert!(actual.cell_is_clear((5, 8, 0)));
    equivalent(&actual, &cold_reference(&input, mask));
}

#[test]
fn nonempty_exclusion_keeps_cold_edge_and_route_behavior() {
    let input = scene(0.1); let mask = wide_mask(true);
    let actual = input.with_virtual_mask(mask.clone()).unwrap();
    assert!(!actual.data.virtual_blocked_cells.is_empty());
    assert!(!Arc::ptr_eq(&actual.data.graph, &input.data.physical_graph));
    equivalent(&actual, &cold_reference(&input, mask));
}

#[test]
#[ignore = "Opt-in paired release diagnostic; fixtures outside timing; wait for release window"]
fn empty_exclusion_derivation_cost_baseline() {
    use std::hint::black_box;
    use std::time::Instant;
    for (width, height) in [(24, 16), (64, 48), (128, 96)] {
        let source = scene_sized(0.1, width, height);
        let (mw, mh) = ((width + 40) as u32, (height + 40) as u32);
        let mask = Arc::new(VirtualObstacleMask::new([-2.0, -2.0], 0.1, mw, mh,
            &vec![false; (mw * mh) as usize]).unwrap());
        let current = source.with_virtual_mask(mask.clone()).unwrap();
        assert!(current.data.virtual_blocked_cells.is_empty());
        equivalent(&current, &cold_reference(&source, mask.clone()));
        drop(current);
        // Both arms retain the same source and immutable mask, build a complete
        // derived snapshot, and retire that result before the timer stops.
        let mut old_ns = Vec::new(); let mut new_ns = Vec::new();
        for iteration in 0..18 {
            for old in if iteration % 2 == 0 { [true, false] } else { [false, true] } {
                let start = Instant::now();
                let result = if old { cold_reference(black_box(&source), black_box(mask.clone())) }
                    else { black_box(&source).with_virtual_mask(black_box(mask.clone())).unwrap() };
                drop(black_box(result));
                let elapsed = start.elapsed().as_nanos();
                if iteration >= 2 { if old { old_ns.push(elapsed); } else { new_ns.push(elapsed); } }
            }
        }
        old_ns.sort_unstable(); new_ns.sort_unstable();
        eprintln!("EMPTY_MASK_COST width={width} height={height} slots={} samples={} old_ns={old_ns:?} new_ns={new_ns:?}",
            source.data.graph.cells.slot_capacity(), old_ns.len());
    }
}

#[test]
fn retained_mask_rederives_new_physical_graph_exactly_like_full_construction() {
    for blocked_line in [false, true] {
        let mut builder = SurfaceMapBuilder::new(scene(0.1).config().clone());
        let columns: Vec<_> = (0..48).flat_map(|x| (0..16).map(move |y|
            SurfaceColumnUpdate { column: [x, y], cells: vec![(0, [0.0, 0.1])] })).collect();
        let old_physical = builder.build_column_updates(&columns, 0.1, true, columns.len())
            .unwrap().snapshot;
        // Even the all-free case has finite bounds that exclude boundary cells.
        let mut blocked = vec![false; 48 * 16];
        if blocked_line { for y in 0..16 { blocked[y * 48 + 12] = true; } }
        let old = old_physical.with_virtual_obstacles([0.0, 0.0], 0.1, 48, 16, &blocked).unwrap();
        let old_edges = edge_bits(&old);
        let changes = [
            SurfaceColumnUpdate { column: [6, 8], cells: Vec::new() },
            SurfaceColumnUpdate { column: [35, 8], cells: (0..8)
                .map(|z| (z, [z as f32 * 0.1, (z + 1) as f32 * 0.1])).collect() },
        ];
        let physical = builder.build_column_updates(&changes, 0.1, false, 8).unwrap().snapshot;
        assert!(!Arc::ptr_eq(&physical, &old_physical));
        assert!(physical.data.virtual_obstacles.is_none());
        assert!(physical.data.graph.cells.id((6, 8, 0)).is_none());
        let physical = physical.as_ref().clone().with_ground_evidence(GroundEvidence::OccupiedVolume);
        assert_ne!(physical.ground_evidence(), old.ground_evidence());
        let actual = physical.with_retained_virtual_obstacles_from(&old).unwrap().unwrap();
        let expected = physical.with_virtual_obstacles([0.0, 0.0], 0.1, 48, 16, &blocked).unwrap();
        assert!(Arc::ptr_eq(actual.data.virtual_obstacles.as_ref().unwrap(),
            old.data.virtual_obstacles.as_ref().unwrap()));
        assert!(!Arc::ptr_eq(actual.data.virtual_obstacles.as_ref().unwrap(),
            expected.data.virtual_obstacles.as_ref().unwrap()));
        assert!(Arc::ptr_eq(&actual.data.physical_graph, &physical.data.physical_graph));
        assert!(Arc::ptr_eq(&actual.data.height_bounds, &physical.data.height_bounds));
        assert!(Arc::ptr_eq(&actual.data.occupied_columns, &physical.data.occupied_columns));
        assert!(!Arc::ptr_eq(&actual.data.graph, &old.data.graph));
        assert!(!Arc::ptr_eq(&actual.data.virtual_blocked_cells, &old.data.virtual_blocked_cells));
        assert_eq!(actual.ground_evidence(), physical.ground_evidence());
        assert_eq!(actual.data.occupied_voxels, physical.data.occupied_voxels);
        assert!(!actual.cell_is_clear((0, 8, 0)));
        equivalent(&actual, &expected);
        assert_eq!(edge_bits(&old), old_edges);
        assert_ne!(edge_bits(&actual), old_edges);
        for id in actual.data.graph.cells.ids() {
            let key = actual.data.graph.cells.coord(id);
            let expected_id = expected.data.graph.cells.id(key).unwrap();
            assert_eq!(actual.cell_position(key).map(f64::to_bits),
                expected.cell_position(key).map(f64::to_bits));
            assert_eq!(actual.cell_is_clear(key), expected.cell_is_clear(key));
            assert_eq!(actual.data.graph.wall_state.dist[id as usize].to_bits(),
                expected.data.graph.wall_state.dist[expected_id as usize].to_bits());
        }
        for (from, to) in [((3, 8, 0), (42, 8, 0)), ((20, 5, 0), (42, 10, 0)),
            ((3, 5, 0), (7, 10, 0))] {
            assert_eq!(
                actual.test_connected(actual.cell_position(from), actual.cell_position(to)),
                expected.test_connected(expected.cell_position(from), expected.cell_position(to)));
                    }
    }
}

#[test]
fn retained_mask_absence_is_explicit_and_does_not_unmask_receiver() {
    let physical = scene(0.1);
    let masked = physical.with_virtual_mask(wide_mask(true)).unwrap();
    let before = edge_bits(&masked);
    assert!(masked.with_retained_virtual_obstacles_from(&physical).is_none());
    assert_eq!(edge_bits(&masked), before);
    assert!(masked.data.virtual_obstacles.is_some());
}
