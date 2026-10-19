// DRAFT ONLY: include inside Scan's existing tests module. Internal reach-branch
// qualification; this is not a substitute for public Full Scan or an ON chain.
#[test]
fn zero_arrival_direct_search_accepts_exact_original_pose() {
    let map = coarse_open_floor();
    let tomo = Tomogram::from_surface(&map).unwrap();
    let start = ([0.0, 0.2, 0.4], [0.0, 0.0, 0.0, 1.0]);
    map.validate_body_pose(start.0, start.1).unwrap();
    let goal = start.0;
    let config = ScanConfig::default();
    assert_eq!(config.search_expansion_limit, DEFAULT_SEARCH_EXPANSION_LIMIT);
    let path = local_search::plan(&map, &tomo, start,
        &[[0.0, 0.2, 0.0], [goal[0], goal[1], 0.0]], &motion(), Some(0.0),
        Some((goal, 0.0)), &[], 3.0, 0.9, true, config.search_expansion_limit).unwrap();
    assert_eq!(path, vec![start], "zero reach includes the original exact finite pose");
}
