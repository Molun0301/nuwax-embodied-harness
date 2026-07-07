//! profile defaults 验收(对应方案 §11.2 `profile_defaults_match_selected_profile`)

use nuwax_explore_terrain_map_bridge::ExplorationTerrainMapExtProfile;

#[test]
fn profile_defaults_name_is_documented() {
    let p = ExplorationTerrainMapExtProfile::default_exploration();
    assert_eq!(p.profile_name, "exploration/default");
}

#[test]
fn profile_defaults_match_selected_profile() {
    // 方案 §7.3 声明的探索 profile 常量必须严格匹配。
    let p = ExplorationTerrainMapExtProfile::default_exploration();
    assert!((p.config.scan_voxel_size - 0.1).abs() < 1e-9);
    assert!((p.config.decay_time - 4.0).abs() < 1e-9);
    assert!(p.config.use_sorting);
    assert!((p.config.quantile_z - 0.1).abs() < 1e-9);
    assert!((p.config.lower_bound_z - -2.5).abs() < 1e-9);
}
