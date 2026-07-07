//! `optional_fields` 集成测试
//!
//! 验证:
//! - `None` 字段不进入 `to_native_args` 输出
//! - `Some(_)` 字段进入 argv 并使用 camelCase 覆盖名
//! - 默认 config(全部 Option=None)只渲染 6 个 required 字段

use nuwax_explore_terrain_bridge::{
    build_specs, to_native_args, TerrainAnalysisBridgeConfig, TerrainAnalysisTopics,
};

#[test]
fn none_fields_are_not_rendered() {
    let cfg = TerrainAnalysisBridgeConfig::default();
    let args = to_native_args(&cfg);
    let joined = args.join(" ");
    // 所有 Option 字段默认 None,不应出现以下 flag
    for flag in &[
        "--vehicleHeight",
        "--minRelZ",
        "--maxRelZ",
        "--useSorting",
        "--quantileZ",
        "--decayTime",
        "--noDecayDis",
        "--clearingDis",
        "--clearDyObs",
        "--noDataObstacle",
        "--noDataBlockSkipNum",
        "--minBlockPointNum",
        "--voxelPointUpdateThre",
        "--voxelTimeUpdateThre",
        "--minDyObsDis",
        "--absDyObsRelZThre",
        "--minDyObsVFOV",
        "--maxDyObsVFOV",
        "--minDyObsPointNum",
        "--minOutOfFovPointNum",
        "--considerDrop",
        "--limitGroundLift",
        "--maxGroundLift",
        "--disRatioZ",
    ] {
        assert!(!joined.contains(flag), "should not render None: {flag}");
    }
}

#[test]
fn some_fields_are_rendered_with_overrides() {
    let cfg = TerrainAnalysisBridgeConfig {
        vehicle_height: Some(0.5),
        use_sorting: Some(true),
        decay_time: Some(4.0),
        max_ground_lift: Some(0.2),
        distance_ratio_z: Some(0.3),
        limit_ground_lift: Some(false),
        consider_drop: Some(true),
        ..TerrainAnalysisBridgeConfig::default()
    };
    let args = to_native_args(&cfg);
    let joined = args.join(" ");
    assert!(joined.contains("--vehicleHeight 0.5"));
    assert!(joined.contains("--useSorting true"));
    assert!(joined.contains("--decayTime 4"));
    assert!(joined.contains("--maxGroundLift 0.2"));
    assert!(joined.contains("--disRatioZ 0.3"));
    assert!(joined.contains("--limitGroundLift false"));
    assert!(joined.contains("--considerDrop true"));
}

#[test]
fn default_only_emits_six_required_flags() {
    let cfg = TerrainAnalysisBridgeConfig::default();
    let args = to_native_args(&cfg);
    // 默认 6 个 required 字段全部出现
    let expected = &[
        "--sensorRange",
        "--scanVoxelSize",
        "--terrainVoxelSize",
        "--terrainVoxelHalfWidth",
        "--obstacleHeightThre",
        "--groundHeightThre",
    ];
    for flag in expected {
        assert!(args.iter().any(|a| a == flag), "missing flag {flag}");
    }
    // 默认 args 数量 = 6 required × 2 = 12
    assert_eq!(args.len(), 12, "unexpected argv: {args:?}");
}

#[test]
fn integer_optional_fields_render_as_numbers() {
    let cfg = TerrainAnalysisBridgeConfig {
        min_block_point_count: Some(5),
        no_data_block_skip_count: Some(3),
        min_dynamic_obstacle_point_count: Some(7),
        ..TerrainAnalysisBridgeConfig::default()
    };
    let args = to_native_args(&cfg);
    let joined = args.join(" ");
    assert!(joined.contains("--minBlockPointNum 5"));
    assert!(joined.contains("--noDataBlockSkipNum 3"));
    assert!(joined.contains("--minDyObsPointNum 7"));
}

#[test]
fn build_specs_does_not_emit_topic_in_args_for_any_topics() {
    let cfg = TerrainAnalysisBridgeConfig::default();
    let topics = TerrainAnalysisTopics {
        registered_scan: None,
        odometry: None,
        terrain_map: Some("/terrain_map".into()),
    };
    let (spec, _) = build_specs(cfg, &topics).expect("valid");
    // 即使 topic 全部 None / 部分 None,args 也不应包含 topic argv
    for a in &spec.args {
        assert!(!a.starts_with("--registered"), "leaked: {a}");
        assert!(!a.starts_with("--odometry"), "leaked: {a}");
        assert!(!a.starts_with("--terrain_map"), "leaked: {a}");
    }
}
