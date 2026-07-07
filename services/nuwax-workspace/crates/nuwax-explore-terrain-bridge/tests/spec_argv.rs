//! `spec_argv` 集成测试
//!
//! 验证:
//! - `build_specs` 后用 `native-runner` `render_args()` 渲染完整 argv
//! - `to_native_args(config)` 单测只含 config args,topics 不重复
//! - port 名固定 `registered_scan` / `odometry` / `terrain_map`
//! - CLI 名遵循 camelCase(`--sensorRange` / `--maxGroundLift` 等)

use nuwax_explore_terrain_bridge::{
    build_specs, cli_exclude, cli_name_override, to_native_args, validate_config,
    TerrainAnalysisBridgeConfig, TerrainAnalysisTopics,
};
use nuwax_nav_native_runner::rendered_args::render_args;

#[test]
fn build_specs_args_contain_config_only() {
    let cfg = TerrainAnalysisBridgeConfig::default();
    let topics = TerrainAnalysisTopics {
        registered_scan: Some("/registered_scan".into()),
        odometry: Some("/odometry".into()),
        terrain_map: Some("/terrain_map".into()),
    };
    let (spec, _build) = build_specs(cfg.clone(), &topics).expect("valid config");
    let joined = spec.args.join(" ");
    assert!(joined.contains("--sensorRange"));
    assert!(joined.contains("--scanVoxelSize"));
    assert!(joined.contains("--terrainVoxelSize"));
    assert!(joined.contains("--terrainVoxelHalfWidth"));
    assert!(joined.contains("--obstacleHeightThre"));
    assert!(joined.contains("--groundHeightThre"));
    // topics 不在 args 中重复
    assert!(!joined.contains("--registeredScan"), "leaked: {joined}");
    assert!(!joined.contains("--terrain_map"), "leaked: {joined}");
    assert_eq!(spec.topics.len(), 3);
}

#[test]
fn to_native_args_argv_order_is_stable() {
    let cfg = TerrainAnalysisBridgeConfig::default();
    let args = to_native_args(&cfg);
    let joined = args.join(" ");
    // 默认 6 个 required 字段按固定顺序出现
    let sensors_pos = joined.find("--sensorRange").expect("sensorRange");
    let scan_pos = joined.find("--scanVoxelSize").expect("scanVoxelSize");
    let terrain_pos = joined.find("--terrainVoxelSize").expect("terrainVoxelSize");
    let half_width_pos = joined
        .find("--terrainVoxelHalfWidth")
        .expect("terrainVoxelHalfWidth");
    let obstacle_pos = joined
        .find("--obstacleHeightThre")
        .expect("obstacleHeightThre");
    let ground_pos = joined.find("--groundHeightThre").expect("groundHeightThre");
    assert!(sensors_pos < scan_pos);
    assert!(scan_pos < terrain_pos);
    assert!(terrain_pos < half_width_pos);
    assert!(half_width_pos < obstacle_pos);
    assert!(obstacle_pos < ground_pos);
}

#[test]
fn to_topic_map_uses_fixed_port_names() {
    let topics = TerrainAnalysisTopics {
        registered_scan: Some("/a".into()),
        odometry: Some("/b".into()),
        terrain_map: Some("/c".into()),
    };
    let m = topics.to_topic_map();
    let keys: Vec<&String> = m.keys().collect();
    // port 名固定;native-runner 会按字典序渲染
    assert_eq!(
        keys,
        vec![
            &"odometry".to_string(),
            &"registered_scan".to_string(),
            &"terrain_map".to_string(),
        ]
    );
}

#[test]
fn build_specs_propagates_restart_policy_as_single_source() {
    use nuwax_nav_base::native::RestartPolicy;
    let cfg = TerrainAnalysisBridgeConfig {
        restart_policy: RestartPolicy::Never,
        ..TerrainAnalysisBridgeConfig::default()
    };
    let (spec, _) = build_specs(cfg.clone(), &TerrainAnalysisTopics::default()).unwrap();
    assert_eq!(spec.restart, cfg.restart_policy);
}

#[test]
fn build_specs_rendered_args_include_topics_and_config() {
    let cfg = TerrainAnalysisBridgeConfig::default();
    let topics = TerrainAnalysisTopics {
        registered_scan: Some("/registered_scan".into()),
        odometry: Some("/odometry".into()),
        terrain_map: Some("/terrain_map".into()),
    };
    let (spec, _) = build_specs(cfg, &topics).unwrap();
    let rendered = render_args(&spec);
    // topics 出现在 rendered argv(port 字典序)
    assert!(rendered
        .windows(2)
        .any(|w| w == ["--odometry", "/odometry"]));
    assert!(rendered
        .windows(2)
        .any(|w| w == ["--registered_scan", "/registered_scan"]));
    assert!(rendered
        .windows(2)
        .any(|w| w == ["--terrain_map", "/terrain_map"]));
    // config args
    assert!(rendered.windows(2).any(|w| w == ["--sensorRange", "20"]));
    assert!(rendered
        .windows(2)
        .any(|w| w == ["--groundHeightThre", "0.1"]));
}

#[test]
fn cli_name_override_covers_all_optional_fields() {
    let m = cli_name_override();
    assert!(m.contains_key("vehicle_height"));
    assert!(m.contains_key("use_sorting"));
    assert!(m.contains_key("decay_time"));
    assert!(m.contains_key("min_dynamic_obstacle_distance"));
    assert!(m.contains_key("max_ground_lift"));
    assert!(m.contains_key("distance_ratio_z"));
}

#[test]
fn cli_exclude_does_not_contain_typed_fields() {
    let s = cli_exclude();
    assert!(s.contains("executable"));
    assert!(s.contains("cwd"));
    assert!(s.contains("env"));
    assert!(s.contains("build_command"));
    assert!(s.contains("build_policy"));
    assert!(s.contains("restart_policy"));
    // 不应排除 typed config 字段
    assert!(!s.contains("sensor_range"));
    assert!(!s.contains("max_ground_lift"));
}

#[test]
fn validate_config_default_is_ok() {
    assert!(validate_config(&TerrainAnalysisBridgeConfig::default()).is_ok());
}
