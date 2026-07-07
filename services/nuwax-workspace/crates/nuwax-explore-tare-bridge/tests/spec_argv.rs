//! `spec_argv` 集成测试:验证 `build_specs` 后用 `native-runner` 渲染完整 argv;
//! `to_native_args(config)` 单测只含 config args。
//!
//! 验收点(方案 §5.4):
//! - port 名固定:`registered_scan` / `odometry` / `way_point`
//! - 参数顺序稳定,且 topics **不在** `NativeProgramSpec.args` 中重复
//! - 通过 `NativeProgramSpec::rendered_args()` 验证最终 argv

use nuwax_explore_tare_bridge::{
    build_specs, to_native_args, TarePlannerBridgeConfig, TarePlannerTopics,
};

#[test]
fn build_specs_args_contain_config_only() {
    let cfg = TarePlannerBridgeConfig::default();
    let topics = TarePlannerTopics {
        registered_scan: Some("/registered_scan".into()),
        odometry: Some("/odometry".into()),
        way_point: Some("/way_point".into()),
    };
    let (spec, _build) = build_specs(cfg.clone(), &topics).expect("valid config");
    // spec.args 只应包含 config args,不含 topics
    let joined = spec.args.join(" ");
    assert!(joined.contains("--explorationRange"));
    assert!(joined.contains("--updateRate"));
    assert!(joined.contains("--sensorRange"));
    assert!(
        !joined.contains("--registeredScan"),
        "topic leaked into args: {joined}"
    );
    assert!(
        !joined.contains("--odometry"),
        "topic leaked into args: {joined}"
    );
    assert!(
        !joined.contains("--wayPoint"),
        "topic leaked into args: {joined}"
    );
    // topics 仍然在 spec.topics 中
    assert_eq!(spec.topics.len(), 3);
}

#[test]
fn to_native_args_argv_order_is_stable() {
    let cfg = TarePlannerBridgeConfig {
        exploration_range: 12.5,
        update_rate: 2.0,
        sensor_range: 8.0,
        ..TarePlannerBridgeConfig::default()
    };
    let args = to_native_args(&cfg);
    assert_eq!(
        args,
        vec![
            "--explorationRange".to_string(),
            "12.5".to_string(),
            "--updateRate".to_string(),
            "2".to_string(),
            "--sensorRange".to_string(),
            "8".to_string(),
        ]
    );
}

#[test]
fn to_topic_map_uses_fixed_port_names() {
    let topics = TarePlannerTopics {
        registered_scan: Some("/a".into()),
        odometry: Some("/b".into()),
        way_point: Some("/c".into()),
    };
    let m = topics.to_topic_map();
    let keys: Vec<&String> = m.keys().collect();
    // port 名固定;native-runner 会按字典序渲染
    assert_eq!(
        keys,
        vec![
            &"odometry".to_string(),
            &"registered_scan".to_string(),
            &"way_point".to_string(),
        ]
    );
}

#[test]
fn build_specs_propagates_restart_policy_as_single_source() {
    let cfg = TarePlannerBridgeConfig {
        restart_policy: nuwax_nav_base::native::RestartPolicy::Never,
        ..TarePlannerBridgeConfig::default()
    };
    let (spec, _) = build_specs(cfg.clone(), &TarePlannerTopics::default()).unwrap();
    assert_eq!(spec.restart, cfg.restart_policy);
}

#[test]
fn build_specs_rendered_args_include_topics_and_config() {
    use nuwax_nav_native_runner::rendered_args::render_args;
    let cfg = TarePlannerBridgeConfig::default();
    let topics = TarePlannerTopics {
        registered_scan: Some("/registered_scan".into()),
        odometry: Some("/odometry".into()),
        way_point: Some("/way_point".into()),
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
        .any(|w| w == ["--way_point", "/way_point"]));
    // config args
    assert!(rendered
        .windows(2)
        .any(|w| w == ["--explorationRange", "20"]));
    assert!(rendered.windows(2).any(|w| w == ["--updateRate", "1"]));
    assert!(rendered.windows(2).any(|w| w == ["--sensorRange", "20"]));
}
