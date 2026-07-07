//! Local Planner bridge argv 渲染 + golden fixture 集成测试
//!
//! 使用 `nuwax-nav-test-fixtures::mock_native_path()` 共享 helper 定位 mock-native,
//! 不依赖 `CARGO_BIN_EXE_mock-native` 在每个 bridge crate 必须被注入的细节。
//! 包含 `local_planner_golden_argv_matches_fixture` synthetic golden fixture 测试。
//!
//! **lifecycle / failed_exit 集成测试已迁移到**:
//! `crates/nuwax-nav-test-fixtures/tests/bridge_lifecycle.rs::local_planner`
//! —— 那处由 cargo 自动注入 `CARGO_BIN_EXE_mock-native`,在任意
//! `CARGO_TARGET_DIR`(包括全新临时目录)下都不需要预构建。

use std::collections::BTreeMap;

use nuwax_nav_local_planner_bridge::{
    cli_exclude, LocalPlannerBridge, LocalPlannerBridgeConfig, LocalPlannerTopics,
};
use nuwax_nav_test_fixtures::goldens::load as load_golden;
use nuwax_nav_test_fixtures::mock_native_path;

fn mock_native_bin() -> std::path::PathBuf {
    mock_native_path()
}

fn mock_config(extra_env: BTreeMap<String, String>) -> LocalPlannerBridgeConfig {
    LocalPlannerBridgeConfig {
        executable: mock_native_bin(),
        env: extra_env,
        restart_policy: nuwax_nav_local_planner_bridge::LocalPlannerRestartPolicy::Never,
        ..LocalPlannerBridgeConfig::default()
    }
}

#[test]
fn local_planner_bridge_argv_matches_expected() {
    let cfg = mock_config(BTreeMap::new());
    let topics = LocalPlannerTopics {
        registered_scan: Some("/scan".to_string()),
        odometry: Some("/odom".to_string()),
        terrain_map: Some("/terrain".to_string()),
        joy_cmd: Some("/joy".to_string()),
        way_point: Some("/way_point".to_string()),
        goal_pose: Some("/goal_pose".to_string()),
        speed: Some("/speed".to_string()),
        navigation_boundary: Some("/nav_boundary".to_string()),
        added_obstacles: Some("/added_obstacles".to_string()),
        check_obstacle: Some("/check_obstacle".to_string()),
        cancel_goal: Some("/cancel_goal".to_string()),
        path: Some("/path".to_string()),
        effective_cmd_vel: Some("/effective_cmd_vel".to_string()),
        free_paths: Some("/free_paths".to_string()),
        slow_down: Some("/slow_down".to_string()),
        goal_reached: Some("/goal_reached".to_string()),
    };
    let bridge = LocalPlannerBridge::new(cfg, &topics);
    let argv = bridge.argv();

    assert!(
        argv[0].contains("mock-native") || argv[0].contains("mock_native"),
        "argv[0] got: {}",
        argv[0]
    );

    // topic args 必须出现
    let argv_joined = argv.join("\n");
    let expected_topics = [
        "/scan",
        "/odom",
        "/terrain",
        "/joy",
        "/way_point",
        "/goal_pose",
        "/speed",
        "/nav_boundary",
        "/added_obstacles",
        "/check_obstacle",
        "/cancel_goal",
        "/path",
        "/effective_cmd_vel",
        "/free_paths",
        "/slow_down",
        "/goal_reached",
    ];
    for t in expected_topics {
        assert!(
            argv_joined.contains(t),
            "topic {t} missing in argv={argv:?}"
        );
    }

    // camelCase 历史名必须出现
    let pairs: Vec<(&str, &str)> = argv
        .windows(2)
        .map(|w| (w[0].as_str(), w[1].as_str()))
        .collect();
    assert!(
        pairs
            .iter()
            .any(|(k, v)| *k == "--maxSpeed" && *v == "0.75"),
        "--maxSpeed 0.75 missing in argv={argv:?}"
    );
    assert!(
        pairs
            .iter()
            .any(|(k, v)| *k == "--vehicleWidth" && *v == "0.5"),
        "--vehicleWidth 0.5 missing in argv={argv:?}"
    );
    assert!(
        pairs
            .iter()
            .any(|(k, v)| *k == "--useTerrainAnalysis" && *v == "true"),
        "--useTerrainAnalysis true missing in argv={argv:?}"
    );

    // None 字段(sensorOffsetX, freezeAng, goalX, sensorOffsetY)不渲染
    assert!(
        !argv.iter().any(|a| a == "--sensorOffsetX"),
        "sensorOffsetX(默认 None)应当被排除"
    );
    assert!(
        !argv.iter().any(|a| a == "--freezeAng"),
        "freezeAng(默认 None)应当被排除"
    );
    assert!(
        !argv.iter().any(|a| a == "--goalX"),
        "goalX(默认 None)应当被排除"
    );

    // 排除字段
    for excluded in cli_exclude() {
        let prefix = format!("--{excluded}");
        assert!(
            !argv.iter().any(|a| a.starts_with(&prefix)),
            "excluded {prefix} leaked into argv={argv:?}"
        );
    }
}

/// Synthetic golden fixture 测试 — 加载 `goldens/local_planner_smoke.json`,
/// 构造 bridge,断言 argv 包含期望 token + pair + 至少 N 个 topic。
#[test]
fn local_planner_golden_argv_matches_fixture() {
    let golden = load_golden("local_planner_smoke");
    assert_eq!(golden.bridge, "local_planner");

    let cfg = mock_config(BTreeMap::new());
    let mut topics = LocalPlannerTopics::default();
    for (port, topic) in &golden.topics {
        match port.as_str() {
            "registered_scan" => topics.registered_scan = Some(topic.clone()),
            "odometry" => topics.odometry = Some(topic.clone()),
            "terrain_map" => topics.terrain_map = Some(topic.clone()),
            "joy_cmd" => topics.joy_cmd = Some(topic.clone()),
            "way_point" => topics.way_point = Some(topic.clone()),
            "goal_pose" => topics.goal_pose = Some(topic.clone()),
            "speed" => topics.speed = Some(topic.clone()),
            "navigation_boundary" => topics.navigation_boundary = Some(topic.clone()),
            "added_obstacles" => topics.added_obstacles = Some(topic.clone()),
            "check_obstacle" => topics.check_obstacle = Some(topic.clone()),
            "cancel_goal" => topics.cancel_goal = Some(topic.clone()),
            "path" => topics.path = Some(topic.clone()),
            "effective_cmd_vel" => topics.effective_cmd_vel = Some(topic.clone()),
            "free_paths" => topics.free_paths = Some(topic.clone()),
            "slow_down" => topics.slow_down = Some(topic.clone()),
            "goal_reached" => topics.goal_reached = Some(topic.clone()),
            _ => {}
        }
    }
    let bridge = LocalPlannerBridge::new(cfg, &topics);
    let argv = bridge.argv();

    let argv_joined = argv.join("\n");
    for token in &golden.expected_argv_contains {
        assert!(
            argv_joined.contains(token),
            "golden token `{token}` missing in argv={argv:?}"
        );
    }
    let pairs: Vec<(&str, &str)> = argv
        .windows(2)
        .map(|w| (w[0].as_str(), w[1].as_str()))
        .collect();
    for (k, v) in &golden.expected_argv_pairs {
        let want = (k.as_str(), v.as_str());
        assert!(
            pairs.contains(&want),
            "golden pair {want:?} missing in argv={argv:?}"
        );
    }
    assert!(
        bridge.spec.topics.len() >= golden.expected_topics_count,
        "expected at least {} topics, got {}",
        golden.expected_topics_count,
        bridge.spec.topics.len()
    );
}
