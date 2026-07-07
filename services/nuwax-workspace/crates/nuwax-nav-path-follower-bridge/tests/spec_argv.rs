//! Path Follower bridge argv 渲染 + golden fixture 集成测试
//!
//! 使用 `nuwax-nav-test-fixtures::mock_native_path()` 共享 helper 定位 mock-native。
//! 包含 `path_follower_golden_argv_matches_fixture` synthetic golden fixture 测试。
//!
//! **lifecycle / failed_exit 集成测试已迁移到**:
//! `crates/nuwax-nav-test-fixtures/tests/bridge_lifecycle.rs::path_follower`
//! —— 那处由 cargo 自动注入 `CARGO_BIN_EXE_mock-native`,在任意
//! `CARGO_TARGET_DIR`(包括全新临时目录)下都不需要预构建。

use std::collections::BTreeMap;

use nuwax_nav_path_follower_bridge::{
    cli_exclude, PathFollowerBridge, PathFollowerBridgeConfig, PathFollowerTopics,
};
use nuwax_nav_test_fixtures::goldens::load as load_golden;
use nuwax_nav_test_fixtures::mock_native_path;

fn mock_native_bin() -> std::path::PathBuf {
    mock_native_path()
}

fn mock_config(extra_env: BTreeMap<String, String>) -> PathFollowerBridgeConfig {
    PathFollowerBridgeConfig {
        executable: mock_native_bin(),
        env: extra_env,
        restart_policy: nuwax_nav_path_follower_bridge::PathFollowerRestartPolicy::Never,
        ..PathFollowerBridgeConfig::default()
    }
}

#[test]
fn path_follower_bridge_argv_matches_expected() {
    let cfg = mock_config(BTreeMap::new());
    let topics = PathFollowerTopics {
        path: Some("/path".to_string()),
        odometry: Some("/odom".to_string()),
        speed: Some("/speed".to_string()),
        slow_down: Some("/slow_down".to_string()),
        safety_stop: Some("/safety_stop".to_string()),
        cmd_vel: Some("/cmd_vel".to_string()),
    };
    let bridge = PathFollowerBridge::new(cfg, &topics);
    let argv = bridge.argv();

    assert!(
        argv[0].contains("mock-native") || argv[0].contains("mock_native"),
        "argv[0] got: {}",
        argv[0]
    );

    // 6 个 topic 都在 argv 中
    let argv_joined = argv.join("\n");
    for t in [
        "/path",
        "/odom",
        "/speed",
        "/slow_down",
        "/safety_stop",
        "/cmd_vel",
    ] {
        assert!(
            argv_joined.contains(t),
            "topic {t} missing in argv={argv:?}"
        );
    }

    // camelCase 历史名
    let pairs: Vec<(&str, &str)> = argv
        .windows(2)
        .map(|w| (w[0].as_str(), w[1].as_str()))
        .collect();
    assert!(
        pairs
            .iter()
            .any(|(k, v)| *k == "--lookAheadDis" && *v == "0.5"),
        "--lookAheadDis 0.5 missing in argv={argv:?}"
    );
    assert!(
        pairs.iter().any(|(k, v)| *k == "--maxSpeed" && *v == "1"),
        "--maxSpeed 1 missing in argv={argv:?}"
    );
    assert!(
        pairs
            .iter()
            .any(|(k, v)| *k == "--maxYawRate" && *v == "45"),
        "--maxYawRate 45 missing in argv={argv:?}"
    );
    assert!(
        pairs
            .iter()
            .any(|(k, v)| *k == "--slowDwnDisThre" && *v == "1"),
        "--slowDwnDisThre missing in argv={argv:?}"
    );
    assert!(
        pairs
            .iter()
            .any(|(k, v)| *k == "--twoWayDrive" && *v == "true"),
        "--twoWayDrive missing in argv={argv:?}"
    );
    // autonomy_mode(默认 None)不渲染
    assert!(
        !argv.iter().any(|a| a == "--autonomyMode"),
        "autonomyMode(默认 None)应被排除"
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

/// Synthetic golden fixture 测试 — 加载 `goldens/path_follower_smoke.json`,
/// 构造 bridge,断言 argv 包含期望 token + pair + 至少 N 个 topic。
#[test]
fn path_follower_golden_argv_matches_fixture() {
    let golden = load_golden("path_follower_smoke");
    assert_eq!(golden.bridge, "path_follower");

    let cfg = mock_config(BTreeMap::new());
    let mut topics = PathFollowerTopics::default();
    for (port, topic) in &golden.topics {
        match port.as_str() {
            "path" => topics.path = Some(topic.clone()),
            "odometry" => topics.odometry = Some(topic.clone()),
            "speed" => topics.speed = Some(topic.clone()),
            "slow_down" => topics.slow_down = Some(topic.clone()),
            "safety_stop" => topics.safety_stop = Some(topic.clone()),
            "cmd_vel" => topics.cmd_vel = Some(topic.clone()),
            _ => {}
        }
    }
    let bridge = PathFollowerBridge::new(cfg, &topics);
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
