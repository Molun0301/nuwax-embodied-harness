//! FAR Planner bridge argv 渲染 + golden fixture 集成测试
//!
//! 使用 `nuwax-nav-test-fixtures` 提供的 `mock-native` binary 作为 argv[0] 占位
//! 路径(纯字符串渲染,不启动 binary):
//! - `mock-native` 会将全部 argv 逐行打印到 stdout,并打 `mock-native-ok` 标记
//! - executable 路径由 `nuwax_nav_test_fixtures::mock_native_path()` 统一解析
//!   (优先 `CARGO_BIN_EXE_mock-native`,否则向上找 workspace `target/debug/mock-native`,
//!   最后回退到相对路径 `"mock-native"` 占位)
//! - 包含 synthetic golden fixture 测试(`far_planner_golden_argv_matches_fixture`),
//!   不依赖真实 rosbag,确定性可复现
//!
//! **lifecycle / failed_exit 集成测试已迁移到**:
//! `crates/nuwax-nav-test-fixtures/tests/bridge_lifecycle.rs::far_planner`
//! —— 那处由 cargo 自动注入 `CARGO_BIN_EXE_mock-native`,在任意
//! `CARGO_TARGET_DIR`(包括全新临时目录)下都不需要预构建。

use std::collections::BTreeMap;

use nuwax_nav_far_planner_bridge::{
    cli_exclude, cli_name_override, FarPlannerBridge, FarPlannerBridgeConfig, FarPlannerTopics,
};
use nuwax_nav_test_fixtures::fixtures::nav_diagnostics;
use nuwax_nav_test_fixtures::fixtures::pose;
use nuwax_nav_test_fixtures::goldens::load as load_golden;
use nuwax_nav_test_fixtures::mock_native_path;

/// 获取 mock-native binary 路径(共享 helper,见 `nuwax-nav-test-fixtures::mock_native_path`)。
fn mock_native_bin() -> std::path::PathBuf {
    mock_native_path()
}

/// 构造一个 FAR Planner config,executable 指向 mock-native
fn mock_config(extra_env: BTreeMap<String, String>) -> FarPlannerBridgeConfig {
    FarPlannerBridgeConfig {
        executable: mock_native_bin(),
        env: extra_env,
        // disable restart forever — test 控制生命周期
        restart_policy: nuwax_nav_far_planner_bridge::FarPlannerRestartPolicy::Never,
        ..FarPlannerBridgeConfig::default()
    }
}

/// 在测试配置上下文中断言 mock-native argv 渲染包含 config + topic 字段
fn assert_argv_renders_executable_and_config(argv: &[String]) {
    assert!(!argv.is_empty(), "argv should not be empty");
    assert!(
        argv[0].contains("mock-native") || argv[0].contains("mock_native"),
        "argv[0] should be the mock binary, got: {}",
        argv[0]
    );

    // topic args 必须出现(BTreeMap 排序:add_obstacle 等)
    // FAR Planner 模板:选定 6 个 in/out topic,逐一断言
    let expected_topic_pairs: &[(&str, &str)] = &[
        ("contour_polygons", "/contour"),
        ("goal", "/goal"),
        ("goal_path", "/goal_path"),
        ("graph_edges", "/graph_edges"),
        ("graph_nodes", "/graph_nodes"),
        ("nav_boundary", "/nav_boundary"),
        ("odometry", "/odom"),
        ("registered_scan", "/scan"),
        ("stop_movement", "/stop"),
        ("terrain_map", "/terrain"),
        ("terrain_map_ext", "/terrain_ext"),
        ("way_point", "/way_point"),
    ];
    let argv_string = argv.join("\n");
    for (port, topic) in expected_topic_pairs {
        assert!(
            argv_string.contains(&format!("--{port}")),
            "argv should contain --{port}, argv={argv_string:?}"
        );
        assert!(
            argv_string.contains(topic),
            "argv should contain topic `{topic}`, argv={argv_string:?}"
        );
    }

    // config args 应包含历史别名 --robot_dim(C++ 拼写)
    let argv_str_pairs: Vec<(&str, &str)> = argv
        .windows(2)
        .map(|w| (w[0].as_str(), w[1].as_str()))
        .collect();
    assert!(
        argv_str_pairs
            .iter()
            .any(|(k, v)| *k == "--robot_dim" && *v == "0.5"),
        "robot_dim override missing in argv={argv:?}"
    );
    // update_rate 默认 5.0
    assert!(
        argv_str_pairs
            .iter()
            .any(|(k, v)| *k == "--update_rate" && *v == "5"),
        "update_rate missing in argv={argv:?}"
    );
    // world_frame = "map"
    assert!(
        argv_str_pairs
            .iter()
            .any(|(k, v)| *k == "--world_frame" && *v == "map"),
        "world_frame missing in argv={argv:?}"
    );

    // 排除字段不应进入 argv
    for excluded in cli_exclude() {
        let prefix = format!("--{excluded}");
        assert!(
            !argv.iter().any(|a| a.starts_with(&prefix)),
            "excluded field {prefix} leaked into argv={argv:?}"
        );
    }

    // name_override: robot_dimension → robot_dim 必须存在(已断言)
    let m = cli_name_override();
    assert_eq!(
        m.get("robot_dimension").map(String::as_str),
        Some("robot_dim")
    );
}

#[test]
fn far_planner_bridge_argv_matches_expected() {
    let cfg = mock_config(BTreeMap::new());
    let topics = FarPlannerTopics {
        contour_polygons: Some("/contour".to_string()),
        goal: Some("/goal".to_string()),
        goal_path: Some("/goal_path".to_string()),
        graph_edges: Some("/graph_edges".to_string()),
        graph_nodes: Some("/graph_nodes".to_string()),
        nav_boundary: Some("/nav_boundary".to_string()),
        odometry: Some("/odom".to_string()),
        registered_scan: Some("/scan".to_string()),
        stop_movement: Some("/stop".to_string()),
        terrain_map: Some("/terrain".to_string()),
        terrain_map_ext: Some("/terrain_ext".to_string()),
        way_point: Some("/way_point".to_string()),
    };
    let bridge = FarPlannerBridge::new(cfg, &topics);
    let argv = bridge.argv();
    assert_argv_renders_executable_and_config(&argv);
}

#[test]
fn fixtures_pose_and_diagnostics_smoke() {
    // 额外 sanity check:确认 fixture helpers 可用
    let _p = pose(1.0, 2.0, 0.5);
    let _d = nav_diagnostics();
}

/// Synthetic golden fixture 测试:
/// 1. 加载 `goldens/far_planner_smoke.json` 描述的 topic + config 期望
/// 2. 构造 bridge,渲染 argv
/// 3. 断言 argv 包含 fixture 期望的所有 token + pair + 至少 N 个 topic
///
/// 该测试是 P1 bridge integration "至少 1 条 rosbag/golden smoke" 的等价物:
/// 确定性 JSON 期望,不依赖真实 C++ binary 或 rosbag。
// 后续按 fixture key 动态 patch cfg 字段,无法用 struct-update 静态列出
#[allow(clippy::field_reassign_with_default)]
#[test]
fn far_planner_golden_argv_matches_fixture() {
    let golden = load_golden("far_planner_smoke");
    assert_eq!(golden.bridge, "far_planner", "fixture bridge mismatch");

    // 本测试通过 `golden.config_overrides` 动态按 key 修改 cfg 字段,
    // 不适合用 struct-update 初始化(无法静态列出所有可写字段)
    #[allow(clippy::field_reassign_with_default)]
    let mut cfg = FarPlannerBridgeConfig::default();
    cfg.executable = mock_native_bin();
    cfg.restart_policy = nuwax_nav_far_planner_bridge::FarPlannerRestartPolicy::Never;

    // 应用 fixture config_overrides
    for (k, v) in &golden.config_overrides {
        match (k.as_str(), v) {
            ("robot_dimension", serde_json::Value::Number(n)) => {
                if let Some(f) = n.as_f64() {
                    cfg.robot_dimension = f;
                }
            }
            ("update_rate", serde_json::Value::Number(n)) => {
                if let Some(f) = n.as_f64() {
                    cfg.update_rate = f;
                }
            }
            ("world_frame", serde_json::Value::String(s)) => {
                cfg.world_frame = s.clone();
            }
            _ => {}
        }
    }

    // 构造 topics from fixture
    let mut topics = FarPlannerTopics::default();
    for (port, topic) in &golden.topics {
        match port.as_str() {
            "contour_polygons" => topics.contour_polygons = Some(topic.clone()),
            "goal" => topics.goal = Some(topic.clone()),
            "goal_path" => topics.goal_path = Some(topic.clone()),
            "graph_edges" => topics.graph_edges = Some(topic.clone()),
            "graph_nodes" => topics.graph_nodes = Some(topic.clone()),
            "nav_boundary" => topics.nav_boundary = Some(topic.clone()),
            "odometry" => topics.odometry = Some(topic.clone()),
            "registered_scan" => topics.registered_scan = Some(topic.clone()),
            "stop_movement" => topics.stop_movement = Some(topic.clone()),
            "terrain_map" => topics.terrain_map = Some(topic.clone()),
            "terrain_map_ext" => topics.terrain_map_ext = Some(topic.clone()),
            "way_point" => topics.way_point = Some(topic.clone()),
            _ => {}
        }
    }

    let bridge = FarPlannerBridge::new(cfg, &topics);
    let argv = bridge.argv();

    // 期望 token 出现
    let argv_joined = argv.join("\n");
    for token in &golden.expected_argv_contains {
        assert!(
            argv_joined.contains(token),
            "golden token `{token}` missing in argv={argv:?}"
        );
    }
    // 期望 pair 出现(--k v 相邻)
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
    // 至少 N 个 topic
    assert!(
        bridge.spec.topics.len() >= golden.expected_topics_count,
        "expected at least {} topics, got {}",
        golden.expected_topics_count,
        bridge.spec.topics.len()
    );
}
