//! PGO bridge argv 渲染 + golden fixture + tf_helper 集成测试
//!
//! 设计:
//! - argv 渲染 + golden fixture 测试使用 `mock_native_path()` 解析 argv[0]
//!   占位路径(不启动 binary);真实生命周期断言已迁移到
//!   `crates/nuwax-nav-test-fixtures/tests/bridge_lifecycle.rs::pgo`。
//! - `pgo_tf_subscription_updates_tf_tree` 走 in-process 路径:
//!   直接调用 `tf_helper::apply_pgo_tf_to_tree` 把 `PgoTfMessage` 写入 `nuwax_tf::TfTree`,
//!   然后断言 `tree.lookup("odom")` 的 stamp + translation 正确。
//!   这样测试不依赖 LCM/ROS2 runtime(socket IPC) — 严格符合审查约束。
//! - `pgo_bridge_handle_pgo_tf_updates_tree` 直接测 `PgoBridge::handle_pgo_tf`
//!   这个 bridge subscription handler,验证 latest_pgo_tf 字段被填充 + TF tree 更新。

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

use nuwax_nav_pgo_bridge::{
    apply_pgo_tf_to_tree, cli_exclude, PgoBridge, PgoBridgeConfig, PgoTfMessage, PgoTopics,
};
use nuwax_nav_test_fixtures::goldens::load as load_golden;
use nuwax_nav_test_fixtures::mock_native_path;
use nuwax_tf::TfTree;

fn mock_native_bin() -> std::path::PathBuf {
    mock_native_path()
}

fn mock_config(extra_env: BTreeMap<String, String>) -> PgoBridgeConfig {
    PgoBridgeConfig {
        executable: mock_native_bin(),
        env: extra_env,
        restart_policy: nuwax_nav_pgo_bridge::PgoRestartPolicy::Never,
        // PGO 默认 cfg.cwd = Some("cpp")(相对路径,运行时无法解析;
        // mock-native 不是 C++ binary,不需要固定 cwd)
        cwd: None,
        ..PgoBridgeConfig::default()
    }
}

#[test]
fn pgo_bridge_argv_matches_expected() {
    let cfg = mock_config(BTreeMap::new());
    let topics = PgoTopics {
        registered_scan: Some("/scan".to_string()),
        odometry: Some("/odom".to_string()),
        corrected_odometry: Some("/corrected_odom".to_string()),
        global_map: Some("/global_map".to_string()),
        pgo_tf: Some("/pgo_tf".to_string()),
    };
    let bridge = PgoBridge::new(cfg, &topics);
    let argv = bridge.argv();

    assert!(
        argv[0].contains("mock-native") || argv[0].contains("mock_native"),
        "argv[0] got: {}",
        argv[0]
    );

    // 5 个 topic
    let argv_joined = argv.join("\n");
    for t in [
        "/scan",
        "/odom",
        "/corrected_odom",
        "/global_map",
        "/pgo_tf",
    ] {
        assert!(
            argv_joined.contains(t),
            "topic {t} missing in argv={argv:?}"
        );
    }

    // config: --world_frame map, --local_frame odom, --loop_submap_half_range 5
    let pairs: Vec<(&str, &str)> = argv
        .windows(2)
        .map(|w| (w[0].as_str(), w[1].as_str()))
        .collect();
    assert!(
        pairs
            .iter()
            .any(|(k, v)| *k == "--world_frame" && *v == "map"),
        "--world_frame map missing in argv={argv:?}"
    );
    assert!(
        pairs
            .iter()
            .any(|(k, v)| *k == "--local_frame" && *v == "odom"),
        "--local_frame odom missing in argv={argv:?}"
    );
    assert!(
        pairs
            .iter()
            .any(|(k, v)| *k == "--loop_submap_half_range" && *v == "5"),
        "--loop_submap_half_range 5 missing in argv={argv:?}"
    );
    assert!(
        pairs
            .iter()
            .any(|(k, v)| *k == "--global_map_voxel_size" && *v == "0.1"),
        "--global_map_voxel_size 0.1 missing in argv={argv:?}"
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

/// In-process mock transport 验证 pgo_tf 订阅会更新 `nuwax_tf::TfTree`
///
/// 设计要点:
/// - **不**真实订阅 LCM/ROS2 socket
/// - 用 `Arc<Mutex<Vec<PgoTfMessage>>>` 当作 mock transport 的 in-process 缓冲
/// - 模拟订阅循环:从缓冲取出消息,调用 `apply_pgo_tf_to_tree` 写入 `TfTree`
/// - 断言 `tree.lookup("odom")` 在多帧更新后保留最新 stamp + 平移
#[test]
fn pgo_tf_subscription_updates_tf_tree() {
    let tree = TfTree::new();

    // 共享 in-process transport buffer(模拟 LCM subscription 进队)
    let transport_buf: Arc<Mutex<Vec<PgoTfMessage>>> = Arc::new(Mutex::new(Vec::new()));
    {
        let mut buf = transport_buf.lock().unwrap();
        // seed identity frame(map -> odom, stamp 0)
        buf.push(PgoTfMessage {
            stamp_secs: 0,
            translation: [0.0, 0.0, 0.0],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        });
        // PGO 修正(stamp 1)
        buf.push(PgoTfMessage {
            stamp_secs: 1,
            translation: [0.5, 0.1, 0.0],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        });
        // 第二次 PGO 修正(stamp 2)
        buf.push(PgoTfMessage {
            stamp_secs: 2,
            translation: [0.7, -0.2, 0.0],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        });
    }

    // 模拟订阅循环:消费 buffer 中的消息,写入 TF tree
    {
        let buf = transport_buf.lock().unwrap();
        for msg in buf.iter() {
            apply_pgo_tf_to_tree(&tree, msg, "map", "odom");
        }
    }

    // 断言:tree.lookup("odom") 应保留最后一条 stamp=2 的修正
    let final_tf = tree.lookup("odom").expect("odom tf should exist");
    assert_eq!(final_tf.parent, "map");
    assert_eq!(final_tf.child, "odom");
    assert_eq!(final_tf.stamp_secs, 2);
    let dx = final_tf.transform.translation.x;
    let dy = final_tf.transform.translation.y;
    let dz = final_tf.transform.translation.z;
    assert!((dx - 0.7).abs() < 1e-9, "expected dx=0.7, got {dx}");
    assert!((dy - (-0.2)).abs() < 1e-9, "expected dy=-0.2, got {dy}");
    assert!((dz - 0.0).abs() < 1e-9, "expected dz=0.0, got {dz}");

    // 旋转为单位四元数(w≈1)
    let qw = final_tf.transform.rotation.w;
    assert!((qw - 1.0).abs() < 1e-9, "expected qw=1.0, got {qw}");
}

#[test]
fn pgo_tf_in_process_mock_transport_multiple_subscribers() {
    // 多个 TF tree 订阅同一 transport(类比多 consumer 订阅同一 topic)
    let tree_a = TfTree::new();
    let tree_b = TfTree::new();

    let transport: Arc<Mutex<Vec<PgoTfMessage>>> = Arc::new(Mutex::new(vec![PgoTfMessage {
        stamp_secs: 42,
        translation: [1.0, 2.0, 3.0],
        orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
    }]));

    let consume = |tree: &TfTree, transport: &Arc<Mutex<Vec<PgoTfMessage>>>| {
        let buf = transport.lock().unwrap();
        for msg in buf.iter() {
            apply_pgo_tf_to_tree(tree, msg, "map", "base_link");
        }
    };
    consume(&tree_a, &transport);
    consume(&tree_b, &transport);

    let a = tree_a.lookup("base_link").unwrap();
    let b = tree_b.lookup("base_link").unwrap();
    assert_eq!(a.stamp_secs, 42);
    assert_eq!(b.stamp_secs, 42);
    assert!((a.transform.translation.x - 1.0).abs() < 1e-9);
    assert!((b.transform.translation.x - 1.0).abs() < 1e-9);
}

/// 直接测 `PgoBridge::handle_pgo_tf` bridge subscription handler:
///
/// - 调用 `bridge.handle_pgo_tf(msg)` 后:
///   1. `bridge.latest_pgo_tf` 字段被填充
///   2. `bridge.tf_tree.lookup("odom")` 包含 msg 中的 stamp + 平移
/// - 这条测试是复审 MEDIUM-2 "测 bridge subscription handler" 的对应实现。
#[test]
fn pgo_bridge_handle_pgo_tf_updates_tree() {
    let cfg = mock_config(BTreeMap::new());
    let topics = PgoTopics::default();
    let mut bridge = PgoBridge::new(cfg, &topics);

    // 初始 latest_pgo_tf 应为 None
    assert!(
        bridge.latest_pgo_tf.is_none(),
        "latest_pgo_tf should start as None"
    );

    // 第一次 handle: identity seed
    let seed = PgoTfMessage {
        stamp_secs: 0,
        translation: [0.0, 0.0, 0.0],
        orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
    };
    bridge.handle_pgo_tf(seed.clone());
    assert_eq!(bridge.latest_pgo_tf.as_ref(), Some(&seed));
    let tf_seed = bridge
        .tf_tree
        .lookup("odom")
        .expect("odom tf should be set after handle_pgo_tf");
    assert_eq!(tf_seed.stamp_secs, 0);
    assert!((tf_seed.transform.translation.x - 0.0).abs() < 1e-9);

    // 第二次 handle: PGO 修正
    let correction = PgoTfMessage {
        stamp_secs: 100,
        translation: [1.5, -0.25, 0.0],
        orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
    };
    bridge.handle_pgo_tf(correction.clone());
    assert_eq!(bridge.latest_pgo_tf.as_ref(), Some(&correction));
    let tf_after = bridge
        .tf_tree
        .lookup("odom")
        .expect("odom tf should remain set");
    assert_eq!(tf_after.stamp_secs, 100);
    assert!((tf_after.transform.translation.x - 1.5).abs() < 1e-9);
    assert!((tf_after.transform.translation.y - (-0.25)).abs() < 1e-9);
    assert_eq!(tf_after.parent, "map");
    assert_eq!(tf_after.child, "odom");
}

/// Synthetic golden fixture 测试 — 加载 `goldens/pgo_smoke.json`,
/// 构造 bridge,断言 argv 包含期望 token + pair + 至少 N 个 topic。
// 后续按 fixture key 动态 patch cfg 字段,无法用 struct-update
#[allow(clippy::field_reassign_with_default)]
#[test]
fn pgo_golden_argv_matches_fixture() {
    let golden = load_golden("pgo_smoke");
    assert_eq!(golden.bridge, "pgo");

    // 本测试通过 `golden.config_overrides` 动态按 key 修改 cfg 字段,
    // 不适合用 struct-update 初始化(无法静态列出所有可写字段)
    let mut cfg = PgoBridgeConfig {
        executable: mock_native_bin(),
        restart_policy: nuwax_nav_pgo_bridge::PgoRestartPolicy::Never,
        cwd: None,
        ..PgoBridgeConfig::default()
    };
    let _ = &mut cfg; // 抑制 mut 警告

    for (k, v) in &golden.config_overrides {
        match (k.as_str(), v) {
            ("world_frame", serde_json::Value::String(s)) => cfg.world_frame = s.clone(),
            ("local_frame", serde_json::Value::String(s)) => cfg.local_frame = s.clone(),
            ("loop_submap_half_range", serde_json::Value::Number(n)) => {
                if let Some(i) = n.as_i64() {
                    cfg.loop_submap_half_range = i;
                }
            }
            ("global_map_voxel_size", serde_json::Value::Number(n)) => {
                if let Some(f) = n.as_f64() {
                    cfg.global_map_voxel_size = f;
                }
            }
            _ => {}
        }
    }

    let mut topics = PgoTopics::default();
    for (port, topic) in &golden.topics {
        match port.as_str() {
            "registered_scan" => topics.registered_scan = Some(topic.clone()),
            "odometry" => topics.odometry = Some(topic.clone()),
            "corrected_odometry" => topics.corrected_odometry = Some(topic.clone()),
            "global_map" => topics.global_map = Some(topic.clone()),
            "pgo_tf" => topics.pgo_tf = Some(topic.clone()),
            _ => {}
        }
    }
    let bridge = PgoBridge::new(cfg, &topics);
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
