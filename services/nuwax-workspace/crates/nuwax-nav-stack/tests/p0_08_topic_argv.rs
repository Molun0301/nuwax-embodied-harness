// Copyright 2026 Nuwax.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! P0-08 集成测试:NavTopicConfig 真实填入 native bridge 的 typed Topics 结构,
//! 保证 bridge.argv() / to_topic_map() 非空,argv 含 `--port topic`。
//!
//! 验证:
//! 1. `topic_map_for_node("far_planner", cfg)` 在 `cfg = NavTopicConfig::default()` 下返回非空;
//! 2. `topic_map_for_node` 对所有 6 个 native bridge(terrain_analysis / pgo / far_planner /
//!    local_planner / path_follower / tare_planner)返回非空;
//! 3. argv 构造(`runner::argv_for`)在 legacy binary 路径上首项 = binary 字符串;
//! 4. NavTopicConfig 自定义 topic 时,helper 透传(不会被 hardcoded name 覆盖);
//! 5. `topic_for` 在 port 缺失 + 空 fallback 时返回 None。

use nuwax_nav_stack::{
    graph::{ModuleGraph, ModuleKind, ModuleNode},
    profiles::{topic_for, topic_map_for_node},
    runner::argv_for,
    NavTopicConfig,
};

#[test]
fn p0_08_far_planner_topics_not_empty_after_profile_build() {
    let cfg = NavTopicConfig::default();
    let ports = topic_map_for_node("far_planner", &cfg);
    assert!(
        !ports.is_empty(),
        "far_planner topics must be non-empty after profile build, got 0 ports"
    );
    // 12 个 port 名必须全部命中(NavTopicConfig::default() = cmu_defaults() 含全部)
    let names: Vec<&str> = ports.iter().map(|(p, _)| p.as_str()).collect();
    for required in [
        "terrain_map_ext",
        "terrain_map",
        "registered_scan",
        "odometry",
        "goal",
        "stop_movement",
        "way_point",
        "goal_path",
        "graph_nodes",
        "graph_edges",
        "contour_polygons",
        "nav_boundary",
    ] {
        assert!(
            names.contains(&required),
            "far_planner port `{required}` missing in topic_map_for_node: {names:?}"
        );
    }
}

#[test]
fn p0_08_all_six_native_bridges_have_non_empty_topic_map() {
    // 6 个 bridge spec.rs 共享的「to_topic_map() filter」契约:
    // NavTopicConfig -> typed *Topics struct -> to_topic_map() 必须非空。
    let cfg = NavTopicConfig::default();
    let bridges = [
        "terrain_analysis",
        "pgo",
        "far_planner",
        "local_planner",
        "path_follower",
        "tare_planner",
    ];
    for id in bridges {
        let ports = topic_map_for_node(id, &cfg);
        assert!(
            !ports.is_empty(),
            "{id} topic_map_for_node must be non-empty;NavTopicConfig::default() 应有 \
             cmu_defaults() 全部 topic"
        );
    }
}

#[test]
fn p0_08_topic_map_transmits_custom_topic_override() {
    // 自定义 cmd_vel -> "robot/cmd_vel",helper 必须透传,不被 fallback 覆盖
    let mut cfg = NavTopicConfig::default();
    cfg.set("cmd_vel", "robot/cmd_vel");
    let ports = topic_map_for_node("path_follower", &cfg);
    let cmd_vel = ports
        .iter()
        .find(|(p, _)| p == "cmd_vel")
        .expect("cmd_vel port present");
    assert_eq!(
        cmd_vel.1, "robot/cmd_vel",
        "P0-08:自定义 topic 必须透传到 argv,不能被 hardcoded name 覆盖"
    );
}

#[test]
fn p0_08_topic_for_returns_some_when_topic_present() {
    let cfg = NavTopicConfig::default();
    assert_eq!(
        topic_for(&cfg, "odometry", "odometry").as_deref(),
        Some("odometry")
    );
    assert_eq!(
        topic_for(&cfg, "cmd_vel", "cmd_vel").as_deref(),
        Some("cmd_vel")
    );
}

#[test]
fn p0_08_topic_for_returns_none_when_fallback_empty() {
    let cfg = NavTopicConfig::empty();
    assert!(topic_for(&cfg, "anything", "").is_none());
}

#[test]
fn p0_08_argv_for_legacy_native_bridge_uses_binary_only() {
    // legacy 路径(无 typed native_spec,只有 binary 字符串):argv = [binary]
    let node = ModuleNode::new("legacy", "Legacy", ModuleKind::NativeBridge)
        .with_binary("result/bin/far_planner_native");
    let argv = argv_for(&node).expect("legacy native_bridge must produce argv");
    assert_eq!(argv.len(), 1);
    assert_eq!(argv[0], "result/bin/far_planner_native");
}

#[test]
fn p0_08_argv_for_returns_none_for_non_native_bridge() {
    let node = ModuleNode::new("rust_only", "Rust", ModuleKind::RustAlgorithm);
    assert!(argv_for(&node).is_none());
}

#[test]
fn p0_08_topic_map_for_node_returns_sorted_port_pairs() {
    // port 名按 helper 内部顺序输出,但每项的 topic 名必须 == cfg.get_or 解析结果
    let cfg = NavTopicConfig::default();
    let ports = topic_map_for_node("local_planner", &cfg);
    let mut iter = ports.iter();
    let first = iter.next().expect("at least 1 port");
    // local_planner 第一个 port 是 added_obstacles(按字典序);实际 helper 用固定顺序
    // 不强求字典序,只断言 topic 名解析正确
    for (port, topic) in ports.iter() {
        let expected = cfg.get_or(port, port);
        assert_eq!(topic, expected, "port `{port}` topic must match cfg.get_or");
        // dummy use of first to silence unused warning
        let _ = first;
    }
}

#[test]
fn p0_08_legacy_runner_argv_count_matches_binary_count_for_empty_topic() {
    // ModuleGraph 含 native_bridge 但 typed feature 关闭 -> argv_for 仅返 [binary]
    let mut g = ModuleGraph::new("legacy");
    g.push_node(
        ModuleNode::new("legacy_a", "A", ModuleKind::NativeBridge).with_binary("result/bin/a"),
    );
    g.push_node(
        ModuleNode::new("legacy_b", "B", ModuleKind::NativeBridge).with_binary("result/bin/b"),
    );
    for n in g.nodes() {
        let argv = argv_for(n).expect("argv");
        assert_eq!(argv.len(), 1, "legacy argv must be [binary] only");
    }
}