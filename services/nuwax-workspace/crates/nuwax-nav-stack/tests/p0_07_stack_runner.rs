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

//! P0-07 集成测试:`StackRunner` 作为 nav-stack process owner 的核心契约。
//!
//! 验证:
//! 1. 空 graph -> 0 个 running node;
//! 2. 单 `NativeBridge` 节点 (binary=`/bin/echo`) -> 真的 spawn 成功;
//! 3. `native_bridge_ids()` 正确列出 graph 内的 native bridge 节点;
//! 4. `shutdown()` 在 5s grace + kill 之后不卡死;
//! 5. `build_runner()`(从 `NavStackBuilder` 走 `build_graph` 路径)在 default
//!    features 下不 panic、不依赖 liblcm。

use std::time::Duration;

use nuwax_nav_stack::{
    graph::{ModuleGraph, ModuleKind, ModuleNode},
    profiles::topic_map_for_node,
    NavStackBuilder, NavStackProfile, NavTopicConfig, StackRunner,
};

#[tokio::test]
async fn p0_07_stack_runner_from_empty_graph_spawns_zero() {
    let graph = ModuleGraph::new("simple-2d");
    let mut runner = StackRunner::from_graph(graph);
    runner.spawn_all().await.expect("spawn_all on empty graph");
    assert_eq!(runner.running_nodes().len(), 0);
    assert!(runner.native_bridge_ids().is_empty());
}

#[tokio::test]
async fn p0_07_stack_runner_spawns_native_bridge_using_echo() {
    // 构造一个 native bridge 节点,program=/bin/echo,args=["hello"]
    // echo 是 non-blocking 进程,但 spawn 后能立刻拿到 child 句柄;
    // 测试目的:验证 Command::new(args).spawn() 真的启动,而不是 stub。
    let node = ModuleNode::new("echo_test", "Echo Test", ModuleKind::NativeBridge)
        .with_binary("/bin/echo");
    let mut graph = ModuleGraph::new("p0-07-echo");
    graph.push_node(node);
    let mut runner = StackRunner::from_graph(graph);
    runner.spawn_all().await.expect("spawn_all");
    assert_eq!(runner.running_nodes().len(), 1);
    assert_eq!(runner.running_nodes(), vec!["echo_test".to_string()]);
    assert_eq!(runner.native_bridge_ids(), vec!["echo_test".to_string()]);
    // shutdown 用 0 grace 也必须返回(grace < 1s 也能跑完)
    runner
        .shutdown_with(Duration::from_millis(0))
        .await
        .expect("shutdown");
    // 二次 shutdown 不应 panic
    runner.shutdown().await.expect("second shutdown noop");
}

#[tokio::test]
async fn p0_07_stack_runner_skips_rust_algorithm_nodes() {
    // RustAlgorithm 不该被 spawn(它由 in-process tokio task / BridgeSupervisor 持有),
    // StackRunner 必须只在 NativeBridge 上 fork execve。
    let rust_node = ModuleNode::new("rust_only", "Rust", ModuleKind::RustAlgorithm)
        .with_binary("result/bin/some_rust");
    let native_node =
        ModuleNode::new("native_one", "Native", ModuleKind::NativeBridge).with_binary("/bin/true");
    let mut graph = ModuleGraph::new("mixed");
    graph.push_node(rust_node);
    graph.push_node(native_node);
    let mut runner = StackRunner::from_graph(graph);
    runner.spawn_all().await.expect("spawn_all");
    // 只 native bridge 进入 handles
    assert_eq!(runner.running_nodes().len(), 1);
    assert_eq!(runner.running_nodes(), vec!["native_one".to_string()]);
    // native_bridge_ids 只列 native bridge(graph 中两个 native_bridge_ids 不计 RustAlgorithm)
    assert_eq!(
        runner.native_bridge_ids(),
        vec!["native_one".to_string()],
        "rust_algorithm node must not appear in native_bridge_ids"
    );
    runner.shutdown().await.expect("shutdown");
}

#[tokio::test]
async fn p0_07_stack_runner_spawn_failure_returns_node_id() {
    // 不存在的 binary -> spawn 失败,SpawnFailed 必须携带 node id。
    let bad_node =
        ModuleNode::new("missing_binary", "Bad", ModuleKind::NativeBridge).with_binary("/nope");
    let mut graph = ModuleGraph::new("bad");
    graph.push_node(bad_node);
    let mut runner = StackRunner::from_graph(graph);
    let err = runner.spawn_all().await.expect_err("spawn must fail");
    let msg = err.to_string();
    assert!(msg.contains("missing_binary"), "error must name node id: {msg}");
    assert!(
        msg.contains("failed to spawn native bridge"),
        "error must be SpawnFailed: {msg}"
    );
}

#[tokio::test]
async fn p0_07_build_runner_from_builder_does_not_panic() {
    // default features 下 nav-stack 不依赖 liblcm / native-runner;
    // build_runner 走 build_graph() -> StackRunner 包装路径,
    // 即便没有任何 native bridge spawn 出来(因为 binary 是字符串),
    // 调用链必须不 panic。
    let mut runner = NavStackBuilder::new(NavStackProfile::Simple2D)
        .build_runner()
        .expect("build_runner on simple2d");
    // spawn_all 在 legacy 路径下若图里没有 native bridge 会立即返回;
    // Simple2D 不含任何 native bridge,所以 handles 应为空。
    runner.spawn_all().await.expect("spawn_all");
    assert!(runner.running_nodes().is_empty());
}

#[tokio::test]
async fn p0_07_graph_holds_native_bridge_ids_for_simple2d_profile() {
    // Simple2D 是纯 Rust pipeline,无 NativeBridge;
    // 列出 graph 节点 + native_bridge_ids 必须保持一致。
    let graph = NavStackBuilder::new(NavStackProfile::Simple2D)
        .build()
        .expect("build");
    let runner = StackRunner::from_graph(graph);
    assert!(runner.native_bridge_ids().is_empty());
}

#[tokio::test]
async fn p0_07_topic_map_helper_lists_far_planner_ports() {
    // P0-08 helper 同步验证:topic_map_for_node 必须返回非空 for far_planner
    let cfg = NavTopicConfig::default();
    let ports = topic_map_for_node("far_planner", &cfg);
    assert!(
        !ports.is_empty(),
        "far_planner 必须至少有 1 个 port 被解析,P0-08 硬契约"
    );
    let names: Vec<&str> = ports.iter().map(|(p, _)| p.as_str()).collect();
    // 关键 topic 必须存在
    for required in ["odometry", "terrain_map_ext", "way_point", "goal_path"] {
        assert!(
            names.contains(&required),
            "far_planner port `{required}` 必须被 P0-08 helper 列出,got: {names:?}"
        );
    }
}

#[tokio::test]
async fn p0_07_topic_map_helper_returns_empty_for_unknown_node() {
    // 未知 node_id 返回空 Vec,而非 panic
    let cfg = NavTopicConfig::default();
    assert!(topic_map_for_node("nonexistent_node", &cfg).is_empty());
}