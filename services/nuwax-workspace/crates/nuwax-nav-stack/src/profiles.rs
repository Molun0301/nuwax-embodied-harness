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

//! 每个 `NavStackProfile` 的 module graph 定义。
//!
//! 关键原则:
//! - **不引用上游 crate 的具体类型**:graph 是纯数据,字段都是字符串 / 配置对象。
//!   即便对应 Cargo feature 未启用,dry-run 仍可输出。
//! - 每个 profile 输出一份 module graph,代表方案 §0 / §3 中的 pipeline。
//! - bridge 节点标注 `feature_gate` 与 native binary 路径,不启动真实进程。
//!
//! P0-08 修订:6 个 native bridge 的 typed `*Topics` 字段不再使用 `::default()`,
//! 改为 `topic_map_for_node(...)` 从 `NavTopicConfig.topics` 复制真实 topic 名,
//! 使 `native_spec.topics` / `bridge.argv()` 含 `--port topic` 参数。

use crate::features::NavFeatureFlags;
use crate::graph::{GraphEdge, ModuleGraph, ModuleKind, ModuleNode, NavFeatureFlagsMirror};
use crate::profile::NavStackProfile;
use crate::topics::NavTopicConfig;

// 注:typed `ModuleNode::from_native_specs` 函数定义在 graph.rs,
// 该模块已经按 `feature = "native-runner"` 引入 `NativeProgramSpec` /
// `NativeBuildSpec`,本文件不需要再重复 import。

/// 根据 profile + topics + features 构造 `ModuleGraph`。
pub fn build_graph(
    profile: NavStackProfile,
    topics: &NavTopicConfig,
    features: NavFeatureFlags,
) -> ModuleGraph {
    let mut g = ModuleGraph::new(profile.as_str()).with_features(mirror_from_flags(&features));
    match profile {
        NavStackProfile::Simple2D => build_simple2d(&mut g, topics),
        NavStackProfile::CmuNative => build_cmu_native(&mut g, topics),
        NavStackProfile::ReplanningAStar => build_replanning_astar(&mut g, topics),
        NavStackProfile::ExplorationNative => build_exploration_native(&mut g, topics),
        NavStackProfile::Mls3D => build_mls3d(&mut g, topics),
    }
    g
}

fn mirror_from_flags(f: &NavFeatureFlags) -> NavFeatureFlagsMirror {
    let mut names: Vec<String> = f
        .iter_names()
        .map(|(n, _)| crate::features::flag_to_kebab(n).to_string())
        .collect();
    // 保持字典序以稳定 dry-run 输出
    names.sort();
    NavFeatureFlagsMirror::from_names(names)
}

// ---------- P0-08 NavTopicConfig -> typed *Topics helper ----------
//
// 旧实现直接 `*Topics::default()` (字段全 `Option::None`),导致
// `to_topic_map()` 返回空 map,native bridge argv 不含 `--port topic`。
// 新实现按节点 id 索引到 `NavTopicConfig.topics` 中对应的 port 名,
// 显式 `Some(...)` 后传入 typed spec builder,保证 port->topic 进入 argv。

/// 按 `node_id` 在 `NavTopicConfig.topics` 中查找一个 port 对应的 topic 字符串。
///
/// 找不到时回退到 `fallback`,便于 dry-run / 单测在 topics 缺失时仍能工作。
#[allow(dead_code)] // 公开的 helper,部分测试可调用
pub fn topic_for(topics: &NavTopicConfig, port: &str, fallback: &str) -> Option<String> {
    let name = topics.get_or(port, fallback);
    if name.is_empty() {
        None
    } else {
        Some(name.to_string())
    }
}

/// 把 `NavTopicConfig` 中与指定 `node_id` 相关的几个 port 抽成 `(port, topic)` 列表。
///
/// 该函数是 P0-08 测试钩子:`topic_map_for_node("far_planner", &cfg)` 必须返回非空。
#[must_use]
pub fn topic_map_for_node(node_id: &str, topics: &NavTopicConfig) -> Vec<(String, String)> {
    if node_id == "mls_planner" {
        return [
            ("global_map", "global_map", "global_map"),
            ("local_map", "local_map", "local_map"),
            ("region_bounds", "region_bounds", "region_bounds"),
            ("start_pose", "start_pose", "start_pose"),
            ("goal_pose", "goal_pose", "goal_pose"),
            ("path", "goal_path", "goal_path"),
            ("surface_map", "surface_map", "mls_surface"),
            ("nodes", "nodes", "mls_nodes"),
            ("node_edges", "node_edges", "mls_node_edges"),
        ]
        .into_iter()
        .map(|(port, key, fallback)| (port.to_string(), topics.get_or(key, fallback).to_string()))
        .collect();
    }
    let ports: &[&str] = match node_id {
        "terrain_analysis" => &["registered_scan", "odometry", "terrain_map"],
        "pgo" => &[
            "registered_scan",
            "odometry",
            "corrected_odometry",
            "global_map",
            "pgo_tf",
        ],
        "far_planner" => &[
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
        ],
        "local_planner" => &[
            "registered_scan",
            "odometry",
            "terrain_map",
            "joy_cmd",
            "way_point",
            "goal",
            "speed",
            "added_obstacles",
            "check_obstacle",
            "cancel_goal",
            "path",
            "effective_cmd_vel",
            "free_paths",
            "slow_down",
            "goal_reached",
        ],
        "path_follower" => &[
            "path",
            "odometry",
            "speed",
            "slow_down",
            "safety_stop",
            "cmd_vel",
        ],
        "tare_planner" => &["registered_scan", "odometry", "way_point"],
        _ => &[],
    };
    ports
        .iter()
        .filter_map(|p| {
            let name = match (node_id, *p) {
                ("path_follower", "cmd_vel") => topics.get_or("nav_cmd_vel", "/nav/cmd_vel"),
                ("far_planner" | "local_planner", "goal") => navigation_goal_topic(topics),
                (_, port) => topics.get_or(port, port),
            };
            if name.is_empty() {
                None
            } else {
                Some(((*p).to_string(), name.to_string()))
            }
        })
        .collect()
}

fn navigation_goal_topic(topics: &NavTopicConfig) -> &str {
    topics
        .topics
        .get("goal_pose")
        .or_else(|| topics.topics.get("goal"))
        .map_or("goal_pose", |topic| topic.as_str())
}

// ---------- Native spec wiring (Phase E6 / R6 补全) ----------
//
// Phase E6 设计目标:
// 1. 当 `native-runner` feature + 对应 bridge feature 同时启用时,
//    bridge 的 `build_specs` / `build_spec` 返回的 typed spec
//    必须进入 `ModuleNode.native_spec`,而不是只填 `binary` 字符串;
//    这样下游 BridgeSupervisor / dry-run / serde roundtrip 都能拿到
//    完整 args / topics / restart / build policy。
// 2. 当对应 bridge feature 关闭时,fallback 到字符串 `with_binary`,
//    `native_spec` 保持 `None`(cfg-gated 字段在 feature 关闭时也不存在)。
// 3. `build_specs` 返回 `Result<(spec, build), _>`,由 caller 端 `match` 错误
//    时降级为 legacy `with_binary(fallback)` 路径,保证 stack 不会因单个
//    bridge config 校验失败而 panic。
//
// 实现:在 `build_cmu_native` / `build_exploration_native` 中按 cfg 双轨内联
// 展开 typed `from_native_specs` 入口与 legacy 字符串路径,避免在 helper
// 签名中硬编码 `nuwax_nav_native_runner::...` 类型,这样在 native-runner
// feature 关闭时整个文件仍能编译。
//
// P0-08 修订:`*Topics::default()` 不再使用,改为 `topic_map_for_node(id, topics)`
// 从 `NavTopicConfig.topics` 取真实 topic 名,然后通过 `XxxTopics { field: Some(...) }`
// 构造 typed struct,确保 `to_topic_map()` 过滤后非空、`bridge.argv()` 含
// `--port topic`。

// ---------- Simple2D ----------

fn build_simple2d(g: &mut ModuleGraph, topics: &NavTopicConfig) {
    g.push_node(map_pose_adapter(topics));

    // Planner-consumed occupancy + real map pose -> versioned path. Public
    // clicks must enter through the Geometry goal-lease owner; this process may
    // not fabricate GoalEnvelope identity/generation fields.
    g.push_node(
        ModuleNode::new(
            "simple_planner",
            "Simple Planner",
            ModuleKind::RustAlgorithm,
        )
        .with_feature("simple-planner")
        .add_input(topics.get_or("occupancy", "/map/occupancy"))
        .add_input(topics.get_or("map_pose", "map_pose"))
        .add_input(topics.get_or("goal_pose", "goal_pose"))
        .add_input(topics.get_or("stop_movement", "stop_movement"))
        .add_output(topics.get_or("way_point", "way_point"))
        .add_output(topics.get_or("goal_path", "goal_path"))
        .depends_on_node("map_pose_adapter"),
    );

    g.push_node(guarded_nav_scheduler(topics, "simple_planner"));

    // edges
    g.push_edge(GraphEdge {
        from: "simple_planner".into(),
        to: "guarded_nav_scheduler".into(),
        topic: topics.get_or("goal_path", "goal_path").into(),
    });
    g.push_edge(GraphEdge {
        from: "map_pose_adapter".into(),
        to: "simple_planner".into(),
        topic: topics.get_or("map_pose", "map_pose").into(),
    });
    g.push_edge(GraphEdge {
        from: "map_pose_adapter".into(),
        to: "guarded_nav_scheduler".into(),
        topic: topics.get_or("map_pose", "map_pose").into(),
    });
    g.push_edge(GraphEdge {
        from: "guarded_nav_scheduler".into(),
        to: "simple_planner".into(),
        topic: topics.get_or("stop_movement", "stop_movement").into(),
    });
}

// ---------- ReplanningAStar ----------

fn build_replanning_astar(g: &mut ModuleGraph, topics: &NavTopicConfig) {
    g.push_node(map_pose_adapter(topics));

    g.push_node(
        ModuleNode::new(
            "astar_planner",
            "Replanning A* Planner",
            ModuleKind::RustAlgorithm,
        )
        .with_feature("astar")
        .add_input(topics.get_or("map_pose", "map_pose"))
        .add_input(topics.get_or("occupancy", "/map/occupancy"))
        .add_input(topics.get_or("goal_pose", "goal_pose"))
        .add_input(topics.get_or("stop_movement", "stop_movement"))
        .add_output(topics.get_or("way_point", "way_point"))
        .add_output(topics.get_or("goal_path", "goal_path"))
        .add_output(topics.get_or("goal_reached", "goal_reached"))
        .add_output(topics.get_or("goal_failed", "goal_failed"))
        .depends_on_node("map_pose_adapter"),
    );

    g.push_node(guarded_nav_scheduler(topics, "astar_planner"));

    g.push_edge(GraphEdge {
        from: "astar_planner".into(),
        to: "guarded_nav_scheduler".into(),
        topic: topics.get_or("goal_path", "goal_path").into(),
    });
    g.push_edge(GraphEdge {
        from: "map_pose_adapter".into(),
        to: "astar_planner".into(),
        topic: topics.get_or("map_pose", "map_pose").into(),
    });
    g.push_edge(GraphEdge {
        from: "map_pose_adapter".into(),
        to: "guarded_nav_scheduler".into(),
        topic: topics.get_or("map_pose", "map_pose").into(),
    });
    g.push_edge(GraphEdge {
        from: "guarded_nav_scheduler".into(),
        to: "astar_planner".into(),
        topic: topics.get_or("stop_movement", "stop_movement").into(),
    });
}

fn map_pose_adapter(topics: &NavTopicConfig) -> ModuleNode {
    ModuleNode::new(
        "map_pose_adapter",
        "Map Pose Adapter",
        ModuleKind::Coordinator,
    )
    .with_feature("lcm-runtime")
    .add_input(topics.get_or("odometry", "odometry"))
    .add_input(topics.get_or("tf", "/tf"))
    .add_output(topics.get_or("map_pose", "/nav/map_pose"))
}

fn guarded_nav_scheduler(topics: &NavTopicConfig, planner: &str) -> ModuleNode {
    ModuleNode::new(
        "guarded_nav_scheduler",
        "Guard-backed Navigation Scheduler",
        ModuleKind::Coordinator,
    )
    .with_feature("guard-runtime")
    .add_input(topics.get_or("goal_pose", "goal_pose"))
    .add_input(topics.get_or("map_pose", "/nav/map_pose"))
    .add_input(topics.get_or("goal_path", "goal_path"))
    .add_input(topics.get_or("occupancy", "/map/occupancy"))
    .add_input(topics.get_or("nav_cancel", "/nav/cancel"))
    .add_output(topics.get_or("cmd_vel", "cmd_vel"))
    .add_output(topics.get_or("nav_state", "/nav/state"))
    .add_output(topics.get_or("goal_reached", "goal_reached"))
    .add_output(topics.get_or("stop_movement", "stop_movement"))
    .depends_on_node(planner)
    .depends_on_node("map_pose_adapter")
}

// ---------- CmuNative ----------

fn build_cmu_native(g: &mut ModuleGraph, topics: &NavTopicConfig) {
    // Terrain analysis / Terrain map ext(native bridges,但这里以 RustAlgorithm 表示,因为
    // explore-terrain-bridge / terrain-map-ext 在我们的命名空间下)
    //
    // Phase E6 stack wiring(R6 补全):当 `native-runner` + `native-terrain` 都
    // 启用时,通过 typed `nuwax-explore-terrain-bridge::build_specs` 把
    // (spec, build) 喂给 `ModuleNode::from_native_specs`,使 args / topics /
    // restart / build policy 写入 `native_spec` 字段;否则保留字符串默认
    // 路径,保证 stack 在无 native spec 时仍可编译 / dry-run。

    // ---- terrain_analysis ----
    let terrain_node = ModuleNode::new(
        "terrain_analysis",
        "Terrain Analysis",
        ModuleKind::NativeBridge,
    )
    .with_feature("native-terrain");
    #[cfg(all(feature = "native-runner", feature = "native-terrain"))]
    let terrain_node = {
        let terrain_topics = nuwax_explore_terrain_bridge::TerrainAnalysisTopics {
            registered_scan: topic_for(topics, "registered_scan", "registered_scan"),
            odometry: topic_for(topics, "odometry", "odometry"),
            terrain_map: topic_for(topics, "terrain_map", "terrain_map"),
        };
        match nuwax_explore_terrain_bridge::build_specs(
            nuwax_explore_terrain_bridge::TerrainAnalysisBridgeConfig::default(),
            &terrain_topics,
        ) {
            Ok((spec, build)) => ModuleNode::from_native_specs(
                "terrain_analysis",
                "Terrain Analysis",
                ModuleKind::NativeBridge,
                spec,
                Some(build),
            )
            .with_feature("native-terrain"),
            Err(_) => terrain_node.with_binary("result/bin/terrain_analysis"),
        }
    };
    #[cfg(not(all(feature = "native-runner", feature = "native-terrain")))]
    let terrain_node = terrain_node.with_binary("result/bin/terrain_analysis");

    g.push_node(
        terrain_node
            .add_input(topics.get_or("registered_scan", "registered_scan"))
            .add_input(topics.get_or("odometry", "odometry"))
            .add_output(topics.get_or("terrain_map", "terrain_map")),
    );

    g.push_node(
        ModuleNode::new(
            "terrain_map_ext",
            "Terrain Map Ext",
            ModuleKind::RustAlgorithm,
        )
        .with_feature("terrain-map-ext")
        .add_input(topics.get_or("registered_scan", "registered_scan"))
        .add_input(topics.get_or("odometry", "odometry"))
        .add_output(topics.get_or("terrain_map_ext", "terrain_map_ext"))
        .depends_on_node("terrain_analysis"),
    );

    // ---- pgo ----
    #[cfg(all(feature = "native-runner", feature = "native-pgo"))]
    let pgo_node = {
        let pgo_topics = nuwax_nav_pgo_bridge::PgoTopics {
            registered_scan: topic_for(topics, "registered_scan", "registered_scan"),
            odometry: topic_for(topics, "odometry", "odometry"),
            corrected_odometry: topic_for(topics, "corrected_odometry", "corrected_odometry"),
            global_map: topic_for(topics, "global_map", "global_map"),
            pgo_tf: topic_for(topics, "pgo_tf", "pgo_tf"),
        };
        ModuleNode::from_native_specs(
            "pgo",
            "PGO",
            ModuleKind::NativeBridge,
            nuwax_nav_pgo_bridge::build_spec(
                nuwax_nav_pgo_bridge::PgoBridgeConfig::default(),
                &pgo_topics,
            ),
            None,
        )
        .with_feature("native-pgo")
    };
    #[cfg(not(all(feature = "native-runner", feature = "native-pgo")))]
    let pgo_node =
        ModuleNode::new("pgo", "PGO", ModuleKind::NativeBridge).with_binary("result/bin/pgo");

    g.push_node(
        pgo_node
            .add_input(topics.get_or("registered_scan", "registered_scan"))
            .add_input(topics.get_or("odometry", "odometry"))
            .add_output(topics.get_or("corrected_odometry", "corrected_odometry"))
            .add_output(topics.get_or("global_map_pgo", "global_map_pgo"))
            .add_output(topics.get_or("pgo_tf", "pgo_tf"))
            .depends_on_node("terrain_analysis"),
    );

    // ---- far_planner ----
    #[cfg(all(feature = "native-runner", feature = "native-far"))]
    let far_node = {
        let far_topics = nuwax_nav_far_planner_bridge::FarPlannerTopics {
            terrain_map_ext: topic_for(topics, "terrain_map_ext", "terrain_map_ext"),
            terrain_map: topic_for(topics, "terrain_map", "terrain_map"),
            registered_scan: topic_for(topics, "registered_scan", "registered_scan"),
            odometry: topic_for(topics, "odometry", "odometry"),
            goal: Some(navigation_goal_topic(topics).to_string()),
            stop_movement: topic_for(topics, "stop_movement", "stop_movement"),
            way_point: topic_for(topics, "way_point", "way_point"),
            goal_path: topic_for(topics, "goal_path", "goal_path"),
            graph_nodes: topic_for(topics, "graph_nodes", "graph_nodes"),
            graph_edges: topic_for(topics, "graph_edges", "graph_edges"),
            contour_polygons: topic_for(topics, "contour_polygons", "contour_polygons"),
            nav_boundary: topic_for(topics, "nav_boundary", "nav_boundary"),
        };
        ModuleNode::from_native_specs(
            "far_planner",
            "FAR Planner",
            ModuleKind::NativeBridge,
            nuwax_nav_far_planner_bridge::build_spec(
                nuwax_nav_far_planner_bridge::FarPlannerBridgeConfig::default(),
                &far_topics,
            ),
            None,
        )
        .with_feature("native-far")
    };
    #[cfg(not(all(feature = "native-runner", feature = "native-far")))]
    let far_node = ModuleNode::new("far_planner", "FAR Planner", ModuleKind::NativeBridge)
        .with_binary("result/bin/far_planner_native");

    g.push_node(
        far_node
            .add_input(topics.get_or("terrain_map", "terrain_map"))
            .add_input(topics.get_or("terrain_map_ext", "terrain_map_ext"))
            .add_input(topics.get_or("registered_scan", "registered_scan"))
            .add_input(topics.get_or("odometry", "odometry"))
            .add_input(navigation_goal_topic(topics))
            .add_input(topics.get_or("stop_movement", "stop_movement"))
            .add_output(topics.get_or("way_point", "way_point"))
            .add_output(topics.get_or("goal_path", "goal_path"))
            .add_output(topics.get_or("nav_boundary", "nav_boundary"))
            .add_output(topics.get_or("graph_nodes", "graph_nodes"))
            .add_output(topics.get_or("graph_edges", "graph_edges"))
            .add_output(topics.get_or("contour_polygons", "contour_polygons"))
            .depends_on_node("terrain_analysis")
            .depends_on_node("terrain_map_ext")
            .depends_on_node("pgo"),
    );

    // ---- local_planner ----
    #[cfg(all(feature = "native-runner", feature = "native-local"))]
    let local_node = {
        let local_topics = nuwax_nav_local_planner_bridge::LocalPlannerTopics {
            registered_scan: topic_for(topics, "registered_scan", "registered_scan"),
            odometry: topic_for(topics, "odometry", "odometry"),
            terrain_map: topic_for(topics, "terrain_map", "terrain_map"),
            joy_cmd: topic_for(topics, "joy_cmd", "joy_cmd"),
            way_point: topic_for(topics, "way_point", "way_point"),
            goal_pose: Some(navigation_goal_topic(topics).to_string()),
            speed: topic_for(topics, "speed", "speed"),
            navigation_boundary: topic_for(topics, "nav_boundary", "nav_boundary"),
            added_obstacles: topic_for(topics, "added_obstacles", "added_obstacles"),
            check_obstacle: topic_for(topics, "check_obstacle", "check_obstacle"),
            cancel_goal: topic_for(topics, "cancel_goal", "cancel_goal"),
            path: topic_for(topics, "path", "path"),
            effective_cmd_vel: topic_for(topics, "effective_cmd_vel", "effective_cmd_vel"),
            free_paths: topic_for(topics, "free_paths", "free_paths"),
            slow_down: topic_for(topics, "slow_down", "slow_down"),
            goal_reached: topic_for(topics, "goal_reached", "goal_reached"),
        };
        ModuleNode::from_native_specs(
            "local_planner",
            "Local Planner",
            ModuleKind::NativeBridge,
            nuwax_nav_local_planner_bridge::build_spec(
                nuwax_nav_local_planner_bridge::LocalPlannerBridgeConfig::default(),
                &local_topics,
            ),
            None,
        )
        .with_feature("native-local")
    };
    #[cfg(not(all(feature = "native-runner", feature = "native-local")))]
    let local_node = ModuleNode::new("local_planner", "Local Planner", ModuleKind::NativeBridge)
        .with_binary("result/bin/local_planner");

    g.push_node(
        local_node
            .add_input(topics.get_or("registered_scan", "registered_scan"))
            .add_input(topics.get_or("odometry", "odometry"))
            .add_input(topics.get_or("terrain_map", "terrain_map"))
            .add_input(topics.get_or("joy_cmd", "joy_cmd"))
            .add_input(topics.get_or("way_point", "way_point"))
            .add_input(navigation_goal_topic(topics))
            .add_input(topics.get_or("speed", "speed"))
            .add_input(topics.get_or("added_obstacles", "added_obstacles"))
            .add_input(topics.get_or("check_obstacle", "check_obstacle"))
            .add_input(topics.get_or("cancel_goal", "cancel_goal"))
            .add_output(topics.get_or("path", "path"))
            .add_output(topics.get_or("effective_cmd_vel", "effective_cmd_vel"))
            .add_output(topics.get_or("free_paths", "free_paths"))
            .add_output(topics.get_or("slow_down", "slow_down"))
            .add_output(topics.get_or("goal_reached", "goal_reached"))
            .depends_on_node("far_planner"),
    );

    // ---- path_follower ----
    #[cfg(all(feature = "native-runner", feature = "native-path-follower"))]
    let pf_node = {
        let pf_topics = nuwax_nav_path_follower_bridge::PathFollowerTopics {
            path: topic_for(topics, "path", "path"),
            odometry: topic_for(topics, "odometry", "odometry"),
            speed: topic_for(topics, "speed", "speed"),
            slow_down: topic_for(topics, "slow_down", "slow_down"),
            safety_stop: topic_for(topics, "safety_stop", "safety_stop"),
            cmd_vel: topic_for(topics, "nav_cmd_vel", "/nav/cmd_vel"),
        };
        ModuleNode::from_native_specs(
            "path_follower",
            "Path Follower",
            ModuleKind::NativeBridge,
            nuwax_nav_path_follower_bridge::build_spec(
                nuwax_nav_path_follower_bridge::PathFollowerBridgeConfig::default(),
                &pf_topics,
            ),
            None,
        )
        .with_feature("native-path-follower")
    };
    #[cfg(not(all(feature = "native-runner", feature = "native-path-follower")))]
    let pf_node = ModuleNode::new("path_follower", "Path Follower", ModuleKind::NativeBridge)
        .with_binary("result/bin/path_follower");

    g.push_node(
        pf_node
            .add_input(topics.get_or("path", "path"))
            .add_input(topics.get_or("odometry", "odometry"))
            .add_input(topics.get_or("speed", "speed"))
            .add_input(topics.get_or("slow_down", "slow_down"))
            .add_input(topics.get_or("safety_stop", "safety_stop"))
            .add_output(topics.get_or("nav_cmd_vel", "/nav/cmd_vel"))
            .depends_on_node("local_planner"),
    );

    g.push_node(
        ModuleNode::new(
            "movement_manager",
            "Movement Manager",
            ModuleKind::Coordinator,
        )
        .with_feature("movement-manager")
        .add_input(topics.get_or("nav_cmd_vel", "/nav/cmd_vel"))
        .add_input(topics.get_or("joy_cmd", "joy_cmd"))
        .add_output(topics.get_or("cmd_vel", "cmd_vel"))
        .add_output(topics.get_or("stop_movement", "stop_movement"))
        .depends_on_node("path_follower"),
    );

    g.push_edge(GraphEdge {
        from: "terrain_analysis".into(),
        to: "terrain_map_ext".into(),
        topic: "registered_scan".into(),
    });
    g.push_edge(GraphEdge {
        from: "pgo".into(),
        to: "far_planner".into(),
        topic: "corrected_odometry".into(),
    });
    g.push_edge(GraphEdge {
        from: "far_planner".into(),
        to: "local_planner".into(),
        topic: "way_point".into(),
    });
    g.push_edge(GraphEdge {
        from: "local_planner".into(),
        to: "path_follower".into(),
        topic: "path".into(),
    });
    g.push_edge(GraphEdge {
        from: "path_follower".into(),
        to: "movement_manager".into(),
        topic: topics.get_or("nav_cmd_vel", "/nav/cmd_vel").into(),
    });
}

// ---------- ExplorationNative ----------

fn build_exploration_native(g: &mut ModuleGraph, topics: &NavTopicConfig) {
    // 复刻 cmu-native pipeline,但额外加 explore 模块
    build_cmu_native(g, topics);

    g.push_node(
        ModuleNode::new(
            "wavefront_explorer",
            "Wavefront Explorer",
            ModuleKind::Explorer,
        )
        .with_feature("explore")
        .add_input(topics.get_or("occupancy", "/map/occupancy"))
        .add_input(topics.get_or("corrected_odometry", "corrected_odometry"))
        .add_input(topics.get_or("goal_reached", "goal_reached"))
        .add_input(topics.get_or("goal_failed", "goal_failed"))
        .add_input(topics.get_or("stop_movement", "stop_movement"))
        .add_output(navigation_goal_topic(topics))
        .depends_on_node("pgo"),
    );

    // ---- tare_planner ----
    #[cfg(all(feature = "native-runner", feature = "native-tare"))]
    let tare_node = {
        let tare_topics = nuwax_explore_tare_bridge::TarePlannerTopics {
            registered_scan: topic_for(topics, "registered_scan", "registered_scan"),
            odometry: topic_for(topics, "odometry", "odometry"),
            way_point: topic_for(topics, "way_point", "way_point"),
        };
        match nuwax_explore_tare_bridge::build_specs(
            nuwax_explore_tare_bridge::TarePlannerBridgeConfig::default(),
            &tare_topics,
        ) {
            Ok((spec, build)) => ModuleNode::from_native_specs(
                "tare_planner",
                "TARE Planner",
                ModuleKind::NativeBridge,
                spec,
                Some(build),
            )
            .with_feature("native-tare"),
            Err(_) => ModuleNode::new("tare_planner", "TARE Planner", ModuleKind::NativeBridge)
                .with_binary("result/bin/tare_planner"),
        }
    };
    #[cfg(not(all(feature = "native-runner", feature = "native-tare")))]
    let tare_node = ModuleNode::new("tare_planner", "TARE Planner", ModuleKind::NativeBridge)
        .with_binary("result/bin/tare_planner")
        .with_feature("native-tare");

    g.push_node(
        tare_node
            .add_input(topics.get_or("registered_scan", "registered_scan"))
            .add_input(topics.get_or("odometry", "odometry"))
            .add_output("tare_goal"),
    );

    g.push_edge(GraphEdge {
        from: "wavefront_explorer".into(),
        to: "far_planner".into(),
        topic: navigation_goal_topic(topics).into(),
    });
    g.push_edge(GraphEdge {
        from: "tare_planner".into(),
        to: "far_planner".into(),
        topic: "tare_goal".into(),
    });
}

// ---------- Mls3D ----------

fn build_mls3d(g: &mut ModuleGraph, topics: &NavTopicConfig) {
    #[cfg(feature = "native-runner")]
    let planner_node = {
        use nuwax_nav_native_runner::prelude::RestartPolicy;
        use nuwax_nav_native_runner::{NativeBuildPolicy, NativeBuildSpec, NativeProgramSpec};

        let workspace_root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .and_then(std::path::Path::parent)
            .expect("nuwax-nav-stack must live under <workspace>/crates/<crate>")
            .to_path_buf();
        let target_dir = workspace_root.join("target");
        let executable = target_dir.join("release").join("mls_planner_native");

        let spec = NativeProgramSpec {
            name: "mls_planner".into(),
            executable,
            topics: topic_map_for_node("mls_planner", topics)
                .into_iter()
                .map(|(port, topic)| (port, absolute_lcm_topic(topic)))
                .collect(),
            args: Vec::new(),
            env: std::collections::BTreeMap::new(),
            cwd: Some(workspace_root.clone()),
            restart: RestartPolicy::Never,
        };
        let build = NativeBuildSpec {
            build_command: Some(
                "cargo build --release -p nuwax-nav-mls-planner --bin mls_planner_native".into(),
            ),
            policy: NativeBuildPolicy::IfMissing,
            cwd: Some(workspace_root),
            env: std::collections::BTreeMap::from([(
                "CARGO_TARGET_DIR".into(),
                target_dir.to_string_lossy().into_owned(),
            )]),
        };
        ModuleNode::from_native_specs(
            "mls_planner",
            "MLS Planner",
            ModuleKind::NativeBridge,
            spec,
            Some(build),
        )
    };
    #[cfg(not(feature = "native-runner"))]
    let planner_node = ModuleNode::new("mls_planner", "MLS Planner", ModuleKind::NativeBridge)
        .with_binary("services/nuwax-workspace/target/release/mls_planner_native");

    g.push_node(
        planner_node
            .with_feature("mls")
            .add_input(topics.get_or("global_map", "global_map"))
            .add_input(topics.get_or("local_map", "local_map"))
            .add_input(topics.get_or("region_bounds", "region_bounds"))
            .add_input(topics.get_or("start_pose", "start_pose"))
            .add_input(topics.get_or("goal_pose", "goal_pose"))
            .add_output(topics.get_or("goal_path", "goal_path"))
            .add_output(topics.get_or("surface_map", "mls_surface"))
            .add_output(topics.get_or("nodes", "mls_nodes"))
            .add_output(topics.get_or("node_edges", "mls_node_edges")),
    );

    g.push_node(
        ModuleNode::new(
            "basic_path_follower",
            "Basic Path Follower",
            ModuleKind::RustAlgorithm,
        )
        .with_feature("basic-path-follower")
        .add_input(topics.get_or("goal_path", "goal_path"))
        .add_input(topics.get_or("odometry", "odometry"))
        .add_input(topics.get_or("stop_movement", "stop_movement"))
        .add_output(topics.get_or("nav_cmd_vel", "/nav/cmd_vel"))
        .depends_on_node("mls_planner"),
    );

    g.push_node(
        ModuleNode::new(
            "movement_manager",
            "Movement Manager",
            ModuleKind::Coordinator,
        )
        .with_feature("movement-manager")
        .add_input(topics.get_or("nav_cmd_vel", "/nav/cmd_vel"))
        .add_input(topics.get_or("joy_cmd", "joy_cmd"))
        .add_output(topics.get_or("cmd_vel", "cmd_vel"))
        .add_output(topics.get_or("stop_movement", "stop_movement"))
        .depends_on_node("basic_path_follower"),
    );

    g.push_edge(GraphEdge {
        from: "mls_planner".into(),
        to: "basic_path_follower".into(),
        topic: topics.get_or("goal_path", "goal_path").into(),
    });
}

#[cfg(feature = "native-runner")]
fn absolute_lcm_topic(topic: String) -> String {
    if topic.starts_with('/') {
        topic
    } else {
        format!("/{topic}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::features::NavStackPreset;

    fn empty_topics() -> NavTopicConfig {
        NavTopicConfig::empty()
    }

    #[test]
    fn simple2d_builds_and_validates() {
        let g = build_graph(
            NavStackProfile::Simple2D,
            &empty_topics(),
            NavStackPreset::None.to_flags(),
        );
        g.validate().expect("validate");
        assert!(g.find("simple_planner").is_some());
        assert!(g.find("map_pose_adapter").is_some());
        assert!(g.find("guarded_nav_scheduler").is_some());
        assert!(g.find("basic_path_follower").is_none());
        assert!(g.find("movement_manager").is_none());
        assert!(g.find("navigation_state_reporter").is_none());
        let planner = g.find("simple_planner").expect("simple planner");
        assert_eq!(planner.inputs[0], "/map/occupancy");
        assert_eq!(planner.inputs.len(), 4);
        assert!(!planner.inputs.iter().any(|input| input == "start_pose"));
        assert!(g.find("click_goal_router").is_none());
        assert!(!planner.inputs.iter().any(|input| input == "terrain_map"));
    }

    #[test]
    fn cmu_native_includes_all_bridges() {
        let g = build_graph(
            NavStackProfile::CmuNative,
            &empty_topics(),
            NavStackPreset::Cmu.to_flags(),
        );
        g.validate().expect("validate");
        for id in ["far_planner", "local_planner", "path_follower", "pgo"] {
            assert!(g.find(id).is_some(), "missing node {id}");
        }
    }

    #[test]
    fn exploration_extends_cmu() {
        let g = build_graph(
            NavStackProfile::ExplorationNative,
            &empty_topics(),
            NavStackPreset::Explore.to_flags(),
        );
        g.validate().expect("validate");
        let wavefront = g.find("wavefront_explorer").expect("wavefront node");
        assert_eq!(wavefront.inputs.len(), 5);
        assert_eq!(wavefront.inputs[0], "/map/occupancy");
        assert_eq!(wavefront.outputs, vec!["goal_pose"]);
        assert!(g.find("far_planner").is_some());
    }

    #[test]
    fn astar_profile_uses_guarded_scheduler_owner() {
        let g = build_graph(
            NavStackProfile::ReplanningAStar,
            &empty_topics(),
            NavStackPreset::Simple.to_flags(),
        );
        g.validate().expect("validate");
        assert!(g.find("astar_planner").is_some());
        assert!(g.find("map_pose_adapter").is_some());
        assert!(g.find("guarded_nav_scheduler").is_some());
        assert!(g.find("basic_path_follower").is_none());
        assert!(g.find("movement_manager").is_none());
        assert!(g.find("navigation_state_reporter").is_none());
        assert_eq!(g.find("astar_planner").unwrap().inputs[0], "map_pose");
        assert_eq!(g.find("astar_planner").unwrap().inputs[1], "/map/occupancy");
        assert!(g.find("click_goal_router").is_none());
        assert!(g.find("patrolling").is_none());
    }

    #[test]
    fn mls_profile_includes_planner_and_follower() {
        let g = build_graph(
            NavStackProfile::Mls3D,
            &empty_topics(),
            NavStackPreset::Mls.to_flags(),
        );
        g.validate().expect("validate");
        assert!(g.find("mls_planner").is_some());
        assert!(g.find("basic_path_follower").is_some());
        assert_eq!(
            g.find("mls_planner").unwrap().kind,
            ModuleKind::NativeBridge
        );
        #[cfg(feature = "native-runner")]
        assert_eq!(
            g.find("mls_planner")
                .unwrap()
                .native_spec
                .as_ref()
                .unwrap()
                .spec
                .topics
                .len(),
            9
        );
    }

    /// R6 验收测试:在 `native-runner` feature 启用 + 对应 bridge feature 启用时,
    /// 各 native node 的 `node.native_spec` 必须非空,且 `binary` 派生自
    /// `spec.executable`;serde roundtrip 后 `native_spec.spec` 必须保持。
    #[cfg(all(
        feature = "native-runner",
        feature = "native-terrain",
        feature = "native-pgo",
        feature = "native-far",
        feature = "native-local",
        feature = "native-path-follower",
    ))]
    #[test]
    fn cmu_native_profile_populates_native_spec_on_all_bridges() {
        let g = build_graph(
            NavStackProfile::CmuNative,
            &empty_topics(),
            NavStackPreset::Cmu.to_flags(),
        );
        g.validate().expect("validate");

        for id in [
            "terrain_analysis",
            "pgo",
            "far_planner",
            "local_planner",
            "path_follower",
        ] {
            let node = g.find(id).unwrap_or_else(|| panic!("missing {id}"));
            let native = node
                .native_spec
                .as_ref()
                .unwrap_or_else(|| panic!("{id} should have native_spec when typed wiring is on"));
            // binary 必须由 spec.executable 派生(不依赖手填字符串)
            assert_eq!(
                node.binary.as_deref(),
                Some(native.binary_display().as_str()),
                "{id} binary mismatch"
            );
            // spec 字段不能为空
            assert!(!native.spec.name.is_empty(), "{id} spec.name empty");
            // serde roundtrip 保留完整 spec
            let s = serde_json::to_string(node).expect("serialize");
            let back: ModuleNode = serde_json::from_str(&s).expect("deserialize");
            assert_eq!(
                back.native_spec.as_ref().map(|n| n.spec.clone()),
                Some(native.spec.clone()),
                "{id} roundtrip lost native_spec.spec"
            );
        }

        // terrain_analysis 同时持有 build policy(summary.build.is_some());
        // 其它 single-spec bridges 不携带 build。
        let terrain = g.find("terrain_analysis").unwrap();
        let terrain_spec = terrain.native_spec.as_ref().unwrap();
        assert!(
            terrain_spec.build.is_some(),
            "terrain_analysis should carry NativeBuildSpec via summary"
        );
        let pgo = g.find("pgo").unwrap();
        assert!(
            pgo.native_spec.as_ref().unwrap().build.is_none(),
            "pgo should not carry NativeBuildSpec"
        );
    }

    /// R6 验收测试:无 `native-runner` feature 时,native node 必须保留
    /// `binary` fallback 字符串;此时 `ModuleNode.native_spec` 字段不存在
    /// (cfg gating),用 binary fallback 的存在性来代替断言。
    #[cfg(not(all(
        feature = "native-runner",
        feature = "native-terrain",
        feature = "native-pgo",
        feature = "native-far",
        feature = "native-local",
        feature = "native-path-follower",
    )))]
    #[test]
    fn cmu_native_profile_uses_legacy_binary_when_native_runner_off() {
        let g = build_graph(
            NavStackProfile::CmuNative,
            &empty_topics(),
            NavStackPreset::Cmu.to_flags(),
        );
        g.validate().expect("validate");

        for id in [
            "terrain_analysis",
            "pgo",
            "far_planner",
            "local_planner",
            "path_follower",
        ] {
            let node = g.find(id).unwrap_or_else(|| panic!("missing {id}"));
            assert!(
                node.binary.is_some(),
                "{id} must keep binary fallback when typed wiring is off"
            );
        }
    }

    /// R6 验收测试:ExplorationNative profile 在 native-runner + native-tare
    /// 同时启用时,`tare_planner` 必须有 native_spec。
    #[cfg(all(feature = "native-runner", feature = "native-tare"))]
    #[test]
    fn exploration_native_profile_tare_has_native_spec() {
        let g = build_graph(
            NavStackProfile::ExplorationNative,
            &empty_topics(),
            NavStackPreset::Explore.to_flags(),
        );
        g.validate().expect("validate");

        let tare = g.find("tare_planner").unwrap();
        let tare_native = tare
            .native_spec
            .as_ref()
            .expect("tare_planner should have native_spec under typed wiring");
        assert!(
            tare_native.build.is_some(),
            "tare_planner should carry NativeBuildSpec via summary"
        );
    }
}
