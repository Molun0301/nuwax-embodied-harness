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

// ---------- Simple2D ----------

fn build_simple2d(g: &mut ModuleGraph, topics: &NavTopicConfig) {
    // 1. Click router:把 clicked_point 转换为 start/goal
    g.push_node(
        ModuleNode::new(
            "click_goal_router",
            "Click Goal Router",
            ModuleKind::Coordinator,
        )
        .with_feature("click-goal")
        .add_input(topics.get_or("clicked_point", "clicked_point"))
        .add_output("start_pose")
        .add_output("goal_pose"),
    );

    // 2. Simple planner:terrain + goal -> waypoint / goal_path
    g.push_node(
        ModuleNode::new(
            "simple_planner",
            "Simple Planner",
            ModuleKind::RustAlgorithm,
        )
        .with_feature("simple-planner")
        .add_input(topics.get_or("terrain_map", "terrain_map"))
        .add_input(topics.get_or("terrain_map_ext", "terrain_map_ext"))
        .add_input(topics.get_or("odometry", "odometry"))
        .add_input("start_pose")
        .add_input("goal_pose")
        .add_output(topics.get_or("way_point", "way_point"))
        .add_output(topics.get_or("goal_path", "goal_path"))
        .depends_on_node("click_goal_router"),
    );

    // 3. Basic path follower:waypoint -> cmd_vel
    g.push_node(
        ModuleNode::new(
            "basic_path_follower",
            "Basic Path Follower",
            ModuleKind::RustAlgorithm,
        )
        .with_feature("basic-path-follower")
        .add_input(topics.get_or("way_point", "way_point"))
        .add_input(topics.get_or("odometry", "odometry"))
        .add_input(topics.get_or("stop_movement", "stop_movement"))
        .add_output(topics.get_or("nav_cmd_vel", "nav_cmd_vel"))
        .add_output(topics.get_or("goal_reached", "goal_reached"))
        .depends_on_node("simple_planner"),
    );

    // 4. Movement manager:teleop mux + cmd_vel final
    g.push_node(
        ModuleNode::new(
            "movement_manager",
            "Movement Manager",
            ModuleKind::Coordinator,
        )
        .with_feature("movement-manager")
        .add_input(topics.get_or("nav_cmd_vel", "nav_cmd_vel"))
        .add_input(topics.get_or("joy_cmd", "joy_cmd"))
        .add_input(topics.get_or("clicked_point", "clicked_point"))
        .add_output(topics.get_or("cmd_vel", "cmd_vel"))
        .add_output(topics.get_or("goal", "goal"))
        .add_output(topics.get_or("way_point", "way_point_out"))
        .add_output(topics.get_or("stop_movement", "stop_movement"))
        .depends_on_node("basic_path_follower")
        .depends_on_node("click_goal_router"),
    );

    // edges
    g.push_edge(GraphEdge {
        from: "click_goal_router".into(),
        to: "simple_planner".into(),
        topic: "goal_pose".into(),
    });
    g.push_edge(GraphEdge {
        from: "simple_planner".into(),
        to: "basic_path_follower".into(),
        topic: "way_point".into(),
    });
    g.push_edge(GraphEdge {
        from: "basic_path_follower".into(),
        to: "movement_manager".into(),
        topic: "nav_cmd_vel".into(),
    });
}

// ---------- ReplanningAStar ----------

fn build_replanning_astar(g: &mut ModuleGraph, topics: &NavTopicConfig) {
    g.push_node(
        ModuleNode::new(
            "click_goal_router",
            "Click Goal Router",
            ModuleKind::Coordinator,
        )
        .with_feature("click-goal")
        .add_input(topics.get_or("clicked_point", "clicked_point"))
        .add_output("goal_pose"),
    );

    g.push_node(
        ModuleNode::new(
            "astar_planner",
            "Replanning A* Planner",
            ModuleKind::RustAlgorithm,
        )
        .with_feature("astar")
        .add_input(topics.get_or("odometry", "odometry"))
        .add_input(topics.get_or("registered_scan", "registered_scan"))
        .add_input("goal_pose")
        .add_output(topics.get_or("way_point", "way_point"))
        .add_output(topics.get_or("goal_path", "goal_path"))
        .add_output(topics.get_or("goal_reached", "goal_reached"))
        .depends_on_node("click_goal_router"),
    );

    g.push_node(
        ModuleNode::new(
            "basic_path_follower",
            "Basic Path Follower",
            ModuleKind::RustAlgorithm,
        )
        .with_feature("basic-path-follower")
        .add_input(topics.get_or("way_point", "way_point"))
        .add_input(topics.get_or("odometry", "odometry"))
        .add_input(topics.get_or("stop_movement", "stop_movement"))
        .add_output(topics.get_or("nav_cmd_vel", "nav_cmd_vel"))
        .depends_on_node("astar_planner"),
    );

    g.push_node(
        ModuleNode::new(
            "movement_manager",
            "Movement Manager",
            ModuleKind::Coordinator,
        )
        .with_feature("movement-manager")
        .add_input(topics.get_or("nav_cmd_vel", "nav_cmd_vel"))
        .add_input(topics.get_or("joy_cmd", "joy_cmd"))
        .add_output(topics.get_or("cmd_vel", "cmd_vel"))
        .add_output(topics.get_or("stop_movement", "stop_movement"))
        .depends_on_node("basic_path_follower"),
    );

    g.push_edge(GraphEdge {
        from: "click_goal_router".into(),
        to: "astar_planner".into(),
        topic: "goal_pose".into(),
    });
    g.push_edge(GraphEdge {
        from: "astar_planner".into(),
        to: "basic_path_follower".into(),
        topic: "way_point".into(),
    });
    g.push_edge(GraphEdge {
        from: "basic_path_follower".into(),
        to: "movement_manager".into(),
        topic: "nav_cmd_vel".into(),
    });
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
    let terrain_node = match nuwax_explore_terrain_bridge::build_specs(
        nuwax_explore_terrain_bridge::TerrainAnalysisBridgeConfig::default(),
        &nuwax_explore_terrain_bridge::TerrainAnalysisTopics::default(),
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
    let pgo_node = ModuleNode::from_native_specs(
        "pgo",
        "PGO",
        ModuleKind::NativeBridge,
        nuwax_nav_pgo_bridge::build_spec(
            nuwax_nav_pgo_bridge::PgoBridgeConfig::default(),
            &nuwax_nav_pgo_bridge::PgoTopics::default(),
        ),
        None,
    )
    .with_feature("native-pgo");
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
    let far_node = ModuleNode::from_native_specs(
        "far_planner",
        "FAR Planner",
        ModuleKind::NativeBridge,
        nuwax_nav_far_planner_bridge::build_spec(
            nuwax_nav_far_planner_bridge::FarPlannerBridgeConfig::default(),
            &nuwax_nav_far_planner_bridge::FarPlannerTopics::default(),
        ),
        None,
    )
    .with_feature("native-far");
    #[cfg(not(all(feature = "native-runner", feature = "native-far")))]
    let far_node = ModuleNode::new("far_planner", "FAR Planner", ModuleKind::NativeBridge)
        .with_binary("result/bin/far_planner_native");

    g.push_node(
        far_node
            .add_input(topics.get_or("terrain_map", "terrain_map"))
            .add_input(topics.get_or("terrain_map_ext", "terrain_map_ext"))
            .add_input(topics.get_or("registered_scan", "registered_scan"))
            .add_input(topics.get_or("odometry", "odometry"))
            .add_input(topics.get_or("goal", "goal"))
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
    let local_node = ModuleNode::from_native_specs(
        "local_planner",
        "Local Planner",
        ModuleKind::NativeBridge,
        nuwax_nav_local_planner_bridge::build_spec(
            nuwax_nav_local_planner_bridge::LocalPlannerBridgeConfig::default(),
            &nuwax_nav_local_planner_bridge::LocalPlannerTopics::default(),
        ),
        None,
    )
    .with_feature("native-local");
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
            .add_input(topics.get_or("goal", "goal"))
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
    let pf_node = ModuleNode::from_native_specs(
        "path_follower",
        "Path Follower",
        ModuleKind::NativeBridge,
        nuwax_nav_path_follower_bridge::build_spec(
            nuwax_nav_path_follower_bridge::PathFollowerBridgeConfig::default(),
            &nuwax_nav_path_follower_bridge::PathFollowerTopics::default(),
        ),
        None,
    )
    .with_feature("native-path-follower");
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
            .add_output(topics.get_or("cmd_vel", "cmd_vel"))
            .depends_on_node("local_planner"),
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
        .add_input(topics.get_or("terrain_map", "terrain_map"))
        .add_input(topics.get_or("corrected_odometry", "corrected_odometry"))
        .add_output("frontier_goal"),
    );

    // ---- tare_planner ----
    #[cfg(all(feature = "native-runner", feature = "native-tare"))]
    let tare_node = match nuwax_explore_tare_bridge::build_specs(
        nuwax_explore_tare_bridge::TarePlannerBridgeConfig::default(),
        &nuwax_explore_tare_bridge::TarePlannerTopics::default(),
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
        topic: "frontier_goal".into(),
    });
    g.push_edge(GraphEdge {
        from: "tare_planner".into(),
        to: "far_planner".into(),
        topic: "tare_goal".into(),
    });
}

// ---------- Mls3D ----------

fn build_mls3d(g: &mut ModuleGraph, topics: &NavTopicConfig) {
    g.push_node(
        ModuleNode::new(
            "mls_transformer",
            "MLS Transformer",
            ModuleKind::Coordinator,
        )
        .with_feature("mls")
        .add_input(topics.get_or("registered_scan", "registered_scan"))
        .add_input(topics.get_or("odometry", "odometry"))
        .add_output("mls_request"),
    );

    g.push_node(
        ModuleNode::new("mls_planner", "MLS Planner", ModuleKind::RustAlgorithm)
            .with_feature("mls")
            .add_input("mls_request")
            .add_output(topics.get_or("goal_path", "goal_path"))
            .add_output("mls_surface")
            .depends_on_node("mls_transformer"),
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
        .add_output(topics.get_or("nav_cmd_vel", "nav_cmd_vel"))
        .depends_on_node("mls_planner"),
    );

    g.push_node(
        ModuleNode::new(
            "movement_manager",
            "Movement Manager",
            ModuleKind::Coordinator,
        )
        .with_feature("movement-manager")
        .add_input(topics.get_or("nav_cmd_vel", "nav_cmd_vel"))
        .add_input(topics.get_or("joy_cmd", "joy_cmd"))
        .add_output(topics.get_or("cmd_vel", "cmd_vel"))
        .add_output(topics.get_or("stop_movement", "stop_movement"))
        .depends_on_node("basic_path_follower"),
    );

    g.push_edge(GraphEdge {
        from: "mls_transformer".into(),
        to: "mls_planner".into(),
        topic: "mls_request".into(),
    });
    g.push_edge(GraphEdge {
        from: "mls_planner".into(),
        to: "basic_path_follower".into(),
        topic: "goal_path".into(),
    });
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
        assert!(g.find("basic_path_follower").is_some());
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
        assert!(g.find("wavefront_explorer").is_some());
        assert!(g.find("far_planner").is_some());
    }

    #[test]
    fn astar_profile_uses_simple_follower() {
        let g = build_graph(
            NavStackProfile::ReplanningAStar,
            &empty_topics(),
            NavStackPreset::Simple.to_flags(),
        );
        g.validate().expect("validate");
        assert!(g.find("astar_planner").is_some());
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
