//! Feature-gated lifecycle owners for in-process graph nodes.
//!
//! Algorithms remain in their own crates. This module only adapts their
//! lifecycle APIs to [`crate::runner::InProcessNode`] and, for synchronous
//! modules, supplies the periodic scheduler their module contract requires.

use crate::runner::StackRunner;

#[cfg(all(
    feature = "native-runner",
    any(feature = "cmu-native", feature = "explore", feature = "mls")
))]
fn native_topic(node: &crate::graph::ModuleNode, port: &str) -> Option<String> {
    node.native_spec.as_ref()?.spec.topics.get(port).cloned()
}

#[cfg(all(
    feature = "native-runner",
    any(feature = "cmu-native", feature = "explore")
))]
fn far_ready_contract(
    node: &crate::graph::ModuleNode,
) -> Option<crate::lcm_dataflow::NativePlannerReadyContract> {
    Some(crate::lcm_dataflow::NativePlannerReadyContract::Far(
        nuwax_nav_far_planner_bridge::worker::FarReadyContract {
            backend: "far-visibility-graph".into(),
            generation: 0,
            goal: native_topic(node, "goal")?,
            cancel: native_topic(node, "stop_movement")?,
            odometry: native_topic(node, "odometry")?,
            terrain_map_ext: native_topic(node, "terrain_map_ext")?,
            terrain_map: native_topic(node, "terrain_map")?,
            registered_scan: native_topic(node, "registered_scan")?,
            planner_result: native_topic(node, "goal_path")?,
            worker_event: native_topic(node, "way_point")?,
            result_contract: nuwax_nav_far_planner_bridge::worker::FAR_RESULT_V3_CONTRACT.into(),
        },
    ))
}

/// Register every concrete in-process owner available in this build.
pub(crate) fn register_compiled_owners(_runner: &mut StackRunner) {
    #[cfg(any(feature = "guard-runtime", feature = "cmu-native", feature = "explore", feature = "mls"))]
    let body_reference = crate::lcm_dataflow::capture_body_reference();
    #[cfg(all(
        feature = "native-runner",
        any(feature = "cmu-native", feature = "explore", feature = "mls")
    ))]
    {
        #[cfg(any(feature = "cmu-native", feature = "explore"))]
        if let Some(planner) = _runner.graph().find("far_planner").cloned() {
            if let Some(expected) = far_ready_contract(&planner) {
                _runner.register_native_readiness(
                    "far_planner",
                    crate::runner::NativeLifecycleOptions::default(),
                    move |generation, _identity| {
                        crate::lcm_dataflow::native_planner_readiness(expected.clone(), generation)
                    },
                );
            }
        }
    }

    #[cfg(feature = "guard-runtime")]
    {
        _runner
            .register_in_process_factory("map_pose_adapter", { let body = body_reference.clone(); move |node: &crate::graph::ModuleNode| crate::lcm_dataflow::map_pose_owner(node, crate::lcm_dataflow::captured_body_reference(&body)?) });
        _runner.register_in_process_factory(
            "guarded_nav_scheduler",
            { let body = body_reference.clone(); move |node: &crate::graph::ModuleNode| crate::lcm_dataflow::guarded_nav_scheduler_owner(node, crate::lcm_dataflow::captured_body_reference(&body)?) },
        );
    }

    #[cfg(feature = "click-goal")]
    _runner.register_in_process_factory("click_goal_router", crate::lcm_dataflow::click_goal_owner);

    #[cfg(feature = "simple-2d")]
    _runner
        .register_in_process_factory("simple_planner", { let body = body_reference.clone(); move |node: &crate::graph::ModuleNode| crate::lcm_dataflow::simple_planner_owner(node, crate::lcm_dataflow::captured_body_reference(&body)?) });

    #[cfg(feature = "replanning-astar")]
    _runner.register_in_process_factory("astar_planner", { let body = body_reference.clone(); move |node: &crate::graph::ModuleNode| crate::lcm_dataflow::astar_planner_owner(node, crate::lcm_dataflow::captured_body_reference(&body)?) });

    #[cfg(any(feature = "cmu-native", feature = "explore", feature = "mls"))]
    _runner.register_in_process_factory(
        "native_planner_path_adapter",
        { let body = body_reference.clone(); move |node: &crate::graph::ModuleNode| crate::lcm_dataflow::native_planner_path_adapter_owner(node, crate::lcm_dataflow::captured_body_reference(&body)?) },
    );

    #[cfg(feature = "basic-path-follower")]
    _runner.register_in_process_factory(
        "basic_path_follower",
        crate::lcm_dataflow::basic_path_follower_owner,
    );

    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    _runner.register_in_process_factory(
        "terrain_map_ext",
        crate::lcm_dataflow::terrain_map_ext_owner,
    );

    #[cfg(feature = "explore")]
    _runner.register_in_process_factory("wavefront_explorer", crate::lcm_dataflow::wavefront_owner);

    #[cfg(feature = "patrolling")]
    _runner.register_in_process_factory("patrolling", crate::lcm_dataflow::patrolling_owner);
    #[cfg(feature = "mls")]
    crate::lcm_dataflow::register_surface_owners(_runner, body_reference);
}

#[cfg(all(
    test,
    any(
        feature = "simple-2d",
        feature = "replanning-astar",
        feature = "cmu-native",
        feature = "explore",
        feature = "mls"
    )
))]
mod tests {
    #[cfg(feature = "simple-2d")]
    use std::time::Duration;

    #[cfg(feature = "simple-2d")]
    use crate::runner::InProcessNode;
    #[cfg(any(feature = "simple-2d", feature = "mls"))]
    use crate::NavStackBuilder;
    #[cfg(any(feature = "simple-2d", feature = "mls"))]
    use crate::NavStackPreset;
    #[cfg(any(feature = "simple-2d", feature = "replanning-astar", feature = "mls"))]
    use crate::NavStackProfile;
    #[cfg(feature = "simple-2d")]
    use crate::NavStackResult;

    #[cfg(feature = "simple-2d")]
    struct HealthyTestNode(bool);

    #[cfg(feature = "simple-2d")]
    impl InProcessNode for HealthyTestNode {
        fn start(&mut self) -> NavStackResult<()> {
            self.0 = true;
            Ok(())
        }

        fn is_running(&mut self) -> NavStackResult<bool> {
            Ok(self.0)
        }

        fn shutdown(&mut self) -> NavStackResult<()> {
            self.0 = false;
            Ok(())
        }
    }

    #[cfg(feature = "simple-2d")]
    fn healthy_test_owner(
        _node: &crate::graph::ModuleNode,
    ) -> NavStackResult<Box<dyn InProcessNode>> {
        Ok(Box::new(HealthyTestNode(false)))
    }

    #[cfg(feature = "simple-2d")]
    #[tokio::test]
    async fn simple_profile_owns_starts_and_stops_every_in_process_node() {
        let mut runner = NavStackBuilder::new(NavStackProfile::Simple2D)
            .preset(NavStackPreset::Simple)
            .build_runner()
            .unwrap();
        assert!(runner.unowned_nodes().is_empty());
        let expected_nodes = runner.graph().nodes().len();
        runner.register_in_process_factory("guarded_nav_scheduler", healthy_test_owner);
        runner.spawn_all().await.unwrap();
        runner.ensure_healthy().unwrap();
        assert_eq!(runner.running_nodes().len(), expected_nodes);
        runner
            .shutdown_with(Duration::from_millis(20))
            .await
            .unwrap();
        assert!(runner.running_nodes().is_empty());
    }

    #[cfg(feature = "replanning-astar")]
    #[tokio::test]
    async fn astar_profile_has_no_placeholder_owners() {
        let runner = NavStackBuilder::new(NavStackProfile::ReplanningAStar)
            .build_runner()
            .unwrap();
        assert!(runner.unowned_nodes().is_empty());
    }

    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    #[test]
    fn far_expected_contract_is_derived_from_graph_native_spec() {
        let far_topics = nuwax_nav_far_planner_bridge::FarPlannerTopics {
            terrain_map_ext: Some("/contract/terrain-ext".into()),
            terrain_map: Some("/contract/terrain".into()),
            registered_scan: Some("/contract/scan".into()),
            odometry: Some("/contract/odometry".into()),
            goal: Some("/contract/goal".into()),
            stop_movement: Some("/contract/cancel".into()),
            way_point: Some("/contract/event".into()),
            goal_path: Some("/contract/result".into()),
            graph_nodes: None,
            graph_edges: None,
            contour_polygons: None,
            nav_boundary: None,
        };
        let planner = crate::graph::ModuleNode::from_native_specs(
            "far_planner",
            "FAR Planner",
            crate::graph::ModuleKind::NativeBridge,
            nuwax_nav_far_planner_bridge::build_spec(
                nuwax_nav_far_planner_bridge::FarPlannerBridgeConfig::default(),
                &far_topics,
            ),
            None,
        );
        let mut graph = crate::graph::ModuleGraph::new("far-contract-test");
        graph.push_node(planner);
        let planner = graph.find("far_planner").expect("FAR planner graph node");
        let ready_contract =
            super::far_ready_contract(planner).expect("derived FAR READY contract");
        #[cfg(feature = "mls")]
        let contract = match ready_contract {
            crate::lcm_dataflow::NativePlannerReadyContract::Far(contract) => contract,
            crate::lcm_dataflow::NativePlannerReadyContract::Mls(_) => {
                panic!("FAR node derived MLS contract")
            }
        };
        #[cfg(not(feature = "mls"))]
        let crate::lcm_dataflow::NativePlannerReadyContract::Far(contract) = ready_contract;
        let topics = &planner
            .native_spec
            .as_ref()
            .expect("FAR native spec")
            .spec
            .topics;
        assert_eq!(contract.goal, topics["goal"]);
        assert_eq!(contract.cancel, topics["stop_movement"]);
        assert_eq!(contract.odometry, topics["odometry"]);
        assert_eq!(contract.terrain_map_ext, topics["terrain_map_ext"]);
        assert_eq!(contract.terrain_map, topics["terrain_map"]);
        assert_eq!(contract.registered_scan, topics["registered_scan"]);
        assert_eq!(contract.planner_result, topics["goal_path"]);
        assert_eq!(contract.worker_event, topics["way_point"]);
        assert_eq!(contract.backend, "far-visibility-graph");
        assert_eq!(contract.generation, 0);
        assert_eq!(
            contract.result_contract,
            nuwax_nav_far_planner_bridge::worker::FAR_RESULT_V3_CONTRACT
        );
    }

    #[cfg(feature = "mls")]
    #[test]
    fn mls_profile_has_only_shared_surface_in_process_owners() {
        let runner = NavStackBuilder::new(NavStackProfile::Mls3D)
            .build_runner()
            .unwrap();
        assert!(runner.native_bridge_ids().is_empty());
        assert!(runner.unowned_nodes().is_empty());
        let graph = runner.graph();
        let planner = graph.find("mls_planner").unwrap();
        let scheduler = graph.find("surface_nav_scheduler").unwrap();
        assert_eq!(planner.inputs.len(), 5);
        assert_eq!(planner.inputs[4], scheduler.outputs[1]);
        assert_eq!(planner.outputs[0], scheduler.inputs[2]);
        assert_eq!(
            graph
                .find("map_pose_adapter")
                .unwrap()
                .feature_gate
                .as_deref(),
            Some("mls")
        );
        assert!(graph.find("basic_path_follower").is_none());
        assert!(graph.find("native_planner_path_adapter").is_none());
    }
}
