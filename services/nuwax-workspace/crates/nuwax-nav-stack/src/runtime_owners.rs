//! Feature-gated lifecycle owners for in-process graph nodes.
//!
//! Algorithms remain in their own crates. This module only adapts their
//! lifecycle APIs to [`crate::runner::InProcessNode`] and, for synchronous
//! modules, supplies the periodic scheduler their module contract requires.

#[cfg(all(
    test,
    any(feature = "simple-2d", feature = "replanning-astar", feature = "mls")
))]
use std::time::Duration;

use crate::runner::StackRunner;

/// Register every concrete in-process owner available in this build.
pub(crate) fn register_compiled_owners(_runner: &mut StackRunner) {
    #[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
    _runner.register_in_process_factory(
        "navigation_state_reporter",
        crate::lcm_dataflow::navigation_state_owner,
    );

    #[cfg(feature = "click-goal")]
    _runner.register_in_process_factory("click_goal_router", crate::lcm_dataflow::click_goal_owner);

    #[cfg(feature = "simple-2d")]
    _runner
        .register_in_process_factory("simple_planner", crate::lcm_dataflow::simple_planner_owner);

    #[cfg(feature = "replanning-astar")]
    _runner.register_in_process_factory("astar_planner", crate::lcm_dataflow::astar_planner_owner);

    #[cfg(any(
        feature = "basic-path-follower",
        feature = "simple-2d",
        feature = "replanning-astar",
        feature = "mls"
    ))]
    _runner.register_in_process_factory(
        "basic_path_follower",
        crate::lcm_dataflow::basic_path_follower_owner,
    );

    #[cfg(feature = "movement-manager")]
    _runner.register_in_process_factory(
        "movement_manager",
        crate::lcm_dataflow::movement_manager_owner,
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
}

#[cfg(all(
    test,
    any(feature = "simple-2d", feature = "replanning-astar", feature = "mls")
))]
mod tests {
    use super::*;
    use crate::{NavStackBuilder, NavStackPreset, NavStackProfile};

    #[cfg(feature = "simple-2d")]
    #[tokio::test]
    async fn simple_profile_owns_starts_and_stops_every_in_process_node() {
        let mut runner = NavStackBuilder::new(NavStackProfile::Simple2D)
            .preset(NavStackPreset::Simple)
            .build_runner()
            .unwrap();
        assert!(runner.unowned_nodes().is_empty());
        runner.spawn_all().await.unwrap();
        runner.ensure_healthy().unwrap();
        assert_eq!(runner.running_nodes().len(), 5);
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

    #[cfg(feature = "mls")]
    #[test]
    fn mls_profile_has_native_worker_and_concrete_rust_owners() {
        let runner = NavStackBuilder::new(NavStackProfile::Mls3D)
            .build_runner()
            .unwrap();
        assert!(runner.unowned_nodes().is_empty());
        assert_eq!(runner.native_bridge_ids(), vec!["mls_planner"]);
    }
}
