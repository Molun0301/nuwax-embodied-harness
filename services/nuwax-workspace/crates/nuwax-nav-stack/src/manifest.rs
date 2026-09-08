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

//! Explicit orchestration contracts for navigation profiles.
//!
//! A manifest identifies only the outputs that are canonical for an
//! orchestration slot. Other diagnostic or intermediate graph outputs remain
//! ordinary outputs and are deliberately not inferred into this contract.

use std::collections::BTreeSet;

use crate::graph::ModuleGraph;
use crate::profile::NavStackProfile;
use crate::topics::NavTopicConfig;

/// Whether a profile may enter the production guard-backed execution path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProfileActivation {
    /// The profile is wired to the production guard-backed runtime owners.
    ProductionGuarded,
    /// The profile remains available for graph inspection but not execution.
    InspectionOnly {
        /// Concrete condition preventing production activation.
        blocker: &'static str,
    },
}

impl ProfileActivation {
    /// Return whether production execution is admitted.
    #[must_use]
    pub fn is_production_guarded(self) -> bool {
        matches!(self, Self::ProductionGuarded)
    }

    /// Return the production blocker for inspection-only profiles.
    #[must_use]
    pub fn blocker(self) -> Option<&'static str> {
        match self {
            Self::ProductionGuarded => None,
            Self::InspectionOnly { blocker } => Some(blocker),
        }
    }
}

/// A named orchestration slot and its sole canonical publisher.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanonicalSlot {
    /// Stable slot name within this profile.
    pub name: &'static str,
    /// Graph node that owns this slot.
    pub writer_node: &'static str,
    /// Resolved canonical output topic.
    pub canonical_topic: String,
}

impl CanonicalSlot {
    fn new(name: &'static str, writer_node: &'static str, canonical_topic: &str) -> Self {
        Self {
            name,
            writer_node,
            canonical_topic: canonical_topic.to_owned(),
        }
    }
}

/// Per-profile orchestration and activation contract.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProfileManifest {
    /// Profile described by this manifest.
    pub profile: NavStackProfile,
    /// Production activation state.
    pub activation: ProfileActivation,
    /// Explicit canonical orchestration slots.
    pub slots: Vec<CanonicalSlot>,
}

impl ProfileManifest {
    /// Build the manifest for a profile with topics resolved from `topics`.
    #[must_use]
    pub fn for_profile(profile: NavStackProfile, topics: &NavTopicConfig) -> Self {
        let slot = |name, writer, key, fallback| {
            CanonicalSlot::new(name, writer, topics.get_or(key, fallback))
        };

        let (activation, slots) = match profile {
            NavStackProfile::Simple2D => (
                ProfileActivation::ProductionGuarded,
                vec![
                    slot("map-pose", "map_pose_adapter", "map_pose", "/nav/map_pose"),
                    slot("goal-path", "simple_planner", "goal_path", "goal_path"),
                    slot(
                        "guarded-command",
                        "guarded_nav_scheduler",
                        "cmd_vel",
                        "cmd_vel",
                    ),
                    slot(
                        "navigation-state",
                        "guarded_nav_scheduler",
                        "nav_state",
                        "/nav/state",
                    ),
                ],
            ),
            NavStackProfile::ReplanningAStar => (
                ProfileActivation::ProductionGuarded,
                vec![
                    slot("map-pose", "map_pose_adapter", "map_pose", "/nav/map_pose"),
                    slot("goal-path", "astar_planner", "goal_path", "goal_path"),
                    slot(
                        "guarded-command",
                        "guarded_nav_scheduler",
                        "cmd_vel",
                        "cmd_vel",
                    ),
                    slot(
                        "navigation-state",
                        "guarded_nav_scheduler",
                        "nav_state",
                        "/nav/state",
                    ),
                ],
            ),
            NavStackProfile::CmuNative => (
                ProfileActivation::InspectionOnly {
                    blocker: "native CMU nodes do not have guard-backed production runtime owners",
                },
                vec![
                    slot("terrain-map", "terrain_analysis", "terrain_map", "terrain_map"),
                    slot(
                        "extended-terrain-map",
                        "terrain_map_ext",
                        "terrain_map_ext",
                        "terrain_map_ext",
                    ),
                    slot(
                        "goal-path",
                        "native_planner_path_adapter",
                        "goal_path",
                        "goal_path",
                    ),
                    slot("local-path", "local_planner", "path", "path"),
                    slot(
                        "navigation-command",
                        "path_follower",
                        "nav_cmd_vel",
                        "/nav/cmd_vel",
                    ),
                    slot("command", "movement_manager", "cmd_vel", "cmd_vel"),
                ],
            ),
            NavStackProfile::ExplorationNative => (
                ProfileActivation::InspectionOnly {
                    blocker: "exploration and native CMU nodes do not have guard-backed production runtime owners",
                },
                vec![
                    slot("terrain-map", "terrain_analysis", "terrain_map", "terrain_map"),
                    slot(
                        "extended-terrain-map",
                        "terrain_map_ext",
                        "terrain_map_ext",
                        "terrain_map_ext",
                    ),
                    CanonicalSlot::new(
                        "exploration-goal",
                        "wavefront_explorer",
                        crate::profiles::navigation_goal_topic(topics),
                    ),
                    slot(
                        "goal-path",
                        "native_planner_path_adapter",
                        "goal_path",
                        "goal_path",
                    ),
                    slot("local-path", "local_planner", "path", "path"),
                    slot(
                        "navigation-command",
                        "path_follower",
                        "nav_cmd_vel",
                        "/nav/cmd_vel",
                    ),
                    slot("command", "movement_manager", "cmd_vel", "cmd_vel"),
                ],
            ),
            NavStackProfile::Mls3D => (
                ProfileActivation::ProductionGuarded,
                vec![
                    slot("map-pose", "map_pose_adapter", "map_pose", "/nav/map_pose"),
                    slot("goal-path", "mls_planner", "goal_path", "goal_path"),
                    slot("guarded-command", "surface_nav_scheduler", "cmd_vel", "cmd_vel"),
                    slot("navigation-state", "surface_nav_scheduler", "nav_state", "/nav/state"),
                ],
            ),
        };

        Self {
            profile,
            activation,
            slots,
        }
    }

    /// Validate canonical slot ownership against a built graph.
    pub fn validate_graph(&self, graph: &ModuleGraph) -> Result<(), String> {
        if graph.profile != self.profile.as_str() {
            return Err(format!(
                "manifest profile `{}` does not match graph profile `{}`",
                self.profile.as_str(),
                graph.profile
            ));
        }

        let mut slot_names = BTreeSet::new();
        for slot in &self.slots {
            if !slot_names.insert(slot.name) {
                return Err(format!("duplicate manifest slot name `{}`", slot.name));
            }

            let writer = graph.find(slot.writer_node).ok_or_else(|| {
                format!(
                    "manifest slot `{}` references missing writer node `{}`",
                    slot.name, slot.writer_node
                )
            })?;
            if !writer
                .outputs
                .iter()
                .any(|output| output == &slot.canonical_topic)
            {
                return Err(format!(
                    "manifest slot `{}` writer `{}` does not publish canonical output `{}`",
                    slot.name, slot.writer_node, slot.canonical_topic
                ));
            }

            let publishers = graph
                .nodes()
                .iter()
                .filter(|node| {
                    node.outputs
                        .iter()
                        .any(|output| output == &slot.canonical_topic)
                })
                .count();
            if publishers != 1 {
                return Err(format!(
                    "manifest slot `{}` canonical topic `{}` has {publishers} publishers; expected exactly one",
                    slot.name, slot.canonical_topic
                ));
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::features::NavFeatureFlags;
    use crate::graph::{ModuleKind, ModuleNode};
    use crate::profiles::build_graph;

    #[test]
    fn every_profile_has_a_valid_explicit_manifest() {
        let topics = NavTopicConfig::empty();
        for profile in NavStackProfile::all() {
            let manifest = ProfileManifest::for_profile(*profile, &topics);
            let graph = build_graph(*profile, &topics, NavFeatureFlags::none());
            assert!(!manifest.slots.is_empty(), "{profile} has no slots");
            manifest.validate_graph(&graph).expect("valid manifest");
        }
    }

    #[test]
    fn activation_is_guarded_for_simple_astar_and_surface() {
        for profile in [
            NavStackProfile::Simple2D,
            NavStackProfile::ReplanningAStar,
            NavStackProfile::Mls3D,
        ] {
            let activation =
                ProfileManifest::for_profile(profile, &NavTopicConfig::empty()).activation;
            assert!(activation.is_production_guarded(), "{profile}");
            assert_eq!(activation.blocker(), None);
        }

        for profile in [
            NavStackProfile::CmuNative,
            NavStackProfile::ExplorationNative,
        ] {
            let activation =
                ProfileManifest::for_profile(profile, &NavTopicConfig::empty()).activation;
            assert!(!activation.is_production_guarded(), "{profile}");
            assert!(!activation.blocker().expect("concrete blocker").is_empty());
        }
    }

    #[test]
    fn validation_rejects_missing_writer_and_missing_output() {
        let topics = NavTopicConfig::empty();
        let manifest = ProfileManifest::for_profile(NavStackProfile::Simple2D, &topics);
        let mut graph = build_graph(NavStackProfile::Simple2D, &topics, NavFeatureFlags::none());

        graph.nodes.retain(|node| node.id != "map_pose_adapter");
        assert!(manifest
            .validate_graph(&graph)
            .unwrap_err()
            .contains("missing writer node"));

        let mut graph = build_graph(NavStackProfile::Simple2D, &topics, NavFeatureFlags::none());
        graph
            .nodes
            .iter_mut()
            .find(|node| node.id == "simple_planner")
            .expect("simple planner")
            .outputs
            .retain(|output| output != "goal_path");
        assert!(manifest
            .validate_graph(&graph)
            .unwrap_err()
            .contains("does not publish canonical output"));
    }

    #[test]
    fn validation_rejects_multiple_canonical_writers() {
        let topics = NavTopicConfig::empty();
        let manifest = ProfileManifest::for_profile(NavStackProfile::Simple2D, &topics);
        let mut graph = build_graph(NavStackProfile::Simple2D, &topics, NavFeatureFlags::none());
        graph.push_node(
            ModuleNode::new("second_path_writer", "Second", ModuleKind::RustAlgorithm)
                .add_output("goal_path"),
        );
        assert!(manifest
            .validate_graph(&graph)
            .unwrap_err()
            .contains("expected exactly one"));
    }

    #[test]
    fn validation_rejects_duplicate_slot_names() {
        let topics = NavTopicConfig::empty();
        let mut manifest = ProfileManifest::for_profile(NavStackProfile::Simple2D, &topics);
        manifest.slots[1].name = manifest.slots[0].name;
        let graph = build_graph(NavStackProfile::Simple2D, &topics, NavFeatureFlags::none());
        assert!(manifest
            .validate_graph(&graph)
            .unwrap_err()
            .contains("duplicate manifest slot name"));
    }
}
