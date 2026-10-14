// Copyright 2026 Nuwax Inc.
// SPDX-License-Identifier: Apache-2.0

//! Production-profile graph conformance for the canonical Guard route.

use crate::graph::ModuleGraph;
use crate::profile::NavStackProfile;
use crate::topics::NavTopicConfig;

/// One graph contract violation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ConformanceIssue {
    /// Stable check identifier.
    pub check: &'static str,
    /// Human-readable failure.
    pub detail: String,
}

/// Result of checking a profile graph.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ConformanceReport {
    /// All detected violations.
    pub issues: Vec<ConformanceIssue>,
}

impl ConformanceReport {
    /// Whether the graph satisfies every guarded-route contract.
    #[must_use]
    pub fn is_conformant(&self) -> bool {
        self.issues.is_empty()
    }
}

/// Validate the production Simple2D or Replanning A* topology.
#[must_use]
pub fn validate_guarded_profile(
    profile: NavStackProfile,
    graph: &ModuleGraph,
    topics: &NavTopicConfig,
) -> ConformanceReport {
    let planner = match profile {
        NavStackProfile::Simple2D => "simple_planner",
        NavStackProfile::ReplanningAStar => "astar_planner",
        _ => {
            return ConformanceReport {
                issues: vec![ConformanceIssue {
                    check: "profile",
                    detail: format!("{} is not a guarded production profile", profile.as_str()),
                }],
            };
        }
    };

    let mut report = ConformanceReport::default();
    require_node(graph, "map_pose_adapter", &mut report);
    require_node(graph, planner, &mut report);
    require_node(graph, "guarded_nav_scheduler", &mut report);

    let goal = topics.get_or("goal_pose", "/nav/goal");
    let cancel = topics.get_or("nav_cancel", "/nav/cancel");
    let state = topics.get_or("nav_state", "/nav/state");
    let path = topics.get_or("goal_path", "goal_path");
    let stop = topics.get_or("stop_movement", "stop_movement");
    let command = topics.get_or("cmd_vel", "cmd_vel");

    require_input(graph, planner, goal, "goal-to-planner", &mut report);
    require_input(
        graph,
        "guarded_nav_scheduler",
        goal,
        "goal-to-guard",
        &mut report,
    );
    require_input(
        graph,
        "guarded_nav_scheduler",
        cancel,
        "cancel-to-guard",
        &mut report,
    );
    require_output(
        graph,
        "guarded_nav_scheduler",
        state,
        "guard-state",
        &mut report,
    );
    require_output(
        graph,
        "guarded_nav_scheduler",
        command,
        "guard-command",
        &mut report,
    );
    require_edge(
        graph,
        planner,
        "guarded_nav_scheduler",
        path,
        "path-to-guard",
        &mut report,
    );
    require_edge(
        graph,
        "guarded_nav_scheduler",
        planner,
        stop,
        "guard-stop-feedback",
        &mut report,
    );

    for phantom in [
        "click_goal_router",
        "patrolling",
        "basic_path_follower",
        "movement_manager",
    ] {
        if graph.find(phantom).is_some() {
            report.issues.push(ConformanceIssue {
                check: "canonical-owner",
                detail: format!("unexpected node {phantom} bypasses the guarded profile owner"),
            });
        }
    }
    report
}

fn require_node(graph: &ModuleGraph, node: &'static str, report: &mut ConformanceReport) {
    if graph.find(node).is_none() {
        report.issues.push(ConformanceIssue {
            check: "node",
            detail: format!("missing required node {node}"),
        });
    }
}

fn require_input(
    graph: &ModuleGraph,
    node: &str,
    topic: &str,
    check: &'static str,
    report: &mut ConformanceReport,
) {
    if !graph
        .find(node)
        .is_some_and(|candidate| candidate.inputs.iter().any(|input| input == topic))
    {
        report.issues.push(ConformanceIssue {
            check,
            detail: format!("node {node} does not consume {topic}"),
        });
    }
}

fn require_output(
    graph: &ModuleGraph,
    node: &str,
    topic: &str,
    check: &'static str,
    report: &mut ConformanceReport,
) {
    if !graph
        .find(node)
        .is_some_and(|candidate| candidate.outputs.iter().any(|output| output == topic))
    {
        report.issues.push(ConformanceIssue {
            check,
            detail: format!("node {node} does not publish {topic}"),
        });
    }
}

fn require_edge(
    graph: &ModuleGraph,
    from: &str,
    to: &str,
    topic: &str,
    check: &'static str,
    report: &mut ConformanceReport,
) {
    if !graph
        .edges()
        .iter()
        .any(|edge| edge.from == from && edge.to == to && edge.topic == topic)
    {
        report.issues.push(ConformanceIssue {
            check,
            detail: format!("missing edge {from} -> {to} on {topic}"),
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::{ModuleKind, ModuleNode};
    use crate::{NavFeatureFlags, NavStackBuilder};
    use nuwax_nav_test_fixtures::recorded_events::guarded_route_fixture;

    #[test]
    fn production_profiles_have_complete_guard_route_and_only_real_nodes() {
        let topics = NavTopicConfig::geometry_runtime_defaults();
        for profile in [NavStackProfile::Simple2D, NavStackProfile::ReplanningAStar] {
            let graph = NavStackBuilder::new(profile)
                .topics(topics.clone())
                .features(NavFeatureFlags::none())
                .build()
                .expect("build profile");
            let report = validate_guarded_profile(profile, &graph, &topics);
            assert!(report.is_conformant(), "{profile}: {:?}", report.issues);
            assert_eq!(
                graph.nodes().len(),
                3,
                "{profile} instantiated phantom nodes"
            );
        }
    }

    #[test]
    fn raw_basic_follower_is_rejected_as_a_canonical_owner_bypass() {
        let topics = NavTopicConfig::geometry_runtime_defaults();
        let mut graph = NavStackBuilder::new(NavStackProfile::Simple2D)
            .topics(topics.clone())
            .build()
            .expect("build profile");
        graph.push_node(ModuleNode::new(
            "basic_path_follower",
            "Raw basic path follower",
            ModuleKind::Coordinator,
        ));

        let report = validate_guarded_profile(NavStackProfile::Simple2D, &graph, &topics);
        assert!(report.issues.iter().any(|issue| {
            issue.check == "canonical-owner" && issue.detail.contains("basic_path_follower")
        }));
    }

    #[test]
    fn missing_guard_feedback_edge_is_reported() {
        let topics = NavTopicConfig::geometry_runtime_defaults();
        let mut graph = NavStackBuilder::new(NavStackProfile::Simple2D)
            .topics(topics.clone())
            .build()
            .expect("build profile");
        graph
            .edges
            .retain(|edge| edge.from != "guarded_nav_scheduler");
        let report = validate_guarded_profile(NavStackProfile::Simple2D, &graph, &topics);
        assert!(report
            .issues
            .iter()
            .any(|issue| issue.check == "guard-stop-feedback"));
    }

    #[test]
    fn typed_recording_replays_guarded_behavior_and_conforms_to_runtime_channels() {
        let fixture = guarded_route_fixture().expect("load typed recording");
        let replay = fixture
            .replay_guarded_route()
            .expect("replay one ordered guarded route");
        assert_eq!(replay.session_id, "session-7");
        assert_eq!(replay.goal_generation, 7);
        assert_eq!(replay.cancel_generation, 1);
        assert_eq!(replay.movement_commands, 1);
        assert!(replay.goal_sequence < replay.path_sequence);
        assert!(replay.path_sequence < replay.moving_sequence);
        assert!(replay.moving_sequence < replay.cancel_sequence);
        assert!(replay.cancel_sequence < replay.cancelled_sequence);
        let recorded_channels: Vec<_> = fixture
            .events()
            .iter()
            .map(|event| event.channel.as_str())
            .collect();
        let topics = NavTopicConfig::geometry_runtime_defaults();
        for (port, fallback) in [
            ("goal_pose", "/nav/goal"),
            ("goal_path", "/nav/path"),
            ("cmd_vel", "/cmd_vel"),
            ("nav_state", "/nav/state"),
            ("nav_cancel", "/nav/cancel"),
        ] {
            let channel = topics.get_or(port, fallback);
            assert!(
                recorded_channels.contains(&channel),
                "recording does not exercise {port} on {channel}"
            );
        }
    }
}
