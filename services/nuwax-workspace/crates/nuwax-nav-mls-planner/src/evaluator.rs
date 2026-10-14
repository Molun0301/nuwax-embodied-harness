// Copyright 2026 Nuwax Inc.
// SPDX-License-Identifier: Apache-2.0

//! Synchronous black-box evaluator for fixed planner scenarios.

use crate::mls_planner::{Config, Planner};
use crate::scenarios::PlannerScenario;

/// Result for one evaluated scenario.
#[derive(Clone, Debug, PartialEq)]
pub struct ScenarioResult {
    /// Scenario name.
    pub name: String,
    /// Expected path presence.
    pub expected_path: bool,
    /// Actual path presence.
    pub got_path: bool,
    /// Whether expectation and result agree.
    pub passed: bool,
    /// Number of returned waypoints.
    pub waypoint_count: usize,
    /// Polyline length in meters.
    pub path_length_m: f32,
}

/// Aggregate evaluation summary.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct EvaluationSummary {
    /// Per-scenario results in input order.
    pub results: Vec<ScenarioResult>,
}

impl EvaluationSummary {
    /// Number of passing scenarios.
    #[must_use]
    pub fn passed(&self) -> usize {
        self.results.iter().filter(|result| result.passed).count()
    }

    /// Whether every scenario passed.
    #[must_use]
    pub fn all_passed(&self) -> bool {
        self.passed() == self.results.len()
    }
}

/// Evaluates scenarios through fresh planner instances.
pub struct Evaluator {
    config: Config,
}

impl Evaluator {
    /// Construct with the planner configuration under test.
    #[must_use]
    pub fn new(config: Config) -> Self {
        Self { config }
    }

    /// Run every scenario through an isolated planner state.
    #[must_use]
    pub fn run(&self, scenarios: &[PlannerScenario]) -> EvaluationSummary {
        let results = scenarios
            .iter()
            .map(|scenario| self.run_one(scenario))
            .collect();
        EvaluationSummary { results }
    }

    /// Evaluate one scenario.
    #[must_use]
    pub fn run_one(&self, scenario: &PlannerScenario) -> ScenarioResult {
        let mut planner = Planner::default();
        planner.update_global_map(&scenario.global_map, &self.config);
        let path = planner
            .plan(scenario.start, scenario.goal, &self.config)
            .unwrap_or_default();
        let got_path = !path.is_empty();
        ScenarioResult {
            name: scenario.name.clone(),
            expected_path: scenario.expect_path,
            got_path,
            passed: got_path == scenario.expect_path,
            waypoint_count: path.len(),
            path_length_m: path_length(&path),
        }
    }
}

fn path_length(path: &[(f32, f32, f32)]) -> f32 {
    path.windows(2)
        .map(|pair| {
            let dx = pair[1].0 - pair[0].0;
            let dy = pair[1].1 - pair[0].1;
            let dz = pair[1].2 - pair[0].2;
            (dx * dx + dy * dy + dz * dz).sqrt()
        })
        .sum()
}
