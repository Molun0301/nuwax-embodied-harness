// Copyright 2026 Nuwax Inc.
// SPDX-License-Identifier: Apache-2.0

//! Deterministic synthetic scenarios for MLS planner evaluation.

const MAX_SCENARIO_POINTS: usize = 1_000_000;

/// One executable planner scenario.
#[derive(Clone, Debug, PartialEq)]
pub struct PlannerScenario {
    /// Stable scenario name.
    pub name: String,
    /// World-frame occupied points.
    pub global_map: Vec<(f32, f32, f32)>,
    /// Start on or immediately above the floor surface.
    pub start: (f32, f32, f32),
    /// Goal on or immediately above the floor surface.
    pub goal: (f32, f32, f32),
    /// Whether the scenario is expected to have a route.
    pub expect_path: bool,
}

/// Sample an axis-aligned flat floor.
#[must_use]
pub fn flat_floor(
    x_min: f32,
    x_max: f32,
    y_min: f32,
    y_max: f32,
    spacing: f32,
) -> Vec<(f32, f32, f32)> {
    if !spacing.is_finite()
        || spacing <= 0.0
        || !x_min.is_finite()
        || !x_max.is_finite()
        || !y_min.is_finite()
        || !y_max.is_finite()
        || x_min > x_max
        || y_min > y_max
    {
        return Vec::new();
    }
    let x_steps = ((f64::from(x_max) - f64::from(x_min) + f64::from(spacing) * 0.25)
        / f64::from(spacing))
    .floor()
        + 1.0;
    let y_steps = ((f64::from(y_max) - f64::from(y_min) + f64::from(spacing) * 0.25)
        / f64::from(spacing))
    .floor()
        + 1.0;
    if x_steps > MAX_SCENARIO_POINTS as f64 || y_steps > MAX_SCENARIO_POINTS as f64 {
        return Vec::new();
    }
    let (x_steps, y_steps) = (x_steps as usize, y_steps as usize);
    let x_iteration_limit = x_steps + 2;
    let y_iteration_limit = y_steps + 2;
    if x_iteration_limit
        .checked_mul(y_iteration_limit)
        .is_none_or(|count| count > MAX_SCENARIO_POINTS)
    {
        return Vec::new();
    }
    let mut points = Vec::with_capacity(x_steps * y_steps);
    let x_limit = x_max + spacing * 0.25;
    let y_limit = y_max + spacing * 0.25;
    let mut x = x_min;
    for _ in 0..x_iteration_limit {
        if x > x_limit {
            break;
        }
        let mut y = y_min;
        for _ in 0..y_iteration_limit {
            if y > y_limit {
                break;
            }
            points.push((x, y, 0.0));
            let next_y = y + spacing;
            if !next_y.is_finite() || next_y <= y {
                return Vec::new();
            }
            y = next_y;
        }
        let next_x = x + spacing;
        if !next_x.is_finite() || next_x <= x {
            return Vec::new();
        }
        x = next_x;
    }
    points
}

/// Sample a vertical wall box between two XY endpoints.
#[must_use]
#[allow(clippy::too_many_arguments)]
pub fn wall(
    x0: f32,
    y0: f32,
    x1: f32,
    y1: f32,
    spacing: f32,
    height: f32,
    thickness: f32,
) -> Vec<(f32, f32, f32)> {
    let parameters = [x0, y0, x1, y1, spacing, height, thickness];
    if !parameters.into_iter().all(f32::is_finite)
        || spacing <= 0.0
        || height <= 0.0
        || thickness <= 0.0
    {
        return Vec::new();
    }
    let dx = x1 - x0;
    let dy = y1 - y0;
    let length = dx.hypot(dy);
    if !length.is_finite() || length == 0.0 {
        return Vec::new();
    }
    let perpendicular = (-dy / length, dx / length);
    let step_estimates = [
        (f64::from(length) / f64::from(spacing)).ceil().max(1.0),
        (f64::from(thickness) / f64::from(spacing)).ceil().max(1.0),
        (f64::from(height) / f64::from(spacing)).ceil().max(1.0),
    ];
    if step_estimates
        .into_iter()
        .any(|steps| !steps.is_finite() || steps >= MAX_SCENARIO_POINTS as f64)
    {
        return Vec::new();
    }
    let [along_steps, thick_steps, height_steps] = step_estimates.map(|steps| steps as usize);
    if (along_steps + 1)
        .checked_mul(thick_steps + 1)
        .and_then(|count| count.checked_mul(height_steps + 1))
        .is_none_or(|count| count > MAX_SCENARIO_POINTS)
    {
        return Vec::new();
    }
    let mut points = Vec::new();
    for along_idx in 0..=along_steps {
        let along = along_idx as f32 / along_steps as f32;
        for thick_idx in 0..=thick_steps {
            let offset = -thickness * 0.5 + thickness * thick_idx as f32 / thick_steps as f32;
            for height_idx in 0..=height_steps {
                let z = height * height_idx as f32 / height_steps as f32;
                points.push((
                    x0 + along * dx + offset * perpendicular.0,
                    y0 + along * dy + offset * perpendicular.1,
                    z,
                ));
            }
        }
    }
    points
}

/// Open floor with a direct route.
#[must_use]
pub fn empty_floor() -> PlannerScenario {
    PlannerScenario {
        name: "empty_floor".into(),
        global_map: flat_floor(-2.0, 8.0, -3.0, 3.0, 0.2),
        start: (-1.0, 0.0, 0.0),
        goal: (7.0, 0.0, 0.0),
        expect_path: true,
    }
}

/// Floor split completely by a wall.
#[must_use]
pub fn blocked_wall() -> PlannerScenario {
    let mut global_map = flat_floor(-2.0, 8.0, -2.0, 2.0, 0.2);
    global_map.extend(wall(3.0, -2.5, 3.0, 2.5, 0.1, 2.0, 0.4));
    PlannerScenario {
        name: "blocked_wall".into(),
        global_map,
        start: (-1.0, 0.0, 0.0),
        goal: (7.0, 0.0, 0.0),
        expect_path: false,
    }
}

/// Two rooms separated by a wall with a central doorway.
#[must_use]
pub fn two_rooms_one_door() -> PlannerScenario {
    let mut global_map = flat_floor(-2.0, 8.0, -3.0, 3.0, 0.2);
    global_map.extend(wall(3.0, -3.0, 3.0, -0.8, 0.1, 2.0, 0.4));
    global_map.extend(wall(3.0, 0.8, 3.0, 3.0, 0.1, 2.0, 0.4));
    PlannerScenario {
        name: "two_rooms_one_door".into(),
        global_map,
        start: (-1.0, 0.0, 0.0),
        goal: (7.0, 0.0, 0.0),
        expect_path: true,
    }
}

/// Built-in scenario sequence corresponding to the Dimos evaluator fixtures.
#[must_use]
pub fn default_scenarios() -> Vec<PlannerScenario> {
    vec![empty_floor(), blocked_wall(), two_rooms_one_door()]
}

/// Add a caller-provided voxelized mesh scenario to the deterministic suite.
#[must_use]
pub fn with_mesh_scenario(
    mut scenarios: Vec<PlannerScenario>,
    name: impl Into<String>,
    points: Vec<(f32, f32, f32)>,
    start: (f32, f32, f32),
    goal: (f32, f32, f32),
    expect_path: bool,
) -> Vec<PlannerScenario> {
    scenarios.push(PlannerScenario {
        name: name.into(),
        global_map: points,
        start,
        goal,
        expect_path,
    });
    scenarios
}

#[cfg(test)]
mod tests {
    use super::{flat_floor, wall};

    #[test]
    fn flat_floor_rejects_invalid_spacing_and_bounds() {
        for spacing in [0.0, -0.1, f32::NAN, f32::INFINITY] {
            assert!(flat_floor(0.0, 1.0, 0.0, 1.0, spacing).is_empty());
        }
        assert!(flat_floor(f32::NAN, 1.0, 0.0, 1.0, 0.1).is_empty());
        assert!(flat_floor(1.0, 0.0, 0.0, 1.0, 0.1).is_empty());
        assert!(flat_floor(0.0, 1_000.0, 0.0, 1_000.0, 0.001).is_empty());
    }

    #[test]
    fn wall_rejects_invalid_and_excessive_work() {
        assert!(wall(0.0, 0.0, 1.0, 0.0, 0.1, f32::NAN, 0.1).is_empty());
        assert!(wall(0.0, 0.0, 1.0, 0.0, 0.1, 1.0, 0.0).is_empty());
        assert!(wall(0.0, 0.0, 1_000.0, 0.0, 0.001, 1.0, 1.0).is_empty());
    }
}
