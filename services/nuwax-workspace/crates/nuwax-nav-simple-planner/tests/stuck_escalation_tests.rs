//! Stuck 升级行为测试（对应 Python TestStuckEscalation）。

use approx::assert_relative_eq;
use nuwax_nav_simple_planner::{progress_tick, StuckState};

fn initial_state(inflation_radius: f64) -> StuckState {
    StuckState {
        ref_goal_dist: f64::INFINITY,
        last_progress_time: 0.0,
        effective_inflation: inflation_radius,
    }
}

fn step(
    state: StuckState,
    dist: f64,
    now: f64,
    progress_epsilon: f64,
    stuck_seconds: f64,
    stuck_shrink_factor: f64,
    stuck_min_inflation: f64,
) -> StuckState {
    let (s, _) = progress_tick(
        state,
        dist,
        now,
        progress_epsilon,
        stuck_seconds,
        stuck_shrink_factor,
        stuck_min_inflation,
    );
    s
}

#[test]
fn progress_refreshes_last_time() {
    let mut state = initial_state(0.4);
    state = step(state, 10.0, 0.0, 0.25, 5.0, 0.5, 0.0);
    assert_relative_eq!(state.ref_goal_dist, 10.0, epsilon = 1e-9);
    state = step(state, 9.0, 1.0, 0.25, 5.0, 0.5, 0.0);
    assert_relative_eq!(state.last_progress_time, 1.0, epsilon = 1e-9);
    assert_relative_eq!(state.ref_goal_dist, 9.0, epsilon = 1e-9);
    assert_relative_eq!(state.effective_inflation, 0.4, epsilon = 1e-9);
}

#[test]
fn tiny_progress_does_not_count() {
    let mut state = initial_state(0.4);
    state = step(state, 10.0, 0.0, 0.25, 5.0, 0.5, 0.0);
    state = step(state, 9.9, 1.0, 0.25, 5.0, 0.5, 0.0);
    assert_relative_eq!(state.ref_goal_dist, 10.0, epsilon = 1e-9);
    assert_relative_eq!(state.last_progress_time, 0.0, epsilon = 1e-9);
}

#[test]
fn escalation_shrinks_inflation() {
    let mut state = initial_state(0.4);
    state = step(state, 10.0, 0.0, 0.25, 5.0, 0.5, 0.0);
    state = step(state, 10.0, 4.9, 0.25, 5.0, 0.5, 0.0);
    assert_relative_eq!(state.effective_inflation, 0.4, epsilon = 1e-9);
    state = step(state, 10.0, 5.0, 0.25, 5.0, 0.5, 0.0);
    assert_relative_eq!(state.effective_inflation, 0.2, epsilon = 1e-9);
    state = step(state, 10.0, 10.0, 0.25, 5.0, 0.5, 0.0);
    assert_relative_eq!(state.effective_inflation, 0.1, epsilon = 1e-9);
}

#[test]
fn escalation_respects_floor() {
    let mut state = initial_state(0.4);
    state = step(state, 10.0, 0.0, 0.25, 1.0, 0.5, 0.2);
    state = step(state, 10.0, 1.0, 0.25, 1.0, 0.5, 0.2);
    assert_relative_eq!(state.effective_inflation, 0.2, epsilon = 1e-9);
    state = step(state, 10.0, 2.0, 0.25, 1.0, 0.5, 0.2);
    assert_relative_eq!(state.effective_inflation, 0.2, epsilon = 1e-9);
    state = step(state, 10.0, 3.0, 0.25, 1.0, 0.5, 0.2);
    assert_relative_eq!(state.effective_inflation, 0.2, epsilon = 1e-9);
}

#[test]
fn progress_after_escalation_keeps_shrunk_inflation() {
    let mut state = initial_state(0.4);
    state = step(state, 10.0, 0.0, 0.25, 1.0, 0.5, 0.0);
    state = step(state, 10.0, 1.0, 0.25, 1.0, 0.5, 0.0);
    assert_relative_eq!(state.effective_inflation, 0.2, epsilon = 1e-9);
    state = step(state, 9.0, 1.5, 0.25, 1.0, 0.5, 0.0);
    assert_relative_eq!(state.effective_inflation, 0.2, epsilon = 1e-9);
}
