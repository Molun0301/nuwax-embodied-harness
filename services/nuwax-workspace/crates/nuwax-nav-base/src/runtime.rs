//! Executable navigation lifecycle owner.
//!
//! This module owns state transitions, cancellation generations and invocation
//! of planner/controller implementations. It also exposes the four operations
//! from DimOS `NavigationInterface` as a compatibility contract.

use nuwax_nav_types_messages::{
    ControllerCommand, NavigationGoal, NavigationStatus, OccupancyGrid2D, Path2D, PlannerRequest,
    PlannerResult, PoseStamped, Timestamp,
};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{ControllerInput, GlobalPlanner, NavError, PathController};

/// DimOS-compatible public navigation state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NavigationState {
    /// No goal is being followed.
    Idle,
    /// A goal has an accepted path and is being followed.
    FollowingPath,
    /// Planning failed or the controller reported a recoverable blockage.
    Recovery,
}

/// Detailed lifecycle state owned by [`NavigationRuntime`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NavigationLifecycleState {
    /// Runtime is stopped and rejects goals.
    Stopped,
    /// Runtime is ready without an active goal.
    Idle,
    /// A planner result is pending.
    Planning,
    /// A path is being followed.
    FollowingPath,
    /// The current goal needs recovery or replanning.
    Recovery,
    /// The current goal was reached.
    Reached,
    /// The current goal was cancelled.
    Cancelled,
    /// The current goal failed with a non-recoverable error.
    Failed,
}

/// Monotonic generation associated with planner and controller work.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct NavigationEpoch(pub u64);

/// Typed lifecycle failure.
#[derive(Debug, Error)]
pub enum NavigationRuntimeError {
    /// Operation is invalid in the current lifecycle state.
    #[error("operation `{operation}` is invalid while navigation is {state:?}")]
    InvalidTransition {
        /// Attempted operation.
        operation: &'static str,
        /// State at the time of the attempt.
        state: NavigationLifecycleState,
    },
    /// Result belongs to cancelled or superseded work.
    #[error("stale navigation result for epoch {received}; current epoch is {current}")]
    StaleEpoch {
        /// Current epoch.
        current: u64,
        /// Received epoch.
        received: u64,
    },
    /// Goal fields are invalid.
    #[error("invalid navigation goal: {0}")]
    InvalidGoal(&'static str),
    /// Planner or controller failed.
    #[error(transparent)]
    Navigation(#[from] NavError),
}

/// DimOS `base.py`/`navigation_spec.py` operation contract.
pub trait NavigationInterface {
    /// Set a new goal non-blockingly, returning whether it was accepted.
    fn set_goal(&mut self, goal: PoseStamped) -> bool;
    /// Return the DimOS-compatible public state.
    fn get_state(&self) -> NavigationState;
    /// Return whether the most recently active goal was reached.
    fn is_goal_reached(&self) -> bool;
    /// Cancel an active goal, returning false when there is no active goal.
    fn cancel_goal(&mut self) -> bool;
}

/// Executable owner for navigation lifecycle, planner and controller state.
#[derive(Debug)]
pub struct NavigationRuntime {
    state: NavigationLifecycleState,
    epoch: u64,
    active_goal: Option<NavigationGoal>,
    current_path: Option<Path2D>,
    goal_reached: bool,
    last_error: Option<String>,
}

impl Default for NavigationRuntime {
    fn default() -> Self {
        Self {
            state: NavigationLifecycleState::Stopped,
            epoch: 0,
            active_goal: None,
            current_path: None,
            goal_reached: false,
            last_error: None,
        }
    }
}

impl NavigationRuntime {
    /// Construct a stopped runtime.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Start the runtime. Starting an already-started runtime is idempotent.
    pub fn start(&mut self) {
        if self.state == NavigationLifecycleState::Stopped {
            self.state = NavigationLifecycleState::Idle;
            self.last_error = None;
        }
    }

    /// Stop navigation, invalidate in-flight work and clear the active goal.
    pub fn stop(&mut self) {
        self.epoch = self.epoch.wrapping_add(1);
        self.active_goal = None;
        self.current_path = None;
        self.goal_reached = false;
        self.state = NavigationLifecycleState::Stopped;
    }

    /// Submit a typed goal and return the epoch that must accompany its work.
    pub fn submit_goal(
        &mut self,
        goal: NavigationGoal,
    ) -> Result<NavigationEpoch, NavigationRuntimeError> {
        if self.state == NavigationLifecycleState::Stopped {
            return Err(NavigationRuntimeError::InvalidTransition {
                operation: "submit_goal",
                state: self.state,
            });
        }
        if !goal.target.pose.position.x.is_finite()
            || !goal.target.pose.position.y.is_finite()
            || !goal.target.pose.position.z.is_finite()
            || !goal.position_tolerance_m.is_finite()
            || goal.position_tolerance_m < 0.0
            || !goal.yaw_tolerance_rad.is_finite()
            || goal.yaw_tolerance_rad < 0.0
        {
            return Err(NavigationRuntimeError::InvalidGoal(
                "pose and tolerances must be finite; tolerances must be non-negative",
            ));
        }
        self.epoch = self.epoch.wrapping_add(1);
        self.active_goal = Some(goal);
        self.current_path = None;
        self.goal_reached = false;
        self.last_error = None;
        self.state = NavigationLifecycleState::Planning;
        Ok(NavigationEpoch(self.epoch))
    }

    /// Execute a concrete global planner for the current goal.
    pub fn planner_step<P: GlobalPlanner>(
        &mut self,
        epoch: NavigationEpoch,
        planner: &P,
        start: PoseStamped,
        map: OccupancyGrid2D,
    ) -> Result<(), NavigationRuntimeError> {
        self.require_epoch(epoch)?;
        if self.state != NavigationLifecycleState::Planning
            && self.state != NavigationLifecycleState::Recovery
        {
            return Err(NavigationRuntimeError::InvalidTransition {
                operation: "planner_step",
                state: self.state,
            });
        }
        let goal = self
            .active_goal
            .as_ref()
            .ok_or(NavigationRuntimeError::InvalidTransition {
                operation: "planner_step_without_goal",
                state: self.state,
            })?;
        let request = PlannerRequest {
            start,
            goal: goal.target.clone(),
            map,
            options: Default::default(),
        };
        match planner.plan(request) {
            Ok(result) => self.accept_plan(epoch, result),
            Err(error) => {
                self.state = NavigationLifecycleState::Failed;
                self.last_error = Some(error.to_string());
                Err(error.into())
            }
        }
    }

    /// Accept a planner result if it belongs to the current epoch.
    pub fn accept_plan(
        &mut self,
        epoch: NavigationEpoch,
        result: PlannerResult,
    ) -> Result<(), NavigationRuntimeError> {
        self.require_epoch(epoch)?;
        if self.state != NavigationLifecycleState::Planning
            && self.state != NavigationLifecycleState::Recovery
        {
            return Err(NavigationRuntimeError::InvalidTransition {
                operation: "accept_plan",
                state: self.state,
            });
        }
        let PlannerResult {
            path,
            status,
            cost,
            explored_nodes: _,
            failure,
        } = result;
        let path_is_valid = path.as_ref().is_some_and(|value| {
            !value.is_empty()
                && value
                    .points
                    .iter()
                    .all(|point| point.x.is_finite() && point.y.is_finite())
        });
        let path_frame_matches_goal = path
            .as_ref()
            .zip(self.active_goal.as_ref())
            .is_some_and(|(path, goal)| path.frame_id == goal.target.frame_id);
        match status {
            NavigationStatus::Planning
                if failure.is_none()
                    && cost.is_some_and(|value| value.is_finite() && value >= 0.0)
                    && path_is_valid
                    && path_frame_matches_goal =>
            {
                let path = path.expect("validated non-empty planner path");
                self.current_path = Some(path);
                self.last_error = None;
                self.state = NavigationLifecycleState::FollowingPath;
                Ok(())
            }
            NavigationStatus::Planning => {
                self.current_path = None;
                self.state = NavigationLifecycleState::Failed;
                self.last_error = Some(format!(
                    "invalid successful planner result: valid_path={path_is_valid}, frame_matches_goal={path_frame_matches_goal}, cost={cost:?}, failure={failure:?}"
                ));
                Ok(())
            }
            NavigationStatus::Blocked => {
                self.current_path = None;
                self.state = NavigationLifecycleState::Recovery;
                self.last_error = Some(format!(
                    "planner reported Blocked: failure={failure:?}, discarded_path={}, discarded_cost={cost:?}",
                    path.is_some()
                ));
                Ok(())
            }
            NavigationStatus::Cancelled => {
                self.epoch = self.epoch.wrapping_add(1);
                self.active_goal = None;
                self.current_path = None;
                self.goal_reached = false;
                self.state = NavigationLifecycleState::Cancelled;
                self.last_error = Some(format!(
                    "planner reported Cancelled: failure={failure:?}, discarded_path={}, discarded_cost={cost:?}",
                    path.is_some()
                ));
                Ok(())
            }
            NavigationStatus::Reached if failure.is_none() && cost.is_none() => {
                self.active_goal = None;
                self.current_path = None;
                self.goal_reached = true;
                self.state = NavigationLifecycleState::Reached;
                self.last_error = Some(format!(
                    "planner reported Reached: failure={failure:?}, discarded_path={}, discarded_cost={cost:?}",
                    path.is_some()
                ));
                Ok(())
            }
            NavigationStatus::Reached => {
                self.current_path = None;
                self.goal_reached = false;
                self.state = NavigationLifecycleState::Failed;
                self.last_error = Some(format!(
                    "invalid Reached planner result: failure={failure:?}, discarded_path={}, discarded_cost={cost:?}",
                    path.is_some()
                ));
                Ok(())
            }
            NavigationStatus::Error => {
                self.current_path = None;
                self.state = NavigationLifecycleState::Failed;
                self.last_error = Some(format!(
                    "planner reported Error: failure={failure:?}, discarded_path={}, discarded_cost={cost:?}",
                    path.is_some()
                ));
                Ok(())
            }
            NavigationStatus::Idle | NavigationStatus::Moving => {
                self.current_path = None;
                self.state = NavigationLifecycleState::Failed;
                self.last_error = Some(format!(
                    "planner returned invalid status {status:?}: failure={failure:?}, discarded_path={}, discarded_cost={cost:?}",
                    path.is_some()
                ));
                Ok(())
            }
        }
    }

    /// Execute a concrete path controller for the accepted path.
    pub fn controller_step<C: PathController>(
        &mut self,
        epoch: NavigationEpoch,
        controller: &mut C,
        now: Timestamp,
        current_pose: PoseStamped,
        dt_seconds: f64,
        speed_limit: Option<f64>,
    ) -> Result<ControllerCommand, NavigationRuntimeError> {
        self.require_epoch(epoch)?;
        if self.state != NavigationLifecycleState::FollowingPath {
            return Err(NavigationRuntimeError::InvalidTransition {
                operation: "controller_step",
                state: self.state,
            });
        }
        let input = ControllerInput {
            now,
            current_pose,
            path: self.current_path.clone(),
            active_goal: self.active_goal.clone(),
            dt_seconds,
            speed_limit,
            stop_requested: false,
        };
        match controller.compute_command(input) {
            Ok(command) => Ok(command),
            Err(error) => {
                self.state = NavigationLifecycleState::Failed;
                self.last_error = Some(error.to_string());
                Err(error.into())
            }
        }
    }

    /// Mark the current goal as needing recovery.
    pub fn mark_recovery(&mut self, epoch: NavigationEpoch) -> Result<(), NavigationRuntimeError> {
        self.require_epoch(epoch)?;
        if self.active_goal.is_none() {
            return Err(NavigationRuntimeError::InvalidTransition {
                operation: "mark_recovery",
                state: self.state,
            });
        }
        self.current_path = None;
        self.state = NavigationLifecycleState::Recovery;
        Ok(())
    }

    /// Mark the current goal reached.
    pub fn mark_reached(&mut self, epoch: NavigationEpoch) -> Result<(), NavigationRuntimeError> {
        self.require_epoch(epoch)?;
        if self.state != NavigationLifecycleState::FollowingPath {
            return Err(NavigationRuntimeError::InvalidTransition {
                operation: "mark_reached",
                state: self.state,
            });
        }
        self.active_goal = None;
        self.current_path = None;
        self.goal_reached = true;
        self.state = NavigationLifecycleState::Reached;
        Ok(())
    }

    /// Cancel the active goal and invalidate in-flight work.
    pub fn cancel(&mut self) -> bool {
        if self.active_goal.is_none() {
            return false;
        }
        self.epoch = self.epoch.wrapping_add(1);
        self.active_goal = None;
        self.current_path = None;
        self.goal_reached = false;
        self.state = NavigationLifecycleState::Cancelled;
        true
    }

    /// Detailed lifecycle state.
    #[must_use]
    pub fn lifecycle_state(&self) -> NavigationLifecycleState {
        self.state
    }

    /// Current cancellation generation.
    #[must_use]
    pub fn epoch(&self) -> NavigationEpoch {
        NavigationEpoch(self.epoch)
    }

    /// Current path, if planning has succeeded.
    #[must_use]
    pub fn current_path(&self) -> Option<&Path2D> {
        self.current_path.as_ref()
    }

    /// Last planner/controller error string for diagnostics.
    #[must_use]
    pub fn last_error(&self) -> Option<&str> {
        self.last_error.as_deref()
    }

    fn require_epoch(&self, epoch: NavigationEpoch) -> Result<(), NavigationRuntimeError> {
        if epoch.0 != self.epoch {
            return Err(NavigationRuntimeError::StaleEpoch {
                current: self.epoch,
                received: epoch.0,
            });
        }
        Ok(())
    }
}

impl NavigationInterface for NavigationRuntime {
    fn set_goal(&mut self, goal: PoseStamped) -> bool {
        self.submit_goal(NavigationGoal::new(
            format!("navigation-{}", self.epoch.wrapping_add(1)),
            goal,
        ))
        .is_ok()
    }

    fn get_state(&self) -> NavigationState {
        match self.state {
            NavigationLifecycleState::FollowingPath => NavigationState::FollowingPath,
            NavigationLifecycleState::Recovery => NavigationState::Recovery,
            _ => NavigationState::Idle,
        }
    }

    fn is_goal_reached(&self) -> bool {
        self.goal_reached
    }

    fn cancel_goal(&mut self) -> bool {
        self.cancel()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_types_messages::{
        NavFrameId, OccupancyCell, PlannerFailure, Point2, Pose3, Twist2,
    };

    struct Planner {
        result: PlannerResult,
    }

    impl GlobalPlanner for Planner {
        fn plan(&self, _request: PlannerRequest) -> Result<PlannerResult, NavError> {
            Ok(self.result.clone())
        }
    }

    #[derive(Default)]
    struct Controller {
        calls: usize,
    }

    impl PathController for Controller {
        fn compute_command(
            &mut self,
            input: ControllerInput,
        ) -> Result<ControllerCommand, NavError> {
            assert!(input.path.is_some());
            assert!(input.active_goal.is_some());
            self.calls += 1;
            Ok(ControllerCommand::twist_2d(
                Twist2 {
                    linear: Point2::new(0.2, 0.0),
                    angular_z: 0.0,
                },
                NavFrameId::Base,
            ))
        }
    }

    fn pose(x: f64, y: f64) -> PoseStamped {
        PoseStamped::from_pose(
            Pose3 {
                position: nuwax_nav_types_messages::Point3::new(x, y, 0.0),
                orientation: nuwax_nav_types_messages::Quaternion::IDENTITY,
            },
            NavFrameId::Map,
        )
    }

    fn map() -> OccupancyGrid2D {
        OccupancyGrid2D::filled(Point2::new(0.0, 0.0), 1.0, 4, 4, OccupancyCell::Free)
    }

    #[test]
    fn dimos_contract_accept_reach_and_cancel_semantics() {
        let mut runtime = NavigationRuntime::new();
        assert_eq!(runtime.get_state(), NavigationState::Idle);
        assert!(!runtime.set_goal(pose(1.0, 1.0)));

        runtime.start();
        assert!(runtime.set_goal(pose(1.0, 1.0)));
        let epoch = runtime.epoch();
        runtime
            .accept_plan(
                epoch,
                PlannerResult::success(
                    Path2D::from_points(vec![Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)]),
                    1.0,
                    2,
                ),
            )
            .unwrap();
        assert_eq!(runtime.get_state(), NavigationState::FollowingPath);
        runtime.mark_reached(epoch).unwrap();
        assert!(runtime.is_goal_reached());
        assert!(!runtime.cancel_goal());

        assert!(runtime.set_goal(pose(2.0, 2.0)));
        assert!(runtime.cancel_goal());
        assert!(!runtime.cancel_goal());
    }

    #[test]
    fn planner_and_controller_execute_through_owner() {
        let mut runtime = NavigationRuntime::new();
        runtime.start();
        let epoch = runtime
            .submit_goal(NavigationGoal::new("goal", pose(2.0, 2.0)))
            .unwrap();
        let planner = Planner {
            result: PlannerResult::success(
                Path2D::from_points(vec![Point2::new(0.0, 0.0), Point2::new(2.0, 2.0)]),
                2.0,
                3,
            ),
        };
        runtime
            .planner_step(epoch, &planner, pose(0.0, 0.0), map())
            .unwrap();
        let mut controller = Controller::default();
        let command = runtime
            .controller_step(
                epoch,
                &mut controller,
                Timestamp::default(),
                pose(0.0, 0.0),
                0.05,
                Some(0.5),
            )
            .unwrap();
        assert_eq!(controller.calls, 1);
        assert!(!command.stop);
    }

    #[test]
    fn blocked_plan_enters_recovery_and_stale_results_are_rejected() {
        let mut runtime = NavigationRuntime::new();
        runtime.start();
        let old_epoch = runtime
            .submit_goal(NavigationGoal::new("old", pose(1.0, 1.0)))
            .unwrap();
        runtime
            .accept_plan(old_epoch, PlannerResult::failure(PlannerFailure::NoPath, 4))
            .unwrap();
        assert_eq!(runtime.get_state(), NavigationState::Recovery);
        assert!(runtime.cancel());

        let new_epoch = runtime
            .submit_goal(NavigationGoal::new("new", pose(2.0, 2.0)))
            .unwrap();
        assert_ne!(old_epoch, new_epoch);
        assert!(matches!(
            runtime.accept_plan(
                old_epoch,
                PlannerResult::success(Path2D::from_points(vec![Point2::new(1.0, 1.0)]), 0.0, 1)
            ),
            Err(NavigationRuntimeError::StaleEpoch { .. })
        ));
    }

    fn terminal_result(status: NavigationStatus) -> PlannerResult {
        let failure = match status {
            NavigationStatus::Blocked => Some(PlannerFailure::NoPath),
            NavigationStatus::Error => Some(PlannerFailure::Internal),
            _ => None,
        };
        PlannerResult {
            path: Some(Path2D::from_points(vec![
                Point2::new(0.0, 0.0),
                Point2::new(1.0, 1.0),
            ])),
            status,
            cost: None,
            explored_nodes: 2,
            failure,
        }
    }

    #[test]
    fn current_epoch_terminal_plans_never_start_path_following() {
        for (status, expected_state) in [
            (
                NavigationStatus::Blocked,
                NavigationLifecycleState::Recovery,
            ),
            (
                NavigationStatus::Cancelled,
                NavigationLifecycleState::Cancelled,
            ),
            (NavigationStatus::Reached, NavigationLifecycleState::Reached),
            (NavigationStatus::Error, NavigationLifecycleState::Failed),
        ] {
            let mut runtime = NavigationRuntime::new();
            runtime.start();
            let epoch = runtime
                .submit_goal(NavigationGoal::new("goal", pose(1.0, 1.0)))
                .unwrap();

            runtime.accept_plan(epoch, terminal_result(status)).unwrap();

            assert_eq!(runtime.lifecycle_state(), expected_state, "{status:?}");
            assert_ne!(runtime.get_state(), NavigationState::FollowingPath);
            assert!(runtime.current_path().is_none());
            assert!(runtime
                .last_error()
                .unwrap()
                .contains(&format!("{status:?}")));
            match status {
                NavigationStatus::Blocked | NavigationStatus::Error => {
                    assert!(!runtime.is_goal_reached());
                    assert!(runtime.cancel_goal());
                }
                NavigationStatus::Cancelled => {
                    assert!(!runtime.is_goal_reached());
                    assert_ne!(runtime.epoch(), epoch);
                    assert!(!runtime.cancel_goal());
                }
                NavigationStatus::Reached => {
                    assert!(runtime.is_goal_reached());
                    assert!(!runtime.cancel_goal());
                }
                _ => unreachable!("test covers terminal statuses only"),
            }
        }
    }

    #[test]
    fn stale_epoch_terminal_plans_are_rejected_without_state_changes() {
        for status in [
            NavigationStatus::Blocked,
            NavigationStatus::Cancelled,
            NavigationStatus::Reached,
            NavigationStatus::Error,
        ] {
            let mut runtime = NavigationRuntime::new();
            runtime.start();
            let stale_epoch = runtime
                .submit_goal(NavigationGoal::new("old", pose(1.0, 1.0)))
                .unwrap();
            let current_epoch = runtime
                .submit_goal(NavigationGoal::new("current", pose(2.0, 2.0)))
                .unwrap();

            assert!(matches!(
                runtime.accept_plan(stale_epoch, terminal_result(status)),
                Err(NavigationRuntimeError::StaleEpoch {
                    current,
                    received
                }) if current == current_epoch.0 && received == stale_epoch.0
            ));
            assert_eq!(
                runtime.lifecycle_state(),
                NavigationLifecycleState::Planning
            );
            assert!(runtime.current_path().is_none());
            assert!(runtime.last_error().is_none());
        }
    }

    #[test]
    fn empty_or_contradictory_success_plans_fail_closed() {
        let invalid_results = [
            PlannerResult::success(Path2D::from_points(vec![]), 0.0, 0),
            PlannerResult {
                path: Some(Path2D::from_points(vec![Point2::new(1.0, 1.0)])),
                status: NavigationStatus::Planning,
                cost: Some(1.0),
                explored_nodes: 1,
                failure: Some(PlannerFailure::Internal),
            },
            PlannerResult {
                path: Some(Path2D::from_points(vec![Point2::new(1.0, 1.0)])),
                status: NavigationStatus::Planning,
                cost: None,
                explored_nodes: 1,
                failure: None,
            },
            PlannerResult::success(
                Path2D::from_points(vec![Point2::new(1.0, 1.0)]),
                f64::NAN,
                1,
            ),
        ];

        for result in invalid_results {
            let mut runtime = NavigationRuntime::new();
            runtime.start();
            let epoch = runtime
                .submit_goal(NavigationGoal::new("goal", pose(1.0, 1.0)))
                .unwrap();

            runtime.accept_plan(epoch, result).unwrap();

            assert_eq!(runtime.lifecycle_state(), NavigationLifecycleState::Failed);
            assert!(runtime.current_path().is_none());
            assert!(runtime
                .last_error()
                .unwrap()
                .contains("invalid successful planner result"));
        }
    }

    #[test]
    fn non_finite_success_path_coordinates_fail_closed() {
        for path in [
            Path2D::from_points(vec![Point2::new(f64::NAN, 1.0)]),
            Path2D::from_points(vec![Point2::new(1.0, f64::INFINITY)]),
        ] {
            let mut runtime = NavigationRuntime::new();
            runtime.start();
            let epoch = runtime
                .submit_goal(NavigationGoal::new("goal", pose(1.0, 1.0)))
                .unwrap();

            runtime
                .accept_plan(epoch, PlannerResult::success(path, 1.0, 1))
                .unwrap();

            assert_eq!(runtime.lifecycle_state(), NavigationLifecycleState::Failed);
            assert!(runtime.current_path().is_none());
            assert!(runtime.last_error().unwrap().contains("valid_path=false"));
        }
    }

    #[test]
    fn success_path_in_incompatible_frame_fails_closed() {
        let mut runtime = NavigationRuntime::new();
        runtime.start();
        let epoch = runtime
            .submit_goal(NavigationGoal::new("goal", pose(1.0, 1.0)))
            .unwrap();
        let mut path = Path2D::from_points(vec![Point2::new(1.0, 1.0)]);
        path.frame_id = NavFrameId::Odom;

        runtime
            .accept_plan(epoch, PlannerResult::success(path, 1.0, 1))
            .unwrap();

        assert_eq!(runtime.lifecycle_state(), NavigationLifecycleState::Failed);
        assert!(runtime.current_path().is_none());
        assert!(runtime
            .last_error()
            .unwrap()
            .contains("frame_matches_goal=false"));
    }

    #[test]
    fn contradictory_reached_results_do_not_mark_goal_complete() {
        for (failure, cost) in [
            (Some(PlannerFailure::Internal), None),
            (None, Some(1.0)),
            (None, Some(f64::NAN)),
        ] {
            let mut runtime = NavigationRuntime::new();
            runtime.start();
            let epoch = runtime
                .submit_goal(NavigationGoal::new("goal", pose(1.0, 1.0)))
                .unwrap();
            let result = PlannerResult {
                path: None,
                status: NavigationStatus::Reached,
                cost,
                explored_nodes: 1,
                failure,
            };

            runtime.accept_plan(epoch, result).unwrap();

            assert_eq!(runtime.lifecycle_state(), NavigationLifecycleState::Failed);
            assert!(!runtime.is_goal_reached());
            assert!(runtime.current_path().is_none());
            assert!(runtime
                .last_error()
                .unwrap()
                .contains("invalid Reached planner result"));
        }
    }

    #[test]
    fn non_planning_statuses_cannot_install_a_success_path() {
        for status in [NavigationStatus::Idle, NavigationStatus::Moving] {
            let mut runtime = NavigationRuntime::new();
            runtime.start();
            let epoch = runtime
                .submit_goal(NavigationGoal::new("goal", pose(1.0, 1.0)))
                .unwrap();
            let mut result =
                PlannerResult::success(Path2D::from_points(vec![Point2::new(1.0, 1.0)]), 1.0, 1);
            result.status = status;

            runtime.accept_plan(epoch, result).unwrap();

            assert_eq!(runtime.lifecycle_state(), NavigationLifecycleState::Failed);
            assert!(runtime.current_path().is_none());
            assert!(runtime
                .last_error()
                .unwrap()
                .contains(&format!("{status:?}")));
        }
    }

    #[test]
    fn invalid_transition_is_typed() {
        let mut runtime = NavigationRuntime::new();
        runtime.start();
        assert!(matches!(
            runtime.mark_reached(runtime.epoch()),
            Err(NavigationRuntimeError::InvalidTransition { .. })
        ));
    }
}
