//! 仲裁矩阵集成测试(方案 §8.2)
//!
//! 覆盖:
//! - `HighestScore` 选 score 最高的 `Goal`
//! - `FirstAvailable` 顺序返回首个 non-pending
//! - `PreferWavefront` pending fallback 到 TARE
//! - `PreferWavefront` stop 立即结束,不 fallback

use std::borrow::Cow;

use nuwax_exploration::conversions::frontier_goal_to_navigation_goal;
use nuwax_exploration::{
    ExplorationArbitrationPolicy, ExplorationController, ExplorationGoalSource, ExplorationPoll,
    ExplorationStateSnapshot, ExplorationStopReason, ExploreError,
};
use nuwax_nav_types_messages::frames::NavFrameId;
use nuwax_nav_types_messages::geometry::{Point3, Pose3, PoseStamped, Quaternion, Timestamp};
use nuwax_nav_types_messages::goal::GoalSource;
use nuwax_nav_types_messages::visual::FrontierGoal;

#[derive(Debug)]
struct ScriptedSource {
    name: Cow<'static, str>,
    sequence: Vec<ExplorationPoll>,
    diagnostics_name: String,
}

impl ScriptedSource {
    fn new(name: &'static str, sequence: Vec<ExplorationPoll>) -> Self {
        Self {
            name: Cow::Borrowed(name),
            sequence,
            diagnostics_name: name.to_string(),
        }
    }
}

impl ExplorationGoalSource for ScriptedSource {
    fn source_name(&self) -> Cow<'static, str> {
        self.name.clone()
    }
    fn begin(&mut self) -> Result<(), ExploreError> {
        Ok(())
    }
    fn end(&mut self) -> Result<(), ExploreError> {
        Ok(())
    }
    fn poll_goal(
        &mut self,
        _state: &ExplorationStateSnapshot<'_>,
    ) -> Result<ExplorationPoll, ExploreError> {
        if self.sequence.is_empty() {
            Ok(ExplorationPoll::Pending)
        } else {
            Ok(self.sequence.remove(0))
        }
    }
    fn diagnostics(&self) -> nuwax_exploration::ExplorationSourceDiagnostics {
        nuwax_exploration::ExplorationSourceDiagnostics {
            source_name: self.diagnostics_name.clone(),
            is_active: true,
            last_goal_id: None,
            last_error: None,
        }
    }
}

fn sample_goal(id: &str, score: f32) -> FrontierGoal {
    let pose = PoseStamped {
        frame_id: NavFrameId::Map,
        stamp: Timestamp::from_unix(0, 0),
        pose: Pose3 {
            position: Point3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: Quaternion::IDENTITY,
        },
    };
    FrontierGoal::new(id, pose, score)
}

fn empty_state<'a>() -> ExplorationStateSnapshot<'a> {
    ExplorationStateSnapshot::from_xy((0.0, 0.0), Timestamp::default())
}

#[test]
fn highest_score_picks_max_score() {
    let mut ctrl = ExplorationController::new(ExplorationArbitrationPolicy::HighestScore);
    ctrl.add_source(ScriptedSource::new(
        "a",
        vec![ExplorationPoll::Goal(sample_goal("a-low", 0.3))],
    ));
    ctrl.add_source(ScriptedSource::new(
        "b",
        vec![ExplorationPoll::Goal(sample_goal("b-high", 0.9))],
    ));
    ctrl.begin_exploration().unwrap();
    let poll = ctrl.poll_next_goal(&empty_state()).unwrap();
    match poll {
        ExplorationPoll::Goal(g) => assert_eq!(g.id, "b-high"),
        _ => panic!("expected Goal(b-high)"),
    }
}

#[test]
fn first_available_returns_first_non_pending() {
    let mut ctrl = ExplorationController::new(ExplorationArbitrationPolicy::FirstAvailable);
    ctrl.add_source(ScriptedSource::new(
        "first",
        vec![
            ExplorationPoll::Pending,
            ExplorationPoll::Goal(sample_goal("first-g", 0.5)),
        ],
    ));
    ctrl.add_source(ScriptedSource::new(
        "second",
        vec![ExplorationPoll::Goal(sample_goal("second-g", 0.7))],
    ));
    ctrl.begin_exploration().unwrap();
    // 第一次:首 source pending,fallback 到 second
    let poll = ctrl.poll_next_goal(&empty_state()).unwrap();
    match poll {
        ExplorationPoll::Goal(g) => assert_eq!(g.id, "second-g"),
        _ => panic!("expected Goal(second-g), got {poll:?}"),
    }
}

#[test]
fn prefer_wavefront_pending_falls_back_to_tare() {
    let mut ctrl = ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront);
    ctrl.add_source(ScriptedSource::new(
        "tare",
        vec![ExplorationPoll::Goal(sample_goal("tare-g", 0.4))],
    ));
    ctrl.add_source(ScriptedSource::new(
        "wavefront",
        vec![ExplorationPoll::Pending],
    ));
    ctrl.begin_exploration().unwrap();
    let poll = ctrl.poll_next_goal(&empty_state()).unwrap();
    match poll {
        ExplorationPoll::Goal(g) => assert_eq!(g.id, "tare-g"),
        _ => panic!("expected Goal(tare-g), got {poll:?}"),
    }
}

#[test]
fn prefer_wavefront_stop_terminates_without_tare_fallback() {
    let mut ctrl = ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront);
    ctrl.add_source(ScriptedSource::new(
        "tare",
        vec![ExplorationPoll::Goal(sample_goal("tare-g", 0.4))],
    ));
    ctrl.add_source(ScriptedSource::new(
        "wavefront",
        vec![ExplorationPoll::Stop(
            ExplorationStopReason::NoInformationGain,
        )],
    ));
    ctrl.begin_exploration().unwrap();
    let poll = ctrl.poll_next_goal(&empty_state()).unwrap();
    match poll {
        ExplorationPoll::Stop(r) => assert_eq!(r, ExplorationStopReason::NoInformationGain),
        _ => panic!("expected Stop"),
    }
    assert!(!ctrl.is_active());
}

#[test]
fn prefer_wavefront_picks_wavefront_when_available() {
    let mut ctrl = ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront);
    ctrl.add_source(ScriptedSource::new(
        "tare",
        vec![ExplorationPoll::Goal(sample_goal("tare-g", 0.4))],
    ));
    ctrl.add_source(ScriptedSource::new(
        "wavefront",
        vec![ExplorationPoll::Goal(sample_goal("wave-g", 0.6))],
    ));
    ctrl.begin_exploration().unwrap();
    let poll = ctrl.poll_next_goal(&empty_state()).unwrap();
    match poll {
        ExplorationPoll::Goal(g) => assert_eq!(g.id, "wave-g"),
        _ => panic!("expected Goal(wave-g)"),
    }
}

#[test]
fn source_unavailable_with_pending_returns_pending() {
    let mut ctrl = ExplorationController::new(ExplorationArbitrationPolicy::FirstAvailable);
    ctrl.add_source(ScriptedSource::new(
        "a",
        vec![ExplorationPoll::SourceUnavailable("oops".into())],
    ));
    ctrl.add_source(ScriptedSource::new("b", vec![ExplorationPoll::Pending]));
    ctrl.begin_exploration().unwrap();
    let poll = ctrl.poll_next_goal(&empty_state()).unwrap();
    assert!(matches!(poll, ExplorationPoll::Pending));
}

#[test]
fn source_unavailable_without_any_pending_returns_unavailable() {
    let mut ctrl = ExplorationController::new(ExplorationArbitrationPolicy::FirstAvailable);
    ctrl.add_source(ScriptedSource::new(
        "a",
        vec![ExplorationPoll::SourceUnavailable("err".into())],
    ));
    ctrl.begin_exploration().unwrap();
    let poll = ctrl.poll_next_goal(&empty_state()).unwrap();
    match poll {
        ExplorationPoll::SourceUnavailable(s) => assert!(s.contains("err")),
        _ => panic!("expected SourceUnavailable, got {poll:?}"),
    }
}

#[test]
fn conversions_used_in_test() {
    // smoke: conversion function is callable from integration test
    let g = sample_goal("c1", 0.42);
    let nav = frontier_goal_to_navigation_goal(&g);
    assert_eq!(nav.id, "c1");
    assert!(matches!(nav.source, GoalSource::Frontier));
}
