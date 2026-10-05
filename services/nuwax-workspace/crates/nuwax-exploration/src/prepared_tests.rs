use super::*;
use std::sync::{Arc, Mutex};

#[derive(Default, Debug, Clone, PartialEq)]
struct Counts {
    commits: usize,
    ids: usize,
    feedback: usize,
    ends: usize,
}
struct TransactionalSource {
    name: &'static str,
    active: bool,
    outcome: ExploreResult<ExplorationPoll>,
    pending: bool,
    counts: Arc<Mutex<Counts>>,
    end_order: Arc<Mutex<Vec<&'static str>>>,
    last_goal: Option<String>,
    fail_feedback: bool,
    prepare_error: bool,
    discard_error: bool,
    finalize_override: Option<ExploreResult<ExplorationPoll>>,
}
impl TransactionalSource {
    fn apply(&mut self) -> ExploreResult<ExplorationPoll> {
        let mut counts = self.counts.lock().unwrap();
        counts.commits += 1;
        let mut result = self.outcome.clone();
        if let Ok(ExplorationPoll::Goal(goal)) = &mut result {
            goal.id = format!("{}-{}", self.name, counts.ids);
            counts.ids += 1;
            self.last_goal = Some(goal.id.clone());
        }
        result
    }
}
impl ExplorationGoalSource for TransactionalSource {
    fn source_name(&self) -> Cow<'static, str> {
        Cow::Borrowed(self.name)
    }
    fn begin(&mut self) -> ExploreResult<()> {
        self.active = true;
        Ok(())
    }
    fn end(&mut self) -> ExploreResult<()> {
        if self.pending {
            return Err(ExploreError::PreparedPollBusy);
        }
        self.active = false;
        self.counts.lock().unwrap().ends += 1;
        self.end_order.lock().unwrap().push(self.name);
        Ok(())
    }
    fn poll_goal(&mut self, _: &ExplorationStateSnapshot<'_>) -> ExploreResult<ExplorationPoll> {
        self.apply()
    }
    fn supports_prepared_poll(&self) -> bool {
        true
    }
    fn prepare_poll(
        &mut self,
        _: &ExplorationStateSnapshot<'_>,
    ) -> ExploreResult<PreparedExplorationPoll> {
        if self.pending {
            return Err(ExploreError::PreparedPollBusy);
        }
        if self.prepare_error {
            return Err(ExploreError::SourceError("prepare rejected".into()));
        }
        self.pending = true;
        Ok(PreparedExplorationPoll::new(self.outcome.clone()))
    }
    fn commit_prepared_poll(&mut self) -> ExploreResult<ExplorationPoll> {
        if !self.pending {
            return Err(ExploreError::NoPreparedPoll);
        }
        self.pending = false;
        let actual = self.apply();
        self.finalize_override.clone().unwrap_or(actual)
    }
    fn discard_prepared_poll(&mut self) -> ExploreResult<()> {
        if self.discard_error {
            return Err(ExploreError::SourceError("discard rejected".into()));
        }
        if !self.pending {
            return Err(ExploreError::NoPreparedPoll);
        }
        self.pending = false;
        Ok(())
    }
    fn report_goal_feedback(&mut self, _: &str, _: ExplorationGoalFeedback) -> ExploreResult<()> {
        assert!(!self.pending);
        self.counts.lock().unwrap().feedback += 1;
        if self.fail_feedback {
            Err(ExploreError::SourceError("accepted feedback error".into()))
        } else {
            Ok(())
        }
    }
    fn diagnostics(&self) -> ExplorationSourceDiagnostics {
        ExplorationSourceDiagnostics {
            source_name: self.name.into(),
            is_active: self.active,
            last_goal_id: self.last_goal.clone(),
            last_error: None,
        }
    }
}
fn state() -> ExplorationStateSnapshot<'static> {
    ExplorationStateSnapshot::from_xy((0.0, 0.0), Timestamp::default())
}
fn goal(score: f32) -> ExploreResult<ExplorationPoll> {
    Ok(ExplorationPoll::Goal(FrontierGoal::new(
        "",
        state().robot_pose,
        score,
    )))
}
fn controller(
    policy: ExplorationArbitrationPolicy,
    outcomes: Vec<ExploreResult<ExplorationPoll>>,
) -> (
    ExplorationController,
    Vec<Arc<Mutex<Counts>>>,
    Arc<Mutex<Vec<&'static str>>>,
) {
    let order = Arc::new(Mutex::new(Vec::new()));
    let mut counts = Vec::new();
    let mut controller = ExplorationController::new(policy);
    for (index, outcome) in outcomes.into_iter().enumerate() {
        let count = Arc::new(Mutex::new(Counts::default()));
        counts.push(count.clone());
        controller.add_source(TransactionalSource {
            name: ["wavefront", "surface", "third"][index],
            active: false,
            outcome,
            pending: false,
            counts: count,
            end_order: order.clone(),
            last_goal: None,
            fail_feedback: false,
            prepare_error: false,
            discard_error: false,
            finalize_override: None,
        });
    }
    controller.begin_exploration().unwrap();
    (controller, counts, order)
}
fn counts(values: &[Arc<Mutex<Counts>>]) -> Vec<Counts> {
    values.iter().map(|v| v.lock().unwrap().clone()).collect()
}

#[test]
fn prepared_controller_matches_sequential_arbitration_including_losers_errors_and_ties() {
    let cases = vec![
        vec![goal(2.0), goal(1.0)],
        vec![goal(1.0), goal(2.0)],
        vec![goal(1.0), goal(1.0)],
        vec![
            Err(ExploreError::SourceError("partial effect".into())),
            goal(1.0),
        ],
        vec![
            Ok(ExplorationPoll::Pending),
            Ok(ExplorationPoll::NoFrontier),
        ],
        vec![
            goal(3.0),
            Ok(ExplorationPoll::Stop(
                ExplorationStopReason::NoInformationGain,
            )),
        ],
        vec![
            Ok(ExplorationPoll::SourceUnavailable("missing".into())),
            Ok(ExplorationPoll::NoFrontier),
        ],
    ];
    for policy in [
        ExplorationArbitrationPolicy::HighestScore,
        ExplorationArbitrationPolicy::FirstAvailable,
        ExplorationArbitrationPolicy::PreferWavefront,
        ExplorationArbitrationPolicy::PreferTare,
    ] {
        for outcomes in &cases {
            for deferred in [false, true] {
                let (mut direct, dc, do_) = controller(policy, outcomes.clone());
                let (mut prepared, pc, po) = controller(policy, outcomes.clone());
                let before = prepared.diagnostics();
                prepared
                    .prepare_poll_with_deferred_stop(&state(), deferred)
                    .unwrap();
                assert_eq!(prepared.diagnostics(), before);
                assert!(counts(&pc)
                    .iter()
                    .all(|c| c.commits == 0 && c.ids == 0 && c.ends == 0));
                let actual = prepared.commit_prepared_poll().unwrap();
                let expected = direct
                    .poll_next_goal_with_deferred_stop(&state(), deferred)
                    .unwrap();
                assert_eq!(actual, expected);
                assert_eq!(counts(&pc), counts(&dc));
                assert_eq!(prepared.diagnostics(), direct.diagnostics());
                assert_eq!(*po.lock().unwrap(), *do_.lock().unwrap());
                assert!(matches!(
                    prepared.commit_prepared_poll(),
                    Err(ExploreError::NoPreparedPoll)
                ));
            }
        }
    }
}
#[test]
fn discard_controller_preserves_every_source_then_commit_allocates_ids_once() {
    let (mut controller, counts_, ends) = controller(
        ExplorationArbitrationPolicy::HighestScore,
        vec![
            goal(2.0),
            goal(1.0),
            Err(ExploreError::SourceError("partial".into())),
        ],
    );
    let baseline = controller.diagnostics();
    for _ in 0..3 {
        controller
            .prepare_poll_with_deferred_stop(&state(), false)
            .unwrap();
        assert!(matches!(
            controller.report_goal_feedback("x", ExplorationGoalFeedback::Reached),
            Err(ExploreError::PreparedPollBusy)
        ));
        assert!(matches!(
            controller.poll_next_goal(&state()),
            Err(ExploreError::PreparedPollBusy)
        ));
        controller.discard_prepared_poll().unwrap();
        assert_eq!(controller.diagnostics(), baseline);
    }
    assert!(counts(&counts_).iter().all(|c| *c == Counts::default()));
    assert!(ends.lock().unwrap().is_empty());
    controller
        .prepare_poll_with_deferred_stop(&state(), false)
        .unwrap();
    let result = controller.commit_prepared_poll().unwrap();
    assert!(matches!(result,ExplorationPoll::Goal(ref g) if g.id=="wavefront-0"));
    assert_eq!(
        counts(&counts_)
            .iter()
            .map(|c| c.commits)
            .collect::<Vec<_>>(),
        vec![1, 1, 1]
    );
    assert_eq!(
        counts(&counts_).iter().map(|c| c.ids).collect::<Vec<_>>(),
        vec![1, 1, 0]
    );
}
#[test]
fn discarded_stop_does_not_end_sources_and_explicit_stop_discards_before_reverse_end() {
    let (mut controller, counts_, ends) = controller(
        ExplorationArbitrationPolicy::HighestScore,
        vec![
            goal(1.0),
            Ok(ExplorationPoll::Stop(
                ExplorationStopReason::NoInformationGain,
            )),
        ],
    );
    controller
        .prepare_poll_with_deferred_stop(&state(), false)
        .unwrap();
    controller.discard_prepared_poll().unwrap();
    assert!(controller.is_active());
    assert!(ends.lock().unwrap().is_empty());
    controller
        .prepare_poll_with_deferred_stop(&state(), false)
        .unwrap();
    controller.cancel().unwrap();
    assert_eq!(*ends.lock().unwrap(), vec!["surface", "wavefront"]);
    assert!(counts(&counts_)
        .iter()
        .all(|c| c.commits == 0 && c.ids == 0 && c.ends == 1));
    assert!(matches!(
        controller.commit_prepared_poll(),
        Err(ExploreError::NoPreparedPoll)
    ));
}
#[test]
fn unsupported_source_is_rejected_before_any_preparation() {
    struct Unsupported;
    impl ExplorationGoalSource for Unsupported {
        fn source_name(&self) -> Cow<'static, str> {
            Cow::Borrowed("unsupported")
        }
        fn begin(&mut self) -> ExploreResult<()> {
            Ok(())
        }
        fn end(&mut self) -> ExploreResult<()> {
            Ok(())
        }
        fn poll_goal(
            &mut self,
            _: &ExplorationStateSnapshot<'_>,
        ) -> ExploreResult<ExplorationPoll> {
            panic!("not a transactional source")
        }
        fn diagnostics(&self) -> ExplorationSourceDiagnostics {
            ExplorationSourceDiagnostics {
                source_name: "unsupported".into(),
                is_active: true,
                last_goal_id: None,
                last_error: None,
            }
        }
    }
    let (mut controller, counts_, _) =
        controller(ExplorationArbitrationPolicy::HighestScore, vec![goal(1.0)]);
    controller.add_source(Unsupported);
    assert!(matches!(
        controller.prepare_poll_with_deferred_stop(&state(), false),
        Err(ExploreError::PreparedPollUnsupported)
    ));
    assert_eq!(counts(&counts_), vec![Counts::default()]);
    assert!(matches!(
        controller.discard_prepared_poll(),
        Err(ExploreError::NoPreparedPoll)
    ));
}
#[test]
fn accepted_partial_feedback_error_is_returned_without_internal_retry() {
    let (mut controller, counts_, _) =
        controller(ExplorationArbitrationPolicy::HighestScore, vec![goal(1.0)]);
    let failing = Arc::new(Mutex::new(Counts::default()));
    controller.add_source(TransactionalSource {
        name: "surface",
        active: true,
        outcome: Ok(ExplorationPoll::Pending),
        pending: false,
        counts: failing.clone(),
        end_order: Arc::new(Mutex::new(Vec::new())),
        last_goal: None,
        fail_feedback: true,
        prepare_error: false,
        discard_error: false,
        finalize_override: None,
    });
    assert!(controller
        .report_goal_feedback("goal", ExplorationGoalFeedback::Reached)
        .is_err());
    assert_eq!(counts(&counts_)[0].feedback, 1);
    assert_eq!(failing.lock().unwrap().feedback, 1);
}

fn scripted_source(
    name: &'static str,
    outcome: ExploreResult<ExplorationPoll>,
) -> TransactionalSource {
    TransactionalSource {
        name,
        active: false,
        outcome,
        pending: false,
        counts: Arc::new(Mutex::new(Counts::default())),
        end_order: Arc::new(Mutex::new(Vec::new())),
        last_goal: None,
        fail_feedback: false,
        prepare_error: false,
        discard_error: false,
        finalize_override: None,
    }
}
#[test]
fn changed_finalization_is_typed_fault_not_arbitration_panic_or_reuse() {
    for final_outcome in [
        Err(ExploreError::SourceError("commit changed".into())),
        Ok(ExplorationPoll::Pending),
        goal(9.0),
    ] {
        let mut first = scripted_source("first", goal(1.0));
        first.finalize_override = Some(final_outcome);
        let mut control = ExplorationController::new(ExplorationArbitrationPolicy::FirstAvailable);
        control.add_source(first);
        control.add_source(scripted_source("unvisited", goal(2.0)));
        control.begin_exploration().unwrap();
        control
            .prepare_poll_with_deferred_stop(&state(), false)
            .unwrap();
        assert!(matches!(
            control.commit_prepared_poll(),
            Err(ExploreError::PreparedPollProtocol(_))
        ));
        assert!(!control.is_active());
        assert!(matches!(
            control.checkpoint(),
            Err(ExploreError::PreparedPollProtocol(_))
        ));
        assert!(matches!(
            control.poll_next_goal(&state()),
            Err(ExploreError::PreparedPollProtocol(_))
        ));
        assert!(matches!(
            control.begin_exploration(),
            Err(ExploreError::PreparedPollProtocol(_))
        ));
        assert!(matches!(
            control.report_goal_feedback("old", ExplorationGoalFeedback::Reached),
            Err(ExploreError::PreparedPollProtocol(_))
        ));
    }
}
#[test]
fn prepare_or_explicit_discard_failure_poison_unresolved_source() {
    for explicit in [false, true] {
        let mut first = scripted_source("first", Ok(ExplorationPoll::Pending));
        first.discard_error = true;
        let mut second = scripted_source("second", Ok(ExplorationPoll::Pending));
        second.prepare_error = !explicit;
        let mut control = ExplorationController::new(ExplorationArbitrationPolicy::HighestScore);
        control.add_source(first);
        control.add_source(second);
        control.begin_exploration().unwrap();
        if explicit {
            control
                .prepare_poll_with_deferred_stop(&state(), false)
                .unwrap();
            assert!(matches!(
                control.discard_prepared_poll(),
                Err(ExploreError::PreparedPollProtocol(_))
            ));
        } else {
            let error = control
                .prepare_poll_with_deferred_stop(&state(), false)
                .unwrap_err();
            assert!(matches!(error, ExploreError::PreparedPollProtocol(_)));
            assert!(error.to_string().contains("discard rejected"));
        }
        assert!(matches!(
            control.checkpoint(),
            Err(ExploreError::PreparedPollProtocol(_))
        ));
        assert!(!control.is_active());
    }
}
#[test]
fn finalization_comparison_preserves_float_bits_without_new_numeric_policy() {
    let mut a = match goal(1.0).unwrap() {
        ExplorationPoll::Goal(g) => g,
        _ => unreachable!(),
    };
    let mut b = a.clone();
    b.id = "accepted".into();
    assert!(prepared_outcome_matches(
        &Ok(ExplorationPoll::Goal(a.clone())),
        &Ok(ExplorationPoll::Goal(b.clone()))
    ));
    b.pose.pose.position.x = -0.0;
    assert!(!prepared_outcome_matches(
        &Ok(ExplorationPoll::Goal(a.clone())),
        &Ok(ExplorationPoll::Goal(b))
    ));
    a.score = f32::from_bits(0x7fc00001);
    let b = a.clone();
    assert!(prepared_outcome_matches(
        &Ok(ExplorationPoll::Goal(a)),
        &Ok(ExplorationPoll::Goal(b))
    ));
}
