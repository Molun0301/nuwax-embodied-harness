//! Test-only provenance for an already-selected command. This is diagnostic
//! data, never motion authority, and keeps no source/map/path handles or hashes.

use crate::{PlannerStep, PoseStamped, Twist2};
use nuwax_nav_types_messages::{NavFrameId, Timestamp};
use std::{cell::RefCell, marker::PhantomData, rc::Rc, time::{Instant, SystemTime, UNIX_EPOCH}};

pub(crate) fn unix_ns() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH)
        .map(|duration| duration.as_nanos().min(u128::from(u64::MAX)) as u64)
        .unwrap_or(0)
}

pub(crate) fn command_bits(command: Twist2) -> [u64; 3] {
    [command.linear.x, command.linear.y, command.angular_z].map(f64::to_bits)
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct PoseTrace {
    pub(crate) bits: [u64; 7],
    pub(crate) stamp: Timestamp,
    pub(crate) frame_id: NavFrameId,
}
impl PoseTrace {
    pub(crate) fn capture(pose: &PoseStamped) -> Self {
        let position = pose.pose.position;
        let orientation = pose.pose.orientation;
        Self {
            bits: [position.x, position.y, position.z, orientation.x, orientation.y,
                orientation.z, orientation.w].map(f64::to_bits),
            stamp: pose.stamp,
            frame_id: pose.frame_id.clone(),
        }
    }
}

#[derive(Clone, Debug)]
pub(crate) struct ComputeInputTrace {
    pub(crate) result_id: u64,
    pub(crate) pose: Option<PoseTrace>,
    pub(crate) applied_bits: [u64; 3],
    pub(crate) epoch: u64,
    pub(crate) goal_version: u64,
    pub(crate) path_version: u64,
    pub(crate) map_version: u64,
    pub(crate) boundary_version: u64,
    pub(crate) pose_version: u64,
    pub(crate) applied_version: u64,
    pub(crate) compute_started: Instant,
    pub(crate) compute_start_unix_ns: u64,
}

#[derive(Clone, Debug)]
pub(crate) struct GuardSelectionTrace {
    pub(crate) invocation_id: u64,
    pub(crate) pose: PoseTrace,
    pub(crate) applied_bits: [u64; 3],
    pub(crate) selected_command_bits: [u64; 3],
    // This is copied from the guard's existing observation_check tuple. No
    // diagnostic clock read supplies, shifts, or substitutes its time anchor.
    pub(crate) observation_anchor_ns: Option<u64>,
    pub(crate) observation_max_age_ns: Option<u64>,
}

#[derive(Clone, Debug)]
pub(crate) struct ComputedCommandContext {
    pub(crate) input: ComputeInputTrace,
    pub(crate) guard: GuardSelectionTrace,
    pub(crate) compute_completion_unix_ns: u64,
    pub(crate) worker_guard_pose_bits_equal: bool,
    pub(crate) worker_guard_pose_stamp_equal: bool,
    pub(crate) worker_guard_pose_frame_equal: bool,
    pub(crate) worker_guard_applied_bits_equal: bool,
}
impl ComputedCommandContext {
    pub(crate) fn published(self) -> PublishedCommandContext {
        PublishedCommandContext { proof: self, publication_unix_ns: unix_ns() }
    }
}

#[derive(Clone, Debug)]
pub(crate) struct PublishedCommandContext {
    pub(crate) proof: ComputedCommandContext,
    pub(crate) publication_unix_ns: u64,
}
impl PublishedCommandContext {
    // The caller holds the result mutex and supplies that very stored result.
    pub(crate) fn retrieved(&self, started: Instant, command: Twist2) -> Option<DeliveredCommandContext> {
        if command == Twist2::ZERO || self.proof.input.compute_started != started
            || self.proof.guard.selected_command_bits != command_bits(command) {
            return None;
        }
        Some(DeliveredCommandContext {
            proof: self.proof.clone(),
            publication_unix_ns: self.publication_unix_ns,
            retrieval_unix_ns: unix_ns(),
        })
    }
}

#[derive(Clone, Debug)]
pub(crate) struct DeliveredCommandContext {
    pub(crate) proof: ComputedCommandContext,
    pub(crate) publication_unix_ns: u64,
    pub(crate) retrieval_unix_ns: u64,
}

struct GuardBasis {
    pose: PoseTrace,
    applied_bits: [u64; 3],
    observation: Option<(u64, u64)>,
}
struct GuardFrame {
    id: u64,
    basis: Option<GuardBasis>,
    selected: Option<GuardSelectionTrace>,
}
struct ComputeFrame {
    token: u64,
    input: ComputeInputTrace,
    next_guard_id: u64,
    guards: Vec<GuardFrame>,
    selected: Option<GuardSelectionTrace>,
}
impl ComputeFrame {
    fn invalidate_selection(&mut self) {
        self.selected = None;
        for guard in &mut self.guards { guard.selected = None; }
    }
}
#[derive(Default)]
struct TraceStack {
    next_token: u64,
    computes: Vec<ComputeFrame>,
}
thread_local! {
    static TRACE: RefCell<TraceStack> = RefCell::new(TraceStack::default());
}

// Scopes cannot cross threads: they own lineage in this thread's TLS only.
pub(crate) struct ComputeScope {
    token: Option<u64>,
    _thread: PhantomData<Rc<()>>,
}
impl ComputeScope {
    pub(crate) fn begin(input: ComputeInputTrace) -> Self {
        let token = TRACE.with(|trace| {
            let mut trace = trace.borrow_mut();
            if let Some(parent) = trace.computes.last_mut() { parent.invalidate_selection(); }
            trace.next_token = trace.next_token.checked_add(1).expect("test compute trace token exhausted");
            let token = trace.next_token;
            trace.computes.push(ComputeFrame {
                token, input, next_guard_id: 0, guards: Vec::new(), selected: None,
            });
            token
        });
        Self { token: Some(token), _thread: PhantomData }
    }

    pub(crate) fn finish(mut self, step: &PlannerStep) -> Option<ComputedCommandContext> {
        let compute_completion_unix_ns = unix_ns();
        let frame = take_compute(self.token.take()?)?;
        let PlannerStep::Command(command) = step else { return None; };
        if *command == Twist2::ZERO || !frame.guards.is_empty() { return None; }
        let guard = frame.selected?;
        if guard.selected_command_bits != command_bits(*command) { return None; }
        let pose = frame.input.pose.as_ref();
        Some(ComputedCommandContext {
            worker_guard_pose_bits_equal: pose.is_some_and(|pose| pose.bits == guard.pose.bits),
            worker_guard_pose_stamp_equal: pose.is_some_and(|pose| pose.stamp == guard.pose.stamp),
            worker_guard_pose_frame_equal: pose.is_some_and(|pose| pose.frame_id == guard.pose.frame_id),
            worker_guard_applied_bits_equal: frame.input.applied_bits == guard.applied_bits,
            input: frame.input, guard, compute_completion_unix_ns,
        })
    }
}
fn take_compute(token: u64) -> Option<ComputeFrame> {
    TRACE.with(|trace| {
        let mut trace = trace.borrow_mut();
        let index = trace.computes.iter().position(|frame| frame.token == token)?;
        if index + 1 != trace.computes.len() {
            // Misnested/manual drops cannot expose an inner scope as its parent.
            trace.computes.truncate(index);
            if let Some(parent) = trace.computes.last_mut() { parent.invalidate_selection(); }
            return None;
        }
        trace.computes.pop()
    })
}
impl Drop for ComputeScope {
    fn drop(&mut self) {
        if let Some(token) = self.token.take() { let _ = take_compute(token); }
    }
}

// Clear at bounded_body_command entry too: a later failed call can return
// before it reaches with_body_motion_checks and must not retain an old choice.
pub(crate) fn invalidate_selection() {
    TRACE.with(|trace| {
        if let Some(frame) = trace.borrow_mut().computes.last_mut() { frame.invalidate_selection(); }
    });
}

pub(crate) struct GuardScope {
    token: Option<(u64, u64)>,
    _thread: PhantomData<Rc<()>>,
}
impl GuardScope {
    pub(crate) fn begin() -> Self {
        let token = TRACE.with(|trace| {
            let mut trace = trace.borrow_mut();
            let frame = trace.computes.last_mut()?;
            frame.invalidate_selection();
            frame.next_guard_id = frame.next_guard_id.checked_add(1).expect("test guard trace ID exhausted");
            let id = frame.next_guard_id;
            frame.guards.push(GuardFrame { id, basis: None, selected: None });
            Some((frame.token, id))
        });
        Self { token, _thread: PhantomData }
    }

    pub(crate) fn record_basis(&self, pose: &PoseStamped, applied: Twist2, observation: Option<(u64, u64)>) {
        let Some((compute, guard)) = self.token else { return; };
        TRACE.with(|trace| {
            let mut trace = trace.borrow_mut();
            let Some(frame) = trace.computes.last_mut().filter(|frame| frame.token == compute) else { return; };
            let Some(active) = frame.guards.last_mut().filter(|active| active.id == guard) else { return; };
            active.basis = Some(GuardBasis { pose: PoseTrace::capture(pose), applied_bits: command_bits(applied), observation });
            active.selected = None;
        });
    }
}
impl Drop for GuardScope {
    fn drop(&mut self) {
        let Some((compute, guard)) = self.token.take() else { return; };
        TRACE.with(|trace| {
            let mut trace = trace.borrow_mut();
            let Some(frame) = trace.computes.last_mut().filter(|frame| frame.token == compute) else { return; };
            let Some(index) = frame.guards.iter().position(|active| active.id == guard) else { return; };
            if index + 1 != frame.guards.len() {
                frame.guards.truncate(index);
                frame.invalidate_selection();
                return;
            }
            let mut finished = frame.guards.pop().unwrap();
            if std::thread::panicking() { finished.selected = None; }
            if frame.guards.is_empty() { frame.selected = finished.selected; }
        });
    }
}

// Called only beside choose_tracking_control's returned Some(choice), while
// its exact with_body_motion_checks invocation is still the current scope.
pub(crate) fn record_selected(command: Twist2) {
    TRACE.with(|trace| {
        let mut trace = trace.borrow_mut();
        let Some(frame) = trace.computes.last_mut() else { return; };
        let Some(active) = frame.guards.last_mut() else { return; };
        let Some(basis) = &active.basis else { return; };
        active.selected = Some(GuardSelectionTrace {
            invocation_id: active.id,
            pose: basis.pose.clone(),
            applied_bits: basis.applied_bits,
            selected_command_bits: command_bits(command),
            observation_anchor_ns: basis.observation.map(|(anchor, _)| anchor),
            observation_max_age_ns: basis.observation.map(|(_, age)| age),
        });
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_types_messages::{Point2, Point3, Pose3, Quaternion};

    fn pose() -> PoseStamped {
        PoseStamped { frame_id: NavFrameId::Map, stamp: Timestamp::from_unix(7, 31),
            pose: Pose3 { position: Point3::new(1.0, -0.0, 0.5), orientation: Quaternion::IDENTITY } }
    }
    fn command() -> Twist2 { Twist2 { linear: Point2::new(0.05, 0.0), angular_z: 0.0 } }
    fn input(id: u64) -> ComputeInputTrace {
        ComputeInputTrace {
            result_id: id, pose: Some(PoseTrace::capture(&pose())), applied_bits: command_bits(Twist2::ZERO),
            epoch: 2, goal_version: 3, path_version: 5, map_version: 7, boundary_version: 11,
            pose_version: 13, applied_version: 17, compute_started: Instant::now(), compute_start_unix_ns: unix_ns(),
        }
    }
    fn selected(anchor: u64) {
        let guard = GuardScope::begin();
        guard.record_basis(&pose(), Twist2::ZERO, Some((anchor, 1_000_000_000)));
        record_selected(command());
    }

    #[test]
    fn exact_selection_carries_both_bases_and_all_result_stages() {
        let mut worker_input = input(23);
        worker_input.pose.as_mut().unwrap().bits[1] = 0.0_f64.to_bits();
        worker_input.pose.as_mut().unwrap().stamp = Timestamp::from_sensor(3);
        worker_input.pose.as_mut().unwrap().frame_id = NavFrameId::Odom;
        worker_input.applied_bits = command_bits(command());
        let started = worker_input.compute_started;
        let scope = ComputeScope::begin(worker_input);
        selected(29);
        let proof = scope.finish(&PlannerStep::Command(command())).unwrap();
        assert_eq!(proof.input.result_id, 23);
        assert_eq!(proof.guard.invocation_id, 1);
        assert_eq!(proof.guard.observation_anchor_ns, Some(29));
        assert_eq!(proof.guard.observation_max_age_ns, Some(1_000_000_000));
        assert_eq!(proof.guard.pose.bits[1], (-0.0_f64).to_bits());
        assert!(!proof.worker_guard_pose_bits_equal);
        assert!(!proof.worker_guard_pose_stamp_equal);
        assert!(!proof.worker_guard_pose_frame_equal);
        assert!(!proof.worker_guard_applied_bits_equal);
        let published = proof.published();
        let retrieved = published.retrieved(started, command()).unwrap();
        assert_eq!(retrieved.proof.input.result_id, 23);
        assert_eq!(retrieved.publication_unix_ns, published.publication_unix_ns);
        assert!(retrieved.retrieval_unix_ns > 0);
        assert!(published.retrieved(started + std::time::Duration::from_nanos(1), command()).is_none());
    }

    #[test]
    fn rejected_or_standalone_later_guard_cannot_relabel_a_previous_selection() {
        for record_later_basis in [false, true] {
            let scope = ComputeScope::begin(input(1));
            selected(19);
            {
                let rejected = GuardScope::begin();
                if record_later_basis { rejected.record_basis(&pose(), Twist2::ZERO, Some((97, 100))); }
                // No choose_tracking_control selection, even if a standalone
                // check or setup succeeded using a more recent observation time.
            }
            assert!(scope.finish(&PlannerStep::Command(command())).is_none());
        }
        let scope = ComputeScope::begin(input(2));
        selected(19);
        invalidate_selection(); // later bounded call fails before guard setup
        assert!(scope.finish(&PlannerStep::Command(command())).is_none());
    }

    #[test]
    fn nested_guard_never_publishes_as_outer_but_outer_can_select_after_it() {
        for outer_selects_after in [false, true] {
            let scope = ComputeScope::begin(input(1));
            {
                let outer = GuardScope::begin();
                outer.record_basis(&pose(), Twist2::ZERO, Some((19, 100)));
                record_selected(command());
                selected(97); // invalidates old outer choice, never exports inner
                if outer_selects_after { record_selected(command()); }
            }
            let proof = scope.finish(&PlannerStep::Command(command()));
            assert_eq!(proof.is_some(), outer_selects_after);
            if let Some(proof) = proof {
                assert_eq!(proof.guard.observation_anchor_ns, Some(19));
                assert_eq!(proof.guard.invocation_id, 1);
            }
        }
    }

    #[test]
    fn nested_compute_and_stale_scopes_never_transfer_selection() {
        let outer = ComputeScope::begin(input(1));
        selected(19);
        let inner = ComputeScope::begin(input(1)); // per-worker IDs may coincide
        selected(97);
        assert_eq!(inner.finish(&PlannerStep::Command(command())).unwrap().guard.observation_anchor_ns, Some(97));
        assert!(outer.finish(&PlannerStep::Command(command())).is_none());

        let old = ComputeScope::begin(input(2));
        let old_guard = GuardScope::begin();
        old_guard.record_basis(&pose(), Twist2::ZERO, Some((19, 100)));
        record_selected(command());
        drop(old);
        let current = ComputeScope::begin(input(3));
        selected(97);
        drop(old_guard);
        assert_eq!(current.finish(&PlannerStep::Command(command())).unwrap().guard.observation_anchor_ns, Some(97));
    }

    #[test]
    fn no_choice_zero_noncommand_and_raw_bit_mismatch_have_no_trace() {
        let scope = ComputeScope::begin(input(1));
        assert!(scope.finish(&PlannerStep::Command(command())).is_none());
        for outcome in [PlannerStep::Command(Twist2::ZERO), PlannerStep::Pending(Vec::new()),
            PlannerStep::Succeeded, PlannerStep::ReplanRequired("rejected".into()), PlannerStep::Failed("failed".into()),
            PlannerStep::Command(Twist2 { linear: Point2::new(0.05, -0.0), angular_z: 0.0 })] {
            let scope = ComputeScope::begin(input(2));
            selected(19);
            assert!(scope.finish(&outcome).is_none());
        }
        let next = ComputeScope::begin(input(3));
        assert!(next.finish(&PlannerStep::Command(command())).is_none());
    }

    #[test]
    fn unfinished_or_misnested_guards_and_unwind_cannot_leave_a_trace() {
        let scope = ComputeScope::begin(input(1));
        let outer = GuardScope::begin();
        let inner = GuardScope::begin();
        inner.record_basis(&pose(), Twist2::ZERO, None);
        record_selected(command());
        drop(outer);
        drop(inner);
        assert!(scope.finish(&PlannerStep::Command(command())).is_none());
        let _ = std::panic::catch_unwind(|| {
            let _scope = ComputeScope::begin(input(2));
            selected(19);
            panic!("exercise diagnostic scope unwinding");
        });
        let scope = ComputeScope::begin(input(3));
        assert!(scope.finish(&PlannerStep::Command(command())).is_none());

        let scope = ComputeScope::begin(input(4));
        let _ = std::panic::catch_unwind(|| {
            let guard = GuardScope::begin();
            guard.record_basis(&pose(), Twist2::ZERO, None);
            record_selected(command());
            panic!("a caught guard panic must not export its selection");
        });
        assert!(scope.finish(&PlannerStep::Command(command())).is_none());
    }
}
