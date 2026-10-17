//! Test-only observations of path admission and the actual selected waypoint.
//! This module must be included behind #[cfg(test)]. No geometry is queried here.
use nuwax_nav_scan::BodyPose;
use nuwax_nav_types_messages::PoseStamped;
use std::cell::RefCell;

#[derive(Clone, Debug)]
pub(crate) struct Admission {
    pub actual: Option<BodyPose>,
    pub path_first: Option<PoseStamped>,
    pub progress_before: f64,
    pub path_index_before: usize,
    pub projected: Option<PoseStamped>,
    pub progress: Option<f64>,
    pub position_gate_passed: Option<bool>,
    /// None means this connector was not reached, not that it failed.
    pub connector_ok: Option<bool>,
    /// None means the observed call did not return normally.
    pub accepted: Option<bool>,
    pub error: Option<String>,
    pub progress_after: Option<f64>,
    pub path_index_after: Option<usize>,
}

#[derive(Clone, Debug)]
pub(crate) struct Target {
    pub path_index: usize,
    pub target_index: usize,
    pub rotation: bool,
    pub actual: BodyPose,
    /// The selected path waypoint, before any later control-target adaptation.
    pub target: PoseStamped,
    /// path[path_index - 1], not path[target_index - 1].
    pub previous: PoseStamped,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct ProbeFrame {
    pub admissions: Vec<Admission>,
    pub targets: Vec<Target>,
}

#[derive(Default)]
struct State {
    frame: Option<ProbeFrame>,
    active_admission: Option<usize>,
}
thread_local! {
    static STATE: RefCell<State> = RefCell::new(State::default());
}

/// Start observing this thread only. Owner-worker geometry is not instrumented.
pub(crate) fn begin() {
    STATE.with(|state| {
        let mut state = state.borrow_mut();
        assert!(state.frame.is_none(), "rejoin probe already active");
        state.active_admission = None;
        state.frame = Some(ProbeFrame::default());
    });
}

/// End observation. Tests may begin/end once for admission and once per tick.
pub(crate) fn end() -> ProbeFrame {
    STATE.with(|state| {
        let mut state = state.borrow_mut();
        state.active_admission = None;
        state.frame.take().expect("active rejoin probe")
    })
}

fn body(pose: &PoseStamped) -> BodyPose {
    let p = pose.pose.position;
    let q = pose.pose.orientation;
    ([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
}

pub(crate) fn admission_begin(
    current: Option<&PoseStamped>,
    path_first: Option<&PoseStamped>,
    progress_before: f64,
    path_index_before: usize,
) {
    STATE.with(|state| {
        let mut state = state.borrow_mut();
        if state.frame.is_none() {
            return;
        }
        assert!(state.active_admission.is_none(), "nested path admission");
        let frame = state.frame.as_mut().unwrap();
        let index = frame.admissions.len();
        frame.admissions.push(Admission {
            actual: current.map(body),
            path_first: path_first.cloned(),
            progress_before,
            path_index_before,
            projected: None,
            progress: None,
            position_gate_passed: None,
            connector_ok: None,
            accepted: None,
            error: None,
            progress_after: None,
            path_index_after: None,
        });
        state.active_admission = Some(index);
    });
}

fn update_admission(update: impl FnOnce(&mut Admission)) {
    STATE.with(|state| {
        let mut state = state.borrow_mut();
        let Some(index) = state.active_admission else {
            return;
        };
        if let Some(admission) = state
            .frame
            .as_mut()
            .and_then(|frame| frame.admissions.get_mut(index))
        {
            update(admission);
        }
    });
}

pub(crate) fn admission_projection(progress: f64, projected: &PoseStamped) {
    update_admission(|admission| {
        admission.projected = Some(projected.clone());
        admission.progress = Some(progress);
    });
}

pub(crate) fn admission_position_gate(passed: bool) {
    update_admission(|admission| admission.position_gate_passed = Some(passed));
}

pub(crate) fn admission_connector(succeeded: bool) {
    update_admission(|admission| admission.connector_ok = Some(succeeded));
}

pub(crate) fn admission_end(
    result: &Result<(), crate::RuntimeError>,
    progress_after: f64,
    path_index_after: usize,
) {
    update_admission(|admission| {
        admission.accepted = Some(result.is_ok());
        admission.error = result.as_ref().err().map(ToString::to_string);
        admission.progress_after = Some(progress_after);
        admission.path_index_after = result.is_ok().then_some(path_index_after);
    });
    STATE.with(|state| state.borrow_mut().active_admission = None);
}

pub(crate) fn target(
    path_index: usize,
    target_index: usize,
    rotation: bool,
    current: &PoseStamped,
    target: &PoseStamped,
    previous: &PoseStamped,
) {
    STATE.with(|state| {
        if let Some(frame) = state.borrow_mut().frame.as_mut() {
            frame.targets.push(Target {
                path_index,
                target_index,
                rotation,
                actual: body(current),
                target: target.clone(),
                previous: previous.clone(),
            });
        }
    });
}
