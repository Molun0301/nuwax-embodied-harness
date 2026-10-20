//! UNCOMPILED, OUT-OF-REPOSITORY R11 LAYER-L QUALIFICATION DRAFT.
//! Child of route_follower, behind surface-recovery-qualification only.
//! Real PlanKernel, original PlanningOwner, no artificial worker wait and no path copy.
#![allow(missing_docs)]
use super::*;
use super::route_pending_source_qualification::{KernelEntry, SubmittedSource};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc;
use crate::planning_owner::qualification_first_dequeue::{FirstDequeueProbe, install as install_first_dequeue};

pub const R11_CONTROL_PERIOD: Duration = Duration::from_millis(50);
pub const R11_POSE_PERIOD: Duration = Duration::from_millis(25);
pub const R11_MAP_PERIOD: Duration = Duration::from_millis(200);
pub const R11_WINDOW: Duration = Duration::from_secs(30);
const SOURCE_PROBE: [i32; 3] = [251, 31, 40];
const EVENT_CAP: usize = 512;
const RECEIPT_CAP: usize = 256;
const CONTROL_CAP: usize = 1024;
const MAP_CAP: usize = 256;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum R11CaseKind { OnlyActive, InitialSlotObservation }
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum R11Arm { Baseline, Candidate }
#[derive(Clone, Debug)]
pub struct R11SourceMeta {
    pub source_ns: i64,
    pub revision: i64,
    pub observed_ns: u64,
    // Source acquisition is bracketed, not asserted to equal either endpoint.
    pub acquisition_before: Instant,
    pub acquisition_after: Instant,
    pub built_at: Instant,
}
pub struct R11Publication {
    pub update: SurfaceMapUpdate,
    pub meta: R11SourceMeta,
}
#[derive(Clone, Copy, Debug)]
pub enum R11Outcome { Full, Partial, Failure }
pub enum R11KernelEvent {
    Entry { at: Instant, identity: KernelEntry, source_ns: Option<u64> },
    Return { at: Instant, ordinal: u64, outcome: R11Outcome, cancelled: bool },
}
pub struct R11Receipt {
    pub observed_at: Instant,
    pub identity: SubmittedSource,
    pub epoch: u64,
    pub progress: f64,
    pub source_ns: i64,
    pub after_damage: bool,
}
#[derive(Clone, Debug)]
pub struct R11MapInstall {
    pub damage: bool,
    pub due: Instant,
    pub began: Instant,
    pub finished: Instant,
    pub source: R11SourceMeta,
    pub epoch_before: u64,
    pub epoch_after: u64,
    pub tickets_before: usize,
    pub tickets_after: usize,
    pub submitted_before: Option<u64>,
    pub submitted_after: Option<u64>,
}
#[derive(Clone, Copy, Debug)]
pub enum R11StepKind { Pending, Zero, Nonzero, Succeeded, Failed, ReplanRequired }
#[derive(Clone, Debug)]
pub struct R11ControlSample {
    pub due: Instant,
    pub entered: Instant,
    pub returned: Instant,
    pub kind: R11StepKind,
    pub admission: Option<u64>,
    pub generation: u64,
    pub plans: u64,
    pub tickets: usize,
    pub bound_present: bool,
    pub bound_deadline: Option<Instant>,
    pub bound_expired_at_return: bool,
    pub bound_expired_at_observation: bool,
    pub map_source_ns: i64,
    pub pose_stamp_ns: i64,
}
#[derive(Clone, Debug)]
pub struct R11Nonzero {
    pub at: Instant,
    pub map_source_ns: i64,
    pub pose_stamp_ns: i64,
    pub generation: u64,
    pub admission: u64,
    pub twist: Twist2,
    pub bound_present: bool,
    pub bound_deadline: Option<Instant>,
}
#[derive(Clone, Copy, Debug)]
pub struct R11FirstDequeue {
    pub id: u64,
    pub epoch: u64,
    pub before: Instant,
    pub after: Instant,
}
pub struct R11Report {
    pub arm: R11Arm,
    pub case: R11CaseKind,
    pub origin: Instant,
    pub ended: Instant,
    pub initial_id: u64,
    pub first_dequeue: Option<R11FirstDequeue>,
    pub receipts: Vec<R11Receipt>,
    pub kernel: Vec<R11KernelEvent>,
    pub maps: Vec<R11MapInstall>,
    pub controls: Vec<R11ControlSample>,
    pub first_nonzero: Option<R11Nonzero>,
    pub costs_before: (u64, u64),
    pub costs_after: (u64, u64),
    pub observer_drops: u64,
    pub skipped_pose_due: u64,
    pub skipped_control_due: u64,
    pub issues: Vec<String>,
    pub cleanup_returned: bool,
}
impl R11Report {
    pub fn damage(&self) -> Option<&R11MapInstall> { self.maps.iter().find(|m| m.damage) }
    pub fn entry_receipt(&self, entry: &KernelEntry, stamp: Option<u64>) -> Result<&R11Receipt, String> {
        let mut matches = self.receipts.iter().filter(|r| {
            entry.matches_submission(&r.identity)
                && entry.epoch == r.epoch
                && stamp == u64::try_from(r.source_ns).ok()
        });
        let first = matches.next().ok_or_else(|| format!("unmatched actual kernel ordinal {}", entry.compute_ordinal))?;
        if matches.next().is_some() {
            return Err(format!("ambiguous actual kernel ordinal {}", entry.compute_ordinal));
        }
        Ok(first)
    }
    // D or its genuinely observed Empty successor, matched to accepted identity.
    // This does not assert the source remains the newest publication at entry.
    pub fn first_latest_entry_detail(&self) -> Option<(Instant, &R11Receipt)> {
        self.kernel.iter().filter_map(|event| match event {
            R11KernelEvent::Entry { at, identity, source_ns } if *at <= self.ended && *at <= self.origin + R11_WINDOW => {
                self.entry_receipt(identity, *source_ns).ok()
                    .filter(|r| r.after_damage).map(|receipt| (*at, receipt))
            }
            _ => None,
        }).min_by_key(|(at, _)| *at)
    }
    pub fn first_latest_entry(&self) -> Option<Instant> { self.first_latest_entry_detail().map(|(at, _)| at) }
    pub fn only_active_at_damage_start(&self) -> bool {
        if self.observer_drops != 0 { return false; }
        let Some(damage) = self.damage() else { return false; };
        if damage.tickets_before != 1 || damage.submitted_before != Some(self.initial_id) { return false; }
        let Some((entered, ordinal)) = self.kernel.iter().find_map(|event| match event {
            R11KernelEvent::Entry { at, identity, source_ns } => self.entry_receipt(identity, *source_ns).ok()
                .filter(|r| r.identity.id == self.initial_id).map(|_| (*at, identity.compute_ordinal)),
            _ => None,
        }) else { return false; };
        let returned = self.kernel.iter().find_map(|event| match event {
            R11KernelEvent::Return { at, ordinal: got, .. } if *got == ordinal => Some(*at),
            _ => None,
        });
        // Use the pre-mutation instant, not update return: old cancellation may
        // correctly make A return during update_execution_map itself.
        entered < damage.began && returned.is_some_and(|at| at > damage.began)
    }
    pub fn initial_pending_coverage(&self) -> &'static str {
        let Some(damage) = self.damage() else { return "UNCOVERED_NO_DAMAGE"; };
        if damage.tickets_before != 1 || damage.submitted_before != Some(self.initial_id) {
            return "UNCOVERED_NOT_INITIAL_SINGLE_SLOT";
        }
        let Some(pop) = self.first_dequeue else { return "UNCOVERED_NO_FIRST_DEQUEUE_RECORD"; };
        if pop.before <= damage.finished && pop.after >= damage.began {
            return "UNCOVERED_DEQUEUE_OVERLAPS_DAMAGE_INTERVAL";
        }
        if pop.after < damage.began { return "NOT_PENDING_DEQUEUED_BEFORE_DAMAGE"; }
        if damage.finished < pop.before {
            // This is the FIRST actual pop in this owner. On baseline A may have
            // been cancelled out of pending, so pop.id may name its successor.
            // Neither case changes the fact that no pop happened before D.
            return "COVERED_FIRST_DEQUEUE_AFTER_DAMAGE_INSTALLED";
        }
        "UNCOVERED_ORDERING_NOT_STRICT"
    }
    pub fn validate_observations(&self) -> Result<(), String> {
        if self.observer_drops != 0 { return Err("kernel observer lost events".into()); }
        for event in &self.kernel {
            if let R11KernelEvent::Entry { identity, source_ns, .. } = event {
                self.entry_receipt(identity, *source_ns)?;
            }
        }
        if let Some(pop) = self.first_dequeue {
            if pop.after < pop.before { return Err("first dequeue interval reversed".into()); }
            let matching = self.receipts.iter().filter(|r| r.identity.id == pop.id && r.epoch == pop.epoch).count();
            if matching != 1 { return Err("actual first dequeue lacks exactly one accepted id/epoch".into()); }
        }
        let damage = self.damage().ok_or("damage publication not installed")?;
        match self.arm {
            R11Arm::Baseline if damage.epoch_after <= damage.epoch_before =>
                return Err("baseline did not execute its original whole-epoch cancellation".into()),
            R11Arm::Candidate if damage.epoch_after != damage.epoch_before =>
                return Err("candidate did not retain known-Columns epoch".into()),
            _ => {}
        }
        if self.costs_after.0 < self.costs_before.0 || self.costs_after.1 < self.costs_before.1 {
            return Err("qualification counter decreased or was reset inside case".into());
        }
        if self.costs_after.0 != self.costs_before.0 {
            return Err("fixed-pose single-slot case attempted new pending-replace lock".into());
        }
        // Builder totals include real ordinary accepted submissions. The F-layer
        // isolated gate test, not this whole-case total, proves builder==0 on skip.
        if self.controls.iter().any(|s| s.tickets > 1) {
            return Err("unexpected second ticket in fixed-pose single-slot replay".into());
        }
        if !self.issues.is_empty() { return Err(self.issues.join("; ")); }
        Ok(())
    }
}

pub struct R11NoHoldCase {
    follower: Option<RouteFollower>,
    arm: R11Arm,
    kind: R11CaseKind,
    receive: mpsc::Receiver<R11KernelEvent>,
    dropped: Arc<AtomicU64>,
    first_dequeue: Arc<FirstDequeueProbe>,
    origin: Option<Instant>,
    ended: Option<Instant>,
    initial_id: u64,
    current_source_ns: i64,
    damage_installed: bool,
    receipts: Vec<R11Receipt>,
    kernel: Vec<R11KernelEvent>,
    maps: Vec<R11MapInstall>,
    controls: Vec<R11ControlSample>,
    first_nonzero: Option<R11Nonzero>,
    costs_before: (u64, u64),
    costs_after: (u64, u64),
    skipped_pose_due: u64,
    skipped_control_due: u64,
    issues: Vec<String>,
}
impl R11NoHoldCase {
    pub fn prepare(arm: R11Arm, kind: R11CaseKind, config: SurfaceFollowerConfig,
        initial: Arc<SurfaceSnapshot>, initial_meta: &R11SourceMeta,
        pose: PoseStamped, route: Path3D, goal: PoseStamped) -> Result<Self, String> {
        if (config.controller.control_frequency - 20.0).abs() > f64::EPSILON {
            return Err("manifest requires original 20Hz controller; do not silently retune".into());
        }
        let actual_ns = initial.observations().and_then(|o| o.get(SOURCE_PROBE)).map(|o| o.observed_ns);
        if actual_ns != u64::try_from(initial_meta.source_ns).ok() { return Err("S0 actual source identity mismatch".into()); }
        let mut follower = RouteFollower::new(config.clone()).map_err(|e| e.to_string())?;
        follower.update_execution_map(ExecutionMap::Surface(initial)).map_err(|e| e.to_string())?;
        follower.update_odometry(pose);
        follower.configure_goal(&goal, GoalOptions::default()).map_err(|e| e.to_string())?;
        follower.replace_execution_path(ExecutionPath::Surface(route), None, goal).map_err(|e| e.to_string())?;
        if let Some(reason) = follower.controller.current_body_error() { return Err(reason); }
        if follower.next_request != 0 || follower.submitted.is_some() || follower.source_tickets.iter().any(Option::is_some) {
            return Err("only an idle never-submitted owner may be wrapped".into());
        }
        let (send, receive) = mpsc::sync_channel(EVENT_CAP);
        let dropped = Arc::new(AtomicU64::new(0));
        let loss = dropped.clone();
        let mut kernel = PlanKernel::new(config);
        let mut ordinal = 0_u64;
        let first_dequeue = FirstDequeueProbe::new();
        let dequeue_install = install_first_dequeue(&first_dequeue);
        follower.owner = Some(PlanningOwner::new(move |request: PlanRequest, cancel| {
            ordinal += 1;
            let identity = KernelEntry { compute_ordinal: ordinal, source: request.source.clone(),
                pose: request.pose.clone(), detour: request.detour,
                generation: request.generation, epoch: cancel.expected };
            let source_ns = request.source.observations().and_then(|o| o.get(SOURCE_PROBE)).map(|o| o.observed_ns);
            // No wait, ACK, sleep, path clone, or formatting in this wrapper.
            let entry = R11KernelEvent::Entry { at: Instant::now(), identity, source_ns };
            if send.try_send(entry).is_err() { loss.fetch_add(1, Ordering::Relaxed); }
            let result = kernel.compute(request, cancel);
            let outcome = match &result.outcome {
                Ok(candidate) if candidate.partial => R11Outcome::Partial,
                Ok(_) => R11Outcome::Full,
                Err(_) => R11Outcome::Failure,
            };
            let event = R11KernelEvent::Return { at: Instant::now(), ordinal, outcome, cancelled: cancel.cancelled() };
            if send.try_send(event).is_err() { loss.fetch_add(1, Ordering::Relaxed); }
            result
        }).map_err(|e| e.to_string())?);
        drop(dequeue_install);
        Ok(Self { follower: Some(follower), arm, kind, receive, dropped, first_dequeue, origin: None, ended: None,
            initial_id: 0, current_source_ns: initial_meta.source_ns, damage_installed: false,
            receipts: Vec::with_capacity(RECEIPT_CAP), kernel: Vec::with_capacity(EVENT_CAP),
            maps: Vec::with_capacity(MAP_CAP), controls: Vec::with_capacity(CONTROL_CAP), first_nonzero: None,
            costs_before: (0, 0), costs_after: (0, 0), skipped_pose_due: 0, skipped_control_due: 0,
            issues: Vec::new() })
    }
    fn record_receipt(&mut self, old_next: u64, observed_at: Instant) -> Result<(), String> {
        let f = self.follower.as_ref().ok_or("case already cleaned")?;
        if f.next_request == old_next { return Ok(()); }
        if f.next_request != old_next.checked_add(1).ok_or("request id exhausted")? {
            return Err("more than one acceptance in one observation interval".into());
        }
        let (id, _, detour) = f.submitted.ok_or("accepted request missing submitted receipt")?;
        if id != f.next_request { return Err("accepted id differs from next_request".into()); }
        if self.receipts.len() == RECEIPT_CAP { return Err("receipt observation cap exceeded".into()); }
        let source = f.map.as_ref().ok_or("accepted request lacks map")?.clone();
        let source_ns = source.observations().and_then(|o| o.get(SOURCE_PROBE)).map(|o| o.observed_ns);
        if source_ns != u64::try_from(self.current_source_ns).ok() { return Err("receipt source stamp mismatch".into()); }
        self.receipts.push(R11Receipt { observed_at,
            identity: SubmittedSource { id, source, pose: f.pose.as_ref().ok_or("accepted request lacks pose")?.clone(),
                detour, generation: f.route_generation },
            epoch: f.owner.as_ref().ok_or("accepted request lacks owner")?.epoch(), progress: f.progress,
            source_ns: self.current_source_ns, after_damage: self.damage_installed });
        Ok(())
    }
    pub fn begin(&mut self) -> Result<Instant, String> {
        if self.origin.is_some() { return Err("case already begun".into()); }
        let f = self.follower.as_mut().ok_or("missing follower")?;
        // Supplied by the separately reviewed two-counter tests-only seam.
        // Baseline and candidate must both expose this exact method.
        self.costs_before = f.pending_source_work_for_qualification();
        let previous = f.next_request;
        f.schedule(false);
        let receipt_at = Instant::now();
        self.record_receipt(previous, receipt_at)?;
        if self.receipts.len() != 1 { return Err("first ordinary A was not accepted".into()); }
        self.initial_id = self.receipts[0].identity.id;
        // The same observation immediately after ordinary schedule return is
        // the accepted-receipt timestamp and t0; receipt bookkeeping adds no
        // second hidden origin or artificial worker wait.
        self.origin = Some(receipt_at);
        Ok(receipt_at)
    }
    pub fn publish(&mut self, publication: R11Publication, damage: bool, due: Instant) -> Result<(), String> {
        if self.origin.is_none() { return Err("publish before begin".into()); }
        if damage == self.damage_installed { return Err("damage must be first and occur exactly once".into()); }
        if publication.meta.source_ns <= self.current_source_ns {
            return Err("genuine continuation source timestamp did not advance".into());
        }
        let actual_ns = publication.update.snapshot.observations().and_then(|o| o.get(SOURCE_PROBE)).map(|o| o.observed_ns);
        if actual_ns != u64::try_from(publication.meta.source_ns).ok() || actual_ns != Some(publication.meta.observed_ns) {
            return Err("actual map source does not match original Writer metadata".into());
        }
        if !damage && !matches!(publication.update.changes.as_ref(), SurfaceGeometryChanges::Empty) {
            return Err("first-batch replay allows only genuine Empty after damage".into());
        }
        if self.maps.len() == MAP_CAP { return Err("map observation cap exceeded".into()); }
        let f = self.follower.as_mut().ok_or("case already cleaned")?;
        let epoch_before = f.owner.as_ref().ok_or("missing owner")?.epoch();
        let submitted_before = f.submitted.map(|s| s.0);
        let tickets_before = f.source_tickets.iter().flatten().count();
        let began = Instant::now();
        let result = f.update_execution_map(ExecutionMap::SurfaceUpdate(publication.update));
        let finished = Instant::now();
        self.maps.push(R11MapInstall { damage, due, began, finished, source: publication.meta.clone(),
            epoch_before, epoch_after: f.owner.as_ref().ok_or("missing owner after update")?.epoch(),
            tickets_before, tickets_after: f.source_tickets.iter().flatten().count(),
            submitted_before, submitted_after: f.submitted.map(|s| s.0) });
        result.map_err(|e| e.to_string())?;
        self.current_source_ns = publication.meta.source_ns;
        self.damage_installed |= damage;
        Ok(())
    }
    pub fn update_pose(&mut self, pose: PoseStamped) -> Result<(), String> {
        let f = self.follower.as_mut().ok_or("case already cleaned")?;
        let old = f.pose.as_ref().ok_or("missing old pose")?;
        if old.frame_id != pose.frame_id || pose_bits(body(old)) != pose_bits(body(&pose))
            || old.stamp.clock != pose.stamp.clock || pose.stamp.nanos < old.stamp.nanos {
            return Err("fixed-pose replay changed pose/frame or moved source time backward".into());
        }
        f.update_odometry(pose);
        Ok(())
    }
    pub fn tick(&mut self, due: Instant) -> Result<(), String> {
        if self.controls.len() == CONTROL_CAP { return Err("control observation cap exceeded".into()); }
        let f = self.follower.as_mut().ok_or("case already cleaned")?;
        let previous = f.next_request;
        let entered = Instant::now();
        let step = f.step(0.3);
        let returned = Instant::now();
        let (kind, nonzero, terminal) = match &step {
            PlannerStep::Command(c) if *c != Twist2::ZERO => (R11StepKind::Nonzero, Some(*c), None),
            PlannerStep::Command(_) => (R11StepKind::Zero, None, None),
            PlannerStep::Pending(_) => (R11StepKind::Pending, None, None),
            PlannerStep::Succeeded => (R11StepKind::Succeeded, None, Some("unexpected arrival at fixed start".to_owned())),
            PlannerStep::GroundArrival(_) => return Err("Ground arrival is unsupported in this Body-only qualification".into()),
            PlannerStep::Failed(e) => (R11StepKind::Failed, None, Some(format!("planner failed: {e}"))),
            PlannerStep::ReplanRequired(e) => (R11StepKind::ReplanRequired, None, Some(format!("planner requested global replan: {e}"))),
        };
        let bound = f.take_command_dispatch_bound();
        let expired_at_return = bound.is_some_and(|b| b.monotonic_deadline().is_none_or(|deadline| returned > deadline));
        let expired_at_observation = bound.is_some_and(|b| b.expired_now());
        let pose_stamp_ns = f.pose.as_ref().ok_or("missing current pose")?.stamp.nanos;
        let sample = R11ControlSample { due, entered, returned, kind,
            admission: f.admitted_generation, generation: f.route_generation, plans: f.local_plans,
            tickets: f.source_tickets.iter().flatten().count(), bound_present: bound.is_some(),
            bound_deadline: bound.and_then(|b| b.monotonic_deadline()), bound_expired_at_return: expired_at_return,
            bound_expired_at_observation: expired_at_observation,
            map_source_ns: self.current_source_ns, pose_stamp_ns };
        if self.damage_installed && self.first_nonzero.is_none() && !expired_at_return
            && self.origin.is_some_and(|origin| returned <= origin + R11_WINDOW) {
            if let Some(command) = nonzero {
                if f.admitted_generation != Some(f.route_generation) {
                    return Err("nonzero without current local admission".into());
                }
                self.first_nonzero = Some(R11Nonzero { at: returned, map_source_ns: self.current_source_ns,
                    pose_stamp_ns, generation: f.route_generation, admission: f.admitted_generation.unwrap(),
                    twist: command, bound_present: bound.is_some(), bound_deadline: bound.and_then(|b| b.monotonic_deadline()) });
            }
        }
        self.controls.push(sample);
        self.record_receipt(previous, returned)?;
        self.drain_events()?;
        if nonzero.is_some() && expired_at_return { return Err("nonzero already outside original dispatch deadline at step return".into()); }
        if let Some(reason) = terminal { return Err(reason); }
        Ok(())
    }
    pub fn drain_events(&mut self) -> Result<(), String> {
        while let Ok(event) = self.receive.try_recv() {
            if self.kernel.len() == EVENT_CAP { return Err("kernel observation cap exceeded".into()); }
            self.kernel.push(event);
        }
        if self.dropped.load(Ordering::Relaxed) != 0 { return Err("nonblocking kernel observation lost events".into()); }
        Ok(())
    }
    pub fn endpoints_seen(&self) -> bool {
        let Some(origin) = self.origin else { return false; };
        let cutoff = self.ended.unwrap_or(origin + R11_WINDOW).min(origin + R11_WINDOW);
        if !self.first_nonzero.as_ref().is_some_and(|command| command.at <= cutoff) { return false; }
        self.kernel.iter().any(|event| match event {
            R11KernelEvent::Entry { at, identity, source_ns } if *at <= cutoff => self.receipts.iter().any(|r| {
                r.after_damage && identity.matches_submission(&r.identity) && identity.epoch == r.epoch
                    && *source_ns == u64::try_from(r.source_ns).ok()
            }),
            _ => false,
        })
    }
    pub fn skipped_pose(&mut self, count: u64) { self.skipped_pose_due += count; }
    pub fn skipped_control(&mut self, count: u64) { self.skipped_control_due += count; }
    pub fn note_issue(&mut self, issue: String) { self.issues.push(issue); }
    pub fn end_measurement(&mut self) {
        if self.ended.is_none() {
            self.ended = Some(Instant::now());
            if let Some(f) = &self.follower { self.costs_after = f.pending_source_work_for_qualification(); }
        }
    }
    pub fn finish(mut self) -> R11Report {
        self.end_measurement();
        self.cleanup();
        if let Err(e) = self.drain_events() { self.issues.push(e); }
        let origin = self.origin.unwrap_or_else(Instant::now);
        let first_dequeue = self.first_dequeue.snapshot().map(|pop| R11FirstDequeue {
            id: pop.id, epoch: pop.epoch, before: pop.before, after: pop.after });
        R11Report { arm: self.arm, case: self.kind, origin, ended: self.ended.unwrap_or(origin), initial_id: self.initial_id, first_dequeue,
            receipts: std::mem::take(&mut self.receipts), kernel: std::mem::take(&mut self.kernel),
            maps: std::mem::take(&mut self.maps), controls: std::mem::take(&mut self.controls),
            first_nonzero: self.first_nonzero.take(), costs_before: self.costs_before, costs_after: self.costs_after,
            observer_drops: self.dropped.load(Ordering::Relaxed), skipped_pose_due: self.skipped_pose_due,
            skipped_control_due: self.skipped_control_due, issues: std::mem::take(&mut self.issues), cleanup_returned: true }
    }
    fn cleanup(&mut self) {
        if let Some(mut follower) = self.follower.take() {
            follower.stop();
            // No artificial worker gate exists to release. Original Drop owns
            // cancellation/join; returning is not an exposed independent join result.
            drop(follower);
        }
    }
}
impl Drop for R11NoHoldCase { fn drop(&mut self) { self.cleanup(); } }
