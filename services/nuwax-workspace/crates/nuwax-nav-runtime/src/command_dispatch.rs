//! Observation-time admissibility for starting an already checked command.
//! This is separate from the velocity lease and does not validate a newer pose.
use std::{cell::Cell,time::{Duration,Instant,SystemTime,UNIX_EPOCH}};

pub(crate) fn unix_ns()->Option<u64> {
    SystemTime::now().duration_since(UNIX_EPOCH).ok()
        .map(|value|value.as_nanos().min(u128::from(u64::MAX))as u64)
}

#[derive(Clone,Copy,Debug)]
pub(crate) struct ObservationClockAnchor {
    pub(crate) monotonic:Instant,
    pub(crate) unix_ns:u64,
}
impl ObservationClockAnchor {
    pub(crate) fn capture()->Self {
        // Reading monotonic first makes the conversion conservative, never newer
        // than the exact Unix instant used by the existing observation checks.
        let monotonic=Instant::now();
        Self {monotonic,unix_ns:unix_ns().unwrap_or(0)}
    }
}

/// Latest admissible start of one selected observation-checked trajectory.
/// It neither extends nor replaces the command's independent velocity lease.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub struct CommandDispatchBound {
    anchor_unix_ns:u64,
    latest_issue_unix_ns:u64,
    monotonic_deadline:Option<Instant>,
}
impl CommandDispatchBound {
    /// Preserve the admitted sensor's absolute time through asynchronous dispatch.
    /// A stale or invalid source produces an already-expired bound, never freshness.
    pub fn from_source_timestamp(source_timestamp_ns:u64,max_age:Duration)->Self {
        let age=max_age.as_nanos().min(u128::from(u64::MAX)) as u64;
        let anchor=ObservationClockAnchor::capture();
        let latest=nuwax_nav_types_messages::source_time::comparison_timestamp_ns_at(
            source_timestamp_ns,anchor.unix_ns).ok().map(|stamp|stamp.saturating_add(age)).unwrap_or(0);
        Self::from_anchor(anchor,latest)
    }
    pub(crate) fn earliest(self,other:Self)->Self {
        Self {
            anchor_unix_ns:self.anchor_unix_ns.max(other.anchor_unix_ns),
            latest_issue_unix_ns:self.latest_issue_unix_ns.min(other.latest_issue_unix_ns),
            monotonic_deadline:self.monotonic_deadline.zip(other.monotonic_deadline)
                .map(|(a,b)|a.min(b)),
        }
    }

    pub(crate) fn from_anchor(anchor:ObservationClockAnchor,latest_issue_unix_ns:u64)->Self {
        let monotonic_deadline=(anchor.unix_ns != 0).then_some(latest_issue_unix_ns)
            .and_then(|latest|latest.checked_sub(anchor.unix_ns))
            .and_then(|remaining|anchor.monotonic.checked_add(Duration::from_nanos(remaining)));
        Self {anchor_unix_ns:anchor.unix_ns,latest_issue_unix_ns,monotonic_deadline}
    }
    pub(crate) fn expired_at(self,now:Instant,unix:Option<u64>)->bool {
        self.monotonic_deadline.is_none_or(|deadline|now>deadline)
            || unix.is_none_or(|unix|unix<self.anchor_unix_ns || unix>self.latest_issue_unix_ns)
    }
    /// Whether either original clock qualification has expired or become invalid.
    pub fn expired_now(self)->bool {self.expired_at(Instant::now(),unix_ns())}
    /// Original absolute issue deadline; None denotes an invalid bound, never an unrestricted one.
    pub fn monotonic_deadline(self)->Option<Instant> {self.monotonic_deadline}
}

/// Fixed scalar dependencies for one invocation. Ranking may accumulate values;
/// reset exactly once before replaying the final choice, not between its tails.
pub(crate) struct ObservationProofAccumulator {
    anchor:Option<ObservationClockAnchor>,
    latest_start:Cell<Option<u64>>,
}
impl ObservationProofAccumulator {
    pub(crate) fn new(anchor:Option<ObservationClockAnchor>)->Self {Self {anchor,latest_start:Cell::new(None)}}
    pub(crate) fn reset(&self) {self.latest_start.set(None);}
    pub(crate) fn merge(&self,latest_start:Option<u64>) {
        if let Some(value)=latest_start {
            self.latest_start.set(Some(self.latest_start.get().map_or(value,|old|old.min(value))));
        }
    }
    pub(crate) fn bound(&self)->Result<Option<CommandDispatchBound>, &'static str> {
        let Some(latest)=self.latest_start.get() else {return Ok(None);};
        let anchor=self.anchor.ok_or("observation proof has no captured clock")?;
        Ok(Some(CommandDispatchBound::from_anchor(anchor,latest)))
    }
}

pub(crate) fn latest_start_for_query(latest_checked:Option<u64>,elapsed_ns:u64)->Option<u64> {
    // MAX imposes no finite upper constraint under the original saturating
    // checked-time semantics; never turn it into a false MAX-elapsed limit.
    latest_checked.filter(|value|*value<u64::MAX).map(|value|value.checked_sub(elapsed_ns).unwrap_or(0))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn source_bound_combination_preserves_both_clock_qualifications() {
        let start=Instant::now();
        let first=CommandDispatchBound::from_anchor(ObservationClockAnchor{monotonic:start,unix_ns:100},200);
        let second=CommandDispatchBound::from_anchor(ObservationClockAnchor{monotonic:start,unix_ns:150},300);
        let combined=first.earliest(second);
        assert!(!combined.expired_at(start,Some(175)));
        assert!(combined.expired_at(start,Some(125)),"newer source clock anchor cannot be discarded");
        assert!(combined.expired_at(start,Some(201)),"earlier absolute limit remains binding");
        assert!(combined.expired_at(start+Duration::from_nanos(101),Some(175)));
    }
    #[test]
    fn inclusive_deadline_and_both_clock_qualifications() {
        let start=Instant::now();let anchor=ObservationClockAnchor{monotonic:start,unix_ns:100};
        let bound=CommandDispatchBound::from_anchor(anchor,200);
        assert!(!bound.expired_at(start+Duration::from_nanos(99),Some(199)));
        assert!(!bound.expired_at(start+Duration::from_nanos(100),Some(200)));
        assert!(bound.expired_at(start+Duration::from_nanos(101),Some(200)));
        assert!(bound.expired_at(start,Some(201)));
        assert!(bound.expired_at(start,Some(99)));
        assert!(bound.expired_at(start,None));
        assert!(CommandDispatchBound::from_anchor(anchor,99).expired_at(start,Some(100)));
        let missing_clock=ObservationClockAnchor{monotonic:start,unix_ns:0};
        assert!(CommandDispatchBound::from_anchor(missing_clock,200).expired_at(start,Some(100)));
    }
    #[test]
    fn selection_reset_and_memo_hit_dependencies_are_explicit() {
        let start=Instant::now();let anchor=ObservationClockAnchor{monotonic:start,unix_ns:100};
        let proof=ObservationProofAccumulator::new(Some(anchor));
        proof.merge(Some(101)); // A rejected candidate's tight dependency.
        proof.reset();
        proof.merge(latest_start_for_query(Some(260),50));
        proof.merge(latest_start_for_query(Some(250),50)); // Cached selected query.
        proof.merge(None);
        let bound=proof.bound().unwrap().unwrap();
        assert!(!bound.expired_at(start+Duration::from_nanos(100),Some(200)));
        assert!(bound.expired_at(start+Duration::from_nanos(101),Some(201)));
        proof.reset();assert_eq!(proof.bound().unwrap(),None);
    }
    #[test]
    fn saturation_and_missing_anchor_cannot_manufacture_a_fresh_bound() {
        assert_eq!(latest_start_for_query(Some(u64::MAX),42),None);
        assert_eq!(latest_start_for_query(None,42),None);
        assert_eq!(latest_start_for_query(Some(41),42),Some(0));
        let proof=ObservationProofAccumulator::new(None);
        assert_eq!(proof.bound().unwrap(),None);
        proof.merge(Some(1));assert!(proof.bound().is_err());
    }
}
