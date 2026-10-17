//! Test-only replay of the removed duplicate projection. No alternate planner.
use super::*;
use std::cell::Cell;

thread_local! {
    static REPLAY_ORIGINAL: Cell<bool> = const { Cell::new(false) };
    static PROJECTIONS: Cell<usize> = const { Cell::new(0) };
}
pub(super) fn replay_original() -> bool { REPLAY_ORIGINAL.get() }
pub(super) fn record_projection() { PROJECTIONS.set(PROJECTIONS.get()+1); }
pub(super) fn projections() -> usize { PROJECTIONS.get() }
pub(super) fn with_original<T>(replay: bool, operation: impl FnOnce()->T) -> T {
    struct Reset(bool);
    impl Drop for Reset { fn drop(&mut self) { REPLAY_ORIGINAL.set(self.0); } }
    let _reset=Reset(REPLAY_ORIGINAL.replace(replay));
    PROJECTIONS.set(0);
    operation()
}
fn pose_bits(p:BodyPose)->([u64;3],[u64;4]) { (p.0.map(f64::to_bits),p.1.map(f64::to_bits)) }
pub(super) fn assert_same_result(a:&Result<LocalTrajectory,LocalPlanError>,b:&Result<LocalTrajectory,LocalPlanError>) {
    match (a,b) {
        (Ok(a),Ok(b)) => {
            assert_eq!(a.smoothed,b.smoothed);
            assert_eq!(a.poses.iter().copied().map(pose_bits).collect::<Vec<_>>(),
                b.poses.iter().copied().map(pose_bits).collect::<Vec<_>>());
        }
        (Err(a),Err(b)) => {
            assert_eq!(a.kind,b.kind);
            assert_eq!(a.reason,b.reason);
            assert_eq!(a.closest_pose.map(pose_bits),b.closest_pose.map(pose_bits));
            assert_eq!(a.blocked.map(|p|p.map(f64::to_bits)),b.blocked.map(|p|p.map(f64::to_bits)));
            assert_eq!(a.body_witness.map(|w|pose_bits(w.pose)),b.body_witness.map(|w|pose_bits(w.pose)));
            // Displayed rejection values must also remain byte-for-byte unchanged.
            assert_eq!(format!("{:?}",a.body_witness),format!("{:?}",b.body_witness));
        }
        _ => panic!("reuse changed outcome: {a:?} vs {b:?}"),
    }
}

#[test]
fn nonarrival_polyline_reuse_preserves_boundary_drain_and_reduces_queries() {
    let base=crate::tests::coarse_open_floor();
    let tomo=Tomogram::from_surface(&base).unwrap();
    let start=([-1.0,0.2,0.4],[0.0,0.0,0.0,1.0]);
    let reference=[[-1.0,0.2,0.0],[0.1,0.2,0.0]];
    let excluded=[[2.0,2.0],[3.0,2.0],[3.0,3.0],[2.0,3.0]];
    let mut results=Vec::new();
    for arrival in [None,Some(([0.1,0.2,0.4],0.1))] {
        results.clear();
        let mut works=Vec::new();
        #[cfg(feature="profile")]
        let mut counts=Vec::new();
        for replay in [true,false] {
            let map=base.fork_for_planning();
            local_search::reset_last_work();
            let result=with_original(replay,||plan(&map,&tomo,start,&reference,&crate::tests::motion(),
                &ScanConfig::default(),None,arrival,&excluded,true,false));
            assert_eq!(result.as_ref().unwrap_err().kind,LocalPlanFailureKind::SearchExhausted);
            assert_eq!(projections(),if replay || arrival.is_some() {2}else{1});
            works.push(local_search::last_work());
            #[cfg(feature="profile")]
            counts.push(profile::calls());
            results.push(result);
        }
        assert_same_result(&results[0],&results[1]);
        assert_eq!(works[0],works[1]);
        #[cfg(feature="profile")]
        {
            eprintln!("POLYLINE_REUSE boundary arrival={} original={:?} reuse={:?} work={:?}",
                arrival.is_some(),counts[0],counts[1],works[0]);
            if arrival.is_none() {
                assert!(counts[0].iter().zip(counts[1]).all(|(a,b)|*a>b),
                    "one rejected projection must remove view, endpoint and sweep work");
            } else { assert_eq!(counts[0],counts[1],"different arrival regions must still project separately"); }
        }
    }
}

#[test]
fn nonarrival_polyline_reuse_preserves_open_path_and_cancellation() {
    let map=crate::tests::coarse_open_floor();
    let tomo=Tomogram::from_surface(&map).unwrap();
    let start=([-1.0,0.2,0.4],[0.0,0.0,0.0,1.0]);
    let reference=[[-1.0,0.2,0.0],[0.1,0.2,0.0]];
    let run=||plan(&map,&tomo,start,&reference,&crate::tests::motion(),&ScanConfig::default(),
        None,None,&[],false,false);
    let original=with_original(true,run);
    let reused=with_original(false,run);
    assert!(original.is_ok());
    assert_same_result(&original,&reused);
    for replay in [true,false] {
        for checkpoints in [0,8,80] {
            let error=with_original(replay,||cancellation::after_checkpoints(checkpoints,run)).unwrap_err();
            assert_eq!(error.kind,LocalPlanFailureKind::Cancelled);
            assert!(error.blocked.is_none() && error.closest_pose.is_none() && error.body_witness.is_none());
        }
    }
    assert_same_result(&original,&with_original(false,run));
}

thread_local! {
    static CANCEL_AT_REUSE: std::cell::RefCell<Option<std::sync::Arc<std::sync::atomic::AtomicU64>>> = const { std::cell::RefCell::new(None) };
}
pub(super) fn before_reuse() {
    if let Some(epoch)=CANCEL_AT_REUSE.with(|slot|slot.borrow_mut().take()) {
        epoch.fetch_add(1,std::sync::atomic::Ordering::Release);
    }
}
#[test]
fn pending_cancellation_wins_over_retained_polyline_error() {
    let map=crate::tests::coarse_open_floor();
    let tomo=Tomogram::from_surface(&map).unwrap();
    let epoch=std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0));
    let token=PlanningCancellation::new(epoch.clone(),0);
    struct Reset;
    impl Drop for Reset { fn drop(&mut self) { CANCEL_AT_REUSE.with(|slot|*slot.borrow_mut()=None); } }
    let _reset=Reset;
    for reference_only in [false,true] {
    epoch.store(0,std::sync::atomic::Ordering::Release);
    CANCEL_AT_REUSE.with(|slot|*slot.borrow_mut()=Some(epoch.clone()));
    let error=with_original(false,||with_planning_cancellation(&token,|| {
        let start=([-1.0,0.2,0.4],[0.0,0.0,0.0,1.0]);
        let reference=[[-1.0,0.2,0.0],[0.1,0.2,0.0]];
        let boundary=[[2.0,2.0],[3.0,2.0],[3.0,3.0],[2.0,3.0]];
        if reference_only {
            probe_reference_candidates(&map,&tomo,start,&reference,&crate::tests::motion(),
                &ScanConfig::default(),None,None,&boundary,true,false,None)
        } else {
            plan(&map,&tomo,start,&reference,&crate::tests::motion(),&ScanConfig::default(),
                None,None,&boundary,true,false).map(ReferenceCandidateOutcome::Complete)
        } })).unwrap_err();
    assert!(CANCEL_AT_REUSE.with(|slot|slot.borrow().is_none()),"must reach reuse boundary");
    assert_eq!(error.kind,LocalPlanFailureKind::Cancelled);
    assert!(error.blocked.is_none() && error.closest_pose.is_none() && error.body_witness.is_none());
    }
}
