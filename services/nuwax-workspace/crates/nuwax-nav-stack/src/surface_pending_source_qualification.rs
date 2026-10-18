// Explicit Q1-R qualification only. Include in the same selective_refresh_tests module,
// after the stage39 qualification include, with the existing explicit feature.
use nuwax_nav_runtime::route_pending_source_qualification::{
    PendingSourceCase as Q1Case, KernelEntry as Q1Entry, SubmittedSource as Q1Submission};
use nuwax_nav_runtime::route_change_scope_qualification::ActualCandidate as Q1Actual;

fn q1_match_real_entry(entry:&Q1Entry,submissions:&[Q1Submission])->u64 {
    let ids:Vec<_>=submissions.iter().filter(|s|entry.matches_submission(s)).map(|s|s.id).collect();
    assert_eq!(ids.len(),1,"direct real source+pose+mode entry must map to exactly one accepted receipt");
    eprintln!("Q1_LINK compute_ordinal={} actual_request_id={} source={:p} pose_x={} detour={} epoch={}",
        entry.compute_ordinal,ids[0],Arc::as_ptr(&entry.source),entry.pose.pose.position.x,entry.detour,entry.epoch);
    ids[0]
}
fn q1_require_full(label:&str,actual:&Q1Actual,entry:&Q1Entry,progress:f64,start_x:f64) {
    let poses:Vec<_>=actual.path.poses.iter().map(|pose| {
        let p=pose.pose.position;let q=pose.pose.orientation;[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
    }).collect();
    eprintln!("Q1_CANDIDATE role={label} partial={} progress={} end={} horizon={} poses={poses:?}",
        actual.partial,actual.progress,actual.end,actual.horizon);
    assert!(!actual.partial,"fixed Q1 {label} full-candidate prerequisite failed; do not move the point/pose or change budgets");
    assert!(Arc::ptr_eq(&actual.source,&entry.source));
    assert_eq!((actual.epoch,actual.generation),(entry.epoch,entry.generation));
    assert!((actual.progress-progress).abs()<1e-6);
    assert!((actual.end-progress-5.0).abs()<1e-6 && (actual.horizon-5.0).abs()<1e-6);
    assert!(actual.path.poses.len()>1);
    let first=actual.path.poses.first().unwrap().pose.position;
    assert!((first.x-start_x).abs()<1e-6 && (first.y-1.55).abs()<1e-6);
}

// Historical old-P baseline test is preserved in commit 90666161 and the
// stage40 diagnostic archive. The pending-only v2 contract now lives in
// surface_pending_only_v2_qualification.rs under its explicit new test name.
