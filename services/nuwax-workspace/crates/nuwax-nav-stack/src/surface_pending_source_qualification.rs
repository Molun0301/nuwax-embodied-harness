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

#[test]
#[ignore="explicit Q1 current-source causality; fixed M3 point/poses, not a benchmark"]
fn scope_q1_real_old_pending_delays_latest_source_after_disjoint_update() {
    let setup=configured(true,true);
    let mut fixture=scope_floor();
    let mut cache=SurfaceGeometryCache::configured(&setup);
    let _floor=scope_build(&mut cache,&mut fixture,[251,31,40]);
    fixture.cells.extend((280..=283).flat_map(|x|(0..=42).flat_map(move |y|(44..=54).map(move |z|[x,y,z]))));
    let s0=scope_build(&mut cache,&mut fixture,[280,0,44]).snapshot;
    let mut case=Q1Case::start(setup.follower.clone(),s0.clone(),scope_pose(12.55),scope_route(),scope_pose(30.55)).unwrap();
    let a_entry=case.entered().unwrap();
    let a_id=q1_match_real_entry(&a_entry,case.submissions());
    assert!(Arc::ptr_eq(&a_entry.source,&s0));
    let a_done=case.computed().unwrap();
    assert_eq!(q1_match_real_entry(&a_done.entry,case.submissions()),a_id);
    assert_eq!(a_done.entry.compute_ordinal,a_entry.compute_ordinal);
    let a=a_done.actual.unwrap_or_else(|e|panic!("fixed A real kernel failed: {e:?}"));
    q1_require_full("A",&a,&a_entry,12.0,12.55);
    let a_state=case.state();
    assert_eq!(a_state.calls,1);
    assert!(a_state.admission.is_none() && a_state.plans==0);
    assert!(!a_entry.detour && case.current_body_ok());

    // Same cells, real newer Writer publication. Snapshot identity is separately
    // observed; the Empty ticket chain is not used as a request-source label.
    let s1=scope_runtime(scope_build(&mut cache,&mut fixture,[271,31,40]));
    assert!(matches!(s1.changes.as_ref(),SurfaceGeometryChanges::Empty));
    assert!(nuwax_nav_pct::same_source_geometry(&s0,&s1.snapshot));
    assert!(!Arc::ptr_eq(&s0,&s1.snapshot),"fixture did not retain a distinct observation publication");
    case.publish(s1.clone()).unwrap();
    let p_pose=scope_pose(13.55);
    let distance=p_pose.pose.position.distance(a_entry.pose.pose.position);
    assert!((case.refresh_distance()-1.0).abs()<1e-12 && distance>=case.refresh_distance().max(0.25));
    case.update_pose_with_body_check(p_pose.clone()).unwrap();
    assert!(case.current_body_ok());
    let p_tick=case.tick_after_original_retry().unwrap();
    assert!(matches!(p_tick,nuwax_nav_runtime::PlannerStep::Pending(_)));
    let pending=case.state();
    assert_eq!((pending.calls,pending.plans,pending.epoch),(2,0,a_state.epoch));
    let p_id=pending.submitted.unwrap().0;
    assert_ne!(p_id,a_id);
    assert_eq!(pending.tickets.iter().map(|t|t.id).collect::<std::collections::BTreeSet<_>>(),
        std::collections::BTreeSet::from([a_id,p_id]));
    let p_submission=case.submissions().iter().find(|s|s.id==p_id).unwrap();
    assert!(Arc::ptr_eq(&p_submission.source,&s1.snapshot));
    assert!(!p_submission.detour);
    assert_eq!(p_submission.pose,p_pose);

    // This is the exact stage39 M3 delta. No pose/mode changes occur after it.
    fixture.cells.insert(SCOPE_DETOUR_HIT);
    let s2=scope_runtime(scope_build(&mut cache,&mut fixture,SCOPE_DETOUR_HIT));
    scope_require_columns(&s2,SCOPE_DETOUR_HIT);
    let mut a_proof=RuntimeTicket::new(Some(a.source.clone()));
    a_proof.observe(&s1.snapshot,s1.change_basis.as_deref(),&s1.changes);
    a_proof.observe(&s2.snapshot,s2.change_basis.as_deref(),&s2.changes);
    assert!(!a_proof.unknown && a_proof.permits_path(&s2.snapshot,&scope_route()));
    assert!(!a_proof.permits_path(&s2.snapshot,&a.path));
    let a_edges=case.audit_edges(&a,&s2.snapshot);
    assert!(a_edges.old_all_valid && a_edges.latest_first_rejection.is_some());
    eprintln!("Q1_PHYSICAL_REJECTION role=A {:?}",a_edges.latest_first_rejection);
    case.publish(s2.clone()).unwrap();
    assert!(case.current_body_ok(),"latest current body must pass before blaming the queue");
    let after_update=case.state();
    assert_eq!(after_update.epoch,a_state.epoch);
    assert_eq!(after_update.retry_after,pending.retry_after);
    assert_eq!(after_update.submitted,pending.submitted);
    assert_eq!(after_update.pose,p_pose);
    assert!(Arc::ptr_eq(&after_update.map,&s2.snapshot));
    assert!(after_update.dirty && after_update.admission.is_none());
    assert_eq!(after_update.tickets.len(),2);
    for ticket in &after_update.tickets {
        assert!(!ticket.unknown && ticket.columns>0 && ticket.batches==1);
        assert!(ticket.basis.as_ref().is_some_and(|basis|Arc::ptr_eq(basis,&s2.snapshot)));
    }
    assert!(matches!(case.tick_after_original_retry().unwrap(),nuwax_nav_runtime::PlannerStep::Pending(_)));
    let after_deadline=case.state();
    assert!(Instant::now()>=pending.retry_after && after_deadline.dirty);
    assert_eq!(after_deadline.calls,2);
    assert_eq!(after_deadline.submitted,pending.submitted);
    assert_eq!(after_deadline.pose,p_pose);
    eprintln!("Q1_AFTER_DEADLINE latest=S2 pending_request=S1 pending_id={p_id} calls=2 unchanged_pose=true unchanged_mode=true body_valid=true ticket_basis=S2");

    case.release_actual(a_entry.compute_ordinal).unwrap();
    case.pump_until_consumed(a_id).unwrap();
    let after_a=case.state();
    assert_eq!((after_a.calls,after_a.plans,after_a.epoch),(2,0,a_state.epoch));
    assert_eq!(after_a.submitted.unwrap().0,p_id);
    assert_eq!(after_a.tickets.iter().map(|t|t.id).collect::<Vec<_>>(),vec![p_id]);
    assert!(after_a.admission.is_none());
    // take notifies the original owner before A's proof check finishes. The P
    // entry event may already exist; do not assert it follows tick/consume return.
    let p_entry=case.entered().unwrap();
    assert_eq!(q1_match_real_entry(&p_entry,case.submissions()),p_id);
    assert!(Arc::ptr_eq(&p_entry.source,&s1.snapshot));
    assert!(!Arc::ptr_eq(&p_entry.source,&s2.snapshot));
    assert!(!nuwax_nav_pct::same_source_geometry(&p_entry.source,&s2.snapshot));
    let p_done=case.computed().unwrap();
    assert_eq!(q1_match_real_entry(&p_done.entry,case.submissions()),p_id);
    assert_eq!(p_done.entry.compute_ordinal,p_entry.compute_ordinal);
    let p=p_done.actual.unwrap_or_else(|e|panic!("fixed P real kernel failed: {e:?}"));
    q1_require_full("P",&p,&p_entry,13.0,13.55);
    let mut p_proof=RuntimeTicket::new(Some(p.source.clone()));
    p_proof.observe(&s2.snapshot,s2.change_basis.as_deref(),&s2.changes);
    assert!(!p_proof.unknown && p_proof.permits_path(&s2.snapshot,&scope_route()));
    assert!(!p_proof.permits_path(&s2.snapshot,&p.path),"fixed S2 point missed actual P scope; do not move it");
    let p_edges=case.audit_edges(&p,&s2.snapshot);
    assert!(p_edges.old_all_valid && p_edges.latest_first_rejection.is_some(),
        "old source alone is not harmful delay: fixed point must physically invalidate the real full P");
    eprintln!("Q1_PHYSICAL_REJECTION role=P {:?}",p_edges.latest_first_rejection);
    assert!(case.current_body_ok());
    assert_eq!(case.state().calls,2);

    case.release_actual(p_entry.compute_ordinal).unwrap();
    case.pump_until_consumed(p_id).unwrap();
    let after_p=case.state();
    assert_eq!((after_p.calls,after_p.plans,after_p.epoch),(3,0,a_state.epoch));
    assert!(after_p.admission.is_none() && after_p.generation==a_state.generation);
    let r_id=after_p.submitted.unwrap().0;
    assert_ne!(r_id,p_id);assert_ne!(r_id,a_id);
    assert_eq!(after_p.tickets.iter().map(|t|t.id).collect::<Vec<_>>(),vec![r_id]);
    let r_entry=case.entered().unwrap();
    assert_eq!(q1_match_real_entry(&r_entry,case.submissions()),r_id);
    assert!(Arc::ptr_eq(&r_entry.source,&s2.snapshot));
    assert_eq!(r_entry.pose,p_pose);
    assert!(!r_entry.detour);
    // Observe the real R result for clean release, but the Q1 assertion is its
    // actual source at start. Do not require/claim R admission or a motion result.
    let r_done=case.computed().unwrap();
    assert_eq!(q1_match_real_entry(&r_done.entry,case.submissions()),r_id);
    match &r_done.actual {
        Ok(actual)=>eprintln!("Q1_R_RESULT source=S2 partial={} poses={}",actual.partial,actual.path.poses.len()),
        Err(errors)=>eprintln!("Q1_R_RESULT source=S2 errors={errors:?}"),
    }
    case.release_actual(r_entry.compute_ordinal).unwrap();
    let final_state=case.finish(); // stop/release/original Drop before final summary
    assert_eq!((final_state.calls,final_state.plans,final_state.epoch),(3,0,a_state.epoch));
    eprintln!("Q1_BASELINE A_source=S0 P_source=S1 latest=S2 old_P_real_compute=true A_and_P_full_physically_invalidated=true latest_R_started_only_after_P_consumed=true cleanup_returned=true first_candidate_only=true no_performance_claim=true");
}
