// Uncompiled tests-only baseline draft. Include in the SAME selective_refresh_tests
// module after the existing stage39 helpers. No production strategy change.
use nuwax_nav_runtime::route_completed_slot_qualification::PublishedKernelCase as CompletedSlotCase;

fn completed_slot_require_full(case:&CompletedSlotCase) {
    let actual=case.actual();let state=case.lifecycle();
    assert!(!actual.partial,"requires real full; never synthesize a candidate");
    assert!((actual.progress-12.0).abs()<1e-6);
    assert!(actual.end>actual.progress && actual.end<30.0);
    assert!(actual.horizon.is_finite() && actual.horizon>0.0);
    assert_eq!(actual.generation,state.generation);assert_eq!(actual.epoch,state.epoch);
    assert!(actual.path.poses.len()>1);
    let first=actual.path.poses.first().unwrap().pose.position;
    assert!((first.x-12.55).abs()<1e-6 && (first.y-1.55).abs()<1e-6);
    assert_eq!(state.calls,1);assert_eq!(state.plans,0);assert!(state.admission.is_none());
    assert_eq!(state.submitted,Some(case.id()));assert_eq!(state.ticket_ids,vec![case.id()]);
    let slot=case.slot().unwrap();
    assert!(slot.is_exact_ready_full(case.id(),state.epoch),"actual slot: {slot:?}");
    eprintln!("COMPLETED_SLOT_PREREQUISITE id={} epoch={} progress={} end={} horizon={} poses={} ready_full=true active=false pending=None mirror_exact=true no_worker_hold=true",
        case.id(),state.epoch,actual.progress,actual.end,actual.horizon,actual.path.poses.len());
}
fn completed_slot_control() {
    let setup=configured(true,true);let mut fixture=scope_floor();
    let mut cache=SurfaceGeometryCache::configured(&setup);
    let initial=scope_build(&mut cache,&mut fixture,[251,31,40]);
    let control=CompletedSlotCase::capture(setup.follower.clone(),initial.snapshot,
        scope_pose(12.55),scope_route(),scope_pose(30.55)).unwrap();
    completed_slot_require_full(&control);
    let audit=control.audit_on(control.actual().source.clone()).unwrap();
    scope_require_current_safety(&audit);assert!(audit.latest_edges_valid);
    let id=control.id();let before=control.lifecycle();
    // Exact readiness is re-observed immediately before the only consumer step.
    assert!(control.slot().unwrap().is_exact_ready_full(id,before.epoch));
    let done=control.finish_with_original_step().unwrap();
    assert!(done.cleanup_returned && done.old_ticket_absent && done.old_submitted_absent && done.old_slot_absent);
    assert_eq!(done.after_step.epoch,before.epoch);assert_eq!(done.after_step.plans,1);
    assert_eq!(done.after_step.calls,1);
    assert_eq!(done.after_step.admission,Some(before.generation));
    assert!(matches!(done.step,nuwax_nav_runtime::PlannerStep::Command(c) if c!=Twist2::ZERO),
        "original step did not produce its current checked nonzero: {:?}",done.step);
    eprintln!("COMPLETED_SLOT_CONTROL id={id} original_published_full_consumed=true original_admission=1 checked_nonzero=true cleanup_returned=true no_performance_claim=true");
}
fn completed_slot_related_baseline(cell:[i32;3]) {
    completed_slot_control(); // fresh independent real kernel, no map update
    let setup=configured(true,true);let mut fixture=scope_floor();
    let mut cache=SurfaceGeometryCache::configured(&setup);
    let initial=scope_build(&mut cache,&mut fixture,[251,31,40]);
    let mut case=CompletedSlotCase::capture(setup.follower.clone(),initial.snapshot,
        scope_pose(12.55),scope_route(),scope_pose(30.55)).unwrap();
    completed_slot_require_full(&case);
    fixture.cells.insert(cell);
    let update=scope_runtime(scope_build(&mut cache,&mut fixture,cell));
    scope_require_columns(&update,cell);
    let proof=case.proof_with_update(&update).unwrap();
    assert!(!proof.unknown,"must use the complete original ticket clone");
    assert!(!proof.permits_path(&update.snapshot,case.global_route()));
    assert!(proof.permits_path(&update.snapshot,&case.actual().path));
    let audit=case.audit_on(update.snapshot.clone()).unwrap();
    scope_require_current_safety(&audit);assert!(audit.latest_edges_valid);
    let before=case.lifecycle();let id=case.id();
    assert!(case.slot().unwrap().is_exact_ready_full(id,before.epoch));
    let begin=Instant::now();case.publish(update).unwrap();let end=Instant::now();
    let after=case.lifecycle();let slot_after=case.slot().unwrap();
    let retained=case.retained_goal_and_route();
    // Read the cleared original slot before any original step may submit again.
    assert!(slot_after.completion_id.is_none() && slot_after.pending.is_none() && !slot_after.active);
    assert!(!slot_after.closed && !slot_after.faulted);
    assert_eq!(slot_after.current_epoch,after.epoch);
    assert_eq!(after.epoch,before.epoch+1);
    assert!(after.submitted.is_none() && after.ticket_ids.is_empty());
    assert_eq!((after.calls,after.plans,after.generation,after.progress),
        (before.calls,before.plans,before.generation,before.progress));
    assert!(after.retry_after>=begin && after.retry_after<=end);assert!(retained);
    let done=case.finish_with_original_step().unwrap();
    assert!(done.cleanup_returned && done.old_ticket_absent && done.old_submitted_absent && done.old_slot_absent);
    assert_eq!(done.after_step.plans,0,"original C must not be admitted after retirement");
    assert_eq!(done.after_step.calls,2,"original retry-now should submit latest inputs");
    assert!(done.after_step.admission.is_none());
    assert!(matches!(done.step,nuwax_nav_runtime::PlannerStep::Pending(_)),"{:?}",done.step);
    eprintln!("COMPLETED_SLOT_BASELINE cell={cell:?} id={id} actual_ready_before_update=true full_candidate_original_ticket_allowed=true latest_all_edges_body_connector_stop_valid=true original_epoch_bumped=true original_slot_cleared=true original_admission=0 next_original_submit=1 cleanup_returned=true no_performance_claim=true");
}
#[test]
#[ignore="explicit original published-completion fact MRE, fixed M1 input, not a benchmark"]
fn completed_slot_m1_published_full_lost_on_historical_column() {
    completed_slot_related_baseline(SCOPE_PREFIX);
}
#[test]
#[ignore="explicit original published-completion fact MRE, fixed M2 input, not a benchmark"]
fn completed_slot_m2_published_full_lost_on_distant_future_column() {
    completed_slot_related_baseline(SCOPE_FUTURE);
}

fn completed_slot_require_combined_columns(update:&RuntimeUpdate,cells:&[[i32;3]]) {
    let SurfaceGeometryChanges::Columns{fine_size,fine,planner_size,coarse}=update.changes.as_ref()
        else {panic!("combined actual Writer update must be complete Columns")};
    assert_eq!(fine_size.to_bits(),update.snapshot.source_voxel_size_m().to_bits());
    assert_eq!(planner_size.to_bits(),update.snapshot.config().voxel_size.to_bits());
    for cell in cells {assert!(update.snapshot.observations().unwrap().get(*cell).is_some());}
    let mut expected_fine:Vec<_>=cells.iter().map(|cell|[cell[0],cell[1]]).collect();
    let mut expected_coarse:Vec<_>=cells.iter().map(|cell| {
        let xyz=cell.map(|v|((f64::from(v)+0.5)*f64::from(*fine_size))as f32);
        let q=xyz.map(|v|(v / *planner_size).floor()as i32);[q[0],q[1]]
    }).collect();
    expected_fine.sort();expected_fine.dedup();expected_coarse.sort();expected_coarse.dedup();
    let mut got_fine=fine.clone();let mut got_coarse=coarse.clone();got_fine.sort();got_coarse.sort();
    assert_eq!(got_fine,expected_fine);assert_eq!(got_coarse,expected_coarse);
    eprintln!("COMPLETED_SLOT_COMBINED_WRITER fixed_cells={cells:?} fine_size={fine_size} fine={fine:?} planner_size={planner_size} coarse={coarse:?}");
}
#[test]
#[ignore="actual ready full with fixed prefix plus original detour hit; do not move points to pass"]
fn completed_slot_m3_published_full_related_actual_hit_is_retired() {
    let setup=configured(true,true);let mut fixture=scope_floor();
    let mut cache=SurfaceGeometryCache::configured(&setup);
    let _floor=scope_build(&mut cache,&mut fixture,[251,31,40]);
    fixture.cells.extend((280..=283).flat_map(|x|(0..=42).flat_map(move |y|(44..=54).map(move |z|[x,y,z]))));
    let walled=scope_build(&mut cache,&mut fixture,[280,0,44]);
    let mut case=CompletedSlotCase::capture(setup.follower.clone(),walled.snapshot,
        scope_pose(12.55),scope_route(),scope_pose(30.55)).unwrap();
    completed_slot_require_full(&case);
    fixture.cells.insert(SCOPE_PREFIX);fixture.cells.insert(SCOPE_DETOUR_HIT);
    let update=scope_runtime(scope_build(&mut cache,&mut fixture,SCOPE_DETOUR_HIT));
    completed_slot_require_combined_columns(&update,&[SCOPE_PREFIX,SCOPE_DETOUR_HIT]);
    let proof=case.proof_with_update(&update).unwrap();assert!(!proof.unknown);
    assert!(!proof.permits_path(&update.snapshot,case.global_route()),"prefix must trigger original related cancel");
    assert!(!proof.permits_path(&update.snapshot,&case.actual().path),"actual candidate scope must be hit");
    let audit=case.audit_on(update.snapshot.clone()).unwrap();scope_require_current_safety(&audit);
    assert!(!audit.latest_edges_valid && audit.first_new_edge_rejection.is_some(),
        "bbox overlap alone is not an actual physical hit");
    eprintln!("COMPLETED_SLOT_FIXED_ACTUAL_HIT first_new_edge_rejection={:?}",audit.first_new_edge_rejection);
    let before=case.lifecycle();let id=case.id();
    assert!(case.slot().unwrap().is_exact_ready_full(id,before.epoch));
    let begin=Instant::now();case.publish(update).unwrap();let end=Instant::now();
    let after=case.lifecycle();let slot_after=case.slot().unwrap();
    assert_eq!(after.epoch,before.epoch+1);assert!(after.submitted.is_none() && after.ticket_ids.is_empty());
    assert!(slot_after.completion_id.is_none() && slot_after.pending.is_none() && !slot_after.active);
    assert!(!slot_after.closed && !slot_after.faulted && slot_after.current_epoch==after.epoch);
    assert_eq!((after.calls,after.plans,after.generation,after.progress),
        (before.calls,before.plans,before.generation,before.progress));
    assert!(after.retry_after>=begin && after.retry_after<=end);assert!(case.retained_goal_and_route());
    let done=case.finish_with_original_step().unwrap();
    assert!(done.cleanup_returned && done.old_ticket_absent && done.old_submitted_absent && done.old_slot_absent);
    assert_eq!(done.after_step.plans,0);assert_eq!(done.after_step.calls,2);assert!(done.after_step.admission.is_none());
    assert!(matches!(done.step,nuwax_nav_runtime::PlannerStep::Pending(_)),"{:?}",done.step);
    eprintln!("COMPLETED_SLOT_HIT_BASELINE id={id} actual_ready_before_update=true fixed_prefix_and_hit=true original_ticket_denied=true actual_full_edge_hit=true current_body_connector_stop_valid=true original_epoch_bumped=true original_slot_cleared=true original_admission=0 cleanup_returned=true no_performance_claim=true");
}
