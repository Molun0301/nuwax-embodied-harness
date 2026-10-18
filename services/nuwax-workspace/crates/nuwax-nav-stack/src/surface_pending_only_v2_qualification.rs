// New pending-only v2 real Writer/PlanKernel qualification tests.
// Include after existing stage39, stage40 and R11 helpers in the SAME module.
// Existing M1/M2/M3 selectors remain byte-for-byte intact; old Q1 is explicitly migrated.
// Held semantic/precheck evidence only; no performance claims; NOT RUN.

#[test]
#[ignore="explicit Q1 current-source causality; fixed M3 point/poses, not a benchmark"]
// New pending-only v2 contract; old-P baseline selector is explicitly removed from B, not left as a failing ignored test.
fn pending_only_v2_q1_replaces_before_take_and_admits_real_latest_p2() {
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
    assert_eq!(after_update.epoch,a_state.epoch,
        "original affected predicate, including local path/Unknown, must preserve this fixed Q1 epoch");
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
    // Actual normal step, while A is still held: P1 is replaced before any
    // possible take(A). No direct consume and no retry deadline assignment.
    let work_before=case.pending_source_work();
    assert!(matches!(case.tick().unwrap(),nuwax_nav_runtime::PlannerStep::Pending(_)));
    let work_after=case.pending_source_work();
    assert_eq!((work_after.0-work_before.0,work_after.1-work_before.1),(1,1));
    let replaced=case.state();
    let p2_id=replaced.submitted.unwrap().0;
    assert_eq!(p2_id,p_id+1);
    assert_eq!((replaced.calls,replaced.plans,replaced.epoch),(3,0,a_state.epoch));
    assert_eq!(replaced.retry_after,pending.retry_after);
    assert_eq!(replaced.pose,p_pose);
    assert_eq!(replaced.tickets.iter().map(|t|t.id).collect::<Vec<_>>(),vec![a_id,p2_id]);
    let before_a=after_update.tickets.iter().find(|t|t.id==a_id).unwrap();
    let kept_a=replaced.tickets.iter().find(|t|t.id==a_id).unwrap();
    assert_eq!((kept_a.columns,kept_a.batches,kept_a.unknown),
        (before_a.columns,before_a.batches,before_a.unknown));
    assert!(Arc::ptr_eq(kept_a.basis.as_ref().unwrap(),before_a.basis.as_ref().unwrap()));
    assert_eq!(kept_a.changes.len(),before_a.changes.len());
    for (before,after) in before_a.changes.iter().zip(&kept_a.changes) {assert!(Arc::ptr_eq(before,after));}
    let p2_ticket=replaced.tickets.iter().find(|t|t.id==p2_id).unwrap();
    assert_eq!((p2_ticket.columns,p2_ticket.batches,p2_ticket.unknown),(0,0,false));
    assert!(Arc::ptr_eq(p2_ticket.basis.as_ref().unwrap(),&s2.snapshot));
    let p2_submission=case.submissions().iter().find(|s|s.id==p2_id).unwrap();
    assert!(Arc::ptr_eq(&p2_submission.source,&s2.snapshot));
    assert_eq!(p2_submission.pose,p_pose);
    assert!(!p2_submission.detour);
    assert!(case.no_entry_ready(),"neither P1 nor P2 may start while A is held");
    eprintln!("PENDING_ONLY_V2_Q1_REPLACE retired_p1={p_id} latest_p2={p2_id} source=S2 before_take=true retry_unchanged=true a_chain_kept=true work_delta=(1,1)");

    case.release_actual(a_entry.compute_ordinal).unwrap();
    case.pump_until_consumed(a_id).unwrap();
    let after_a=case.state();
    assert_eq!((after_a.calls,after_a.plans,after_a.epoch),(3,0,a_state.epoch));
    assert_eq!(after_a.submitted.unwrap().0,p2_id);
    assert_eq!(after_a.tickets.iter().map(|t|t.id).collect::<Vec<_>>(),vec![p2_id]);
    assert!(after_a.admission.is_none(),"damaged A cannot restore admission");
    let p2_entry=case.entered().unwrap();
    assert_eq!(p2_entry.compute_ordinal,a_entry.compute_ordinal+1,
        "single original worker must start P2 directly, with no P1 compute in between");
    assert_eq!(q1_match_real_entry(&p2_entry,case.submissions()),p2_id);
    assert!(Arc::ptr_eq(&p2_entry.source,&s2.snapshot));
    assert!(!Arc::ptr_eq(&p2_entry.source,&s1.snapshot));
    assert_eq!(p2_entry.pose,p_pose);assert!(!p2_entry.detour);
    let p2_done=case.computed().unwrap();
    assert_eq!(q1_match_real_entry(&p2_done.entry,case.submissions()),p2_id);
    assert_eq!(p2_done.entry.compute_ordinal,p2_entry.compute_ordinal);
    let p2=p2_done.actual.unwrap_or_else(|e|panic!("fixed P2 latest-source kernel failed: {e:?}"));
    q1_require_full("P2_NEW_REAL_RESULT",&p2,&p2_entry,13.0,13.55);
    // This actual P2 result, not the historical R=514 observation, supplies all
    // full/path/current connector/braking/admission evidence for this selector.
    let p2_proof=RuntimeTicket::new(Some(p2.source.clone()));
    assert!(p2_proof.permits_path(&s2.snapshot,&p2.path));
    let audit=case.audit_current_candidate(&p2,s2.snapshot.clone()).unwrap();
    scope_require_current_safety(&audit);assert!(audit.latest_edges_valid);
    assert_eq!(case.state().plans,0);
    case.release_actual(p2_entry.compute_ordinal).unwrap();
    let step=case.pump_until_consumed_with_step(p2_id).unwrap();
    let after_p2=case.state();
    assert_eq!((after_p2.calls,after_p2.plans,after_p2.epoch),(3,1,a_state.epoch));
    assert_eq!(after_p2.admission,Some(after_p2.generation));
    assert!(after_p2.tickets.is_empty() && after_p2.submitted.is_none());
    assert!(matches!(step,nuwax_nav_runtime::PlannerStep::Command(c) if c!=Twist2::ZERO),
        "real P2 must supply current nonzero command, got {step:?}");
    assert!(case.no_entry_ready(),"no retired P1 or duplicate latest work may start");
    let final_state=case.finish();
    assert_eq!((final_state.calls,final_state.plans,final_state.epoch),(3,1,a_state.epoch));
    eprintln!("PENDING_ONLY_V2_Q1 A_source=S0 retired_P1_source=S1 P1_starts=0 P2_source=S2 actual_kernel_starts=2 accepted=3 replaced=1 A_rejected=true P2_full=true P2_current_safe=true P2_admitted=1 P2_nonzero=true cleanup_returned=true no_performance_claim=true");
}

// TESTS-ONLY pending-only v2 companion; not run. Include in surface_selective_refresh_tests
// after the stage39 and stage40 qualification includes, under their same feature.
// Genuine FineFixture Writer/cache publications and original PlanKernel. Held
// F-layer causality only; no artificial wait is a production timing sample.
fn pending_only_v2_writer_history(readd:bool) {
    let setup=configured(true,true);
    let mut fixture=scope_floor();
    let mut cache=SurfaceGeometryCache::configured(&setup);
    let _floor=scope_build(&mut cache,&mut fixture,[251,31,40]);
    fixture.cells.extend((280..=283).flat_map(|x|(0..=42).flat_map(move |y|(44..=54).map(move |z|[x,y,z]))));
    let s0=scope_build(&mut cache,&mut fixture,[280,0,44]).snapshot;
    let mut case=Q1Case::start(setup.follower.clone(),s0.clone(),scope_pose(12.55),scope_route(),scope_pose(30.55)).unwrap();
    let a_entry=case.entered().unwrap();
    let a_id=q1_match_real_entry(&a_entry,case.submissions());
    let a_done=case.computed().unwrap();
    assert_eq!(q1_match_real_entry(&a_done.entry,case.submissions()),a_id);
    let a=a_done.actual.unwrap();
    q1_require_full("DELETE_A",&a,&a_entry,12.0,12.55);
    assert_eq!(a.path.poses.len(),448,"fixed original stage39/40 A geometry must not drift");
    let epoch=case.state().epoch;
    let s1=scope_runtime(scope_build(&mut cache,&mut fixture,[271,31,40]));
    assert!(matches!(s1.changes.as_ref(),SurfaceGeometryChanges::Empty));
    case.publish(s1.clone()).unwrap();
    case.update_pose_with_body_check(scope_pose(13.55)).unwrap();
    case.tick_after_original_retry().unwrap();
    let p1=case.state().submitted.unwrap().0;
    assert_eq!(case.state().calls,2);
    let deadline=case.state().retry_after;
    fixture.cells.insert(SCOPE_DETOUR_HIT);
    let s2=scope_runtime(scope_build(&mut cache,&mut fixture,SCOPE_DETOUR_HIT));
    scope_require_columns(&s2,SCOPE_DETOUR_HIT);
    case.publish(s2.clone()).unwrap();
    assert_eq!(case.state().epoch,epoch,"original affected must leave the fixed Q1 Writer update disjoint");
    let work=case.pending_source_work();
    case.tick().unwrap();
    let after=case.pending_source_work();assert_eq!((after.0-work.0,after.1-work.1),(1,1));
    let p2=case.state().submitted.unwrap().0;
    assert_eq!(p2,p1+1);assert_eq!(case.state().retry_after,deadline);
    assert_eq!(case.state().tickets.iter().find(|t|t.id==a_id).unwrap().batches,1);
    assert!(case.no_entry_ready());

    // Real deletion, not a synthetic Empty/revision reversal. The cells after
    // deletion match S0 numerically; source identity is observed separately.
    assert!(fixture.cells.remove(&SCOPE_DETOUR_HIT));
    let s3=scope_runtime(scope_build(&mut cache,&mut fixture,[271,31,40]));
    assert!(s3.snapshot.observations().unwrap().get(SCOPE_DETOUR_HIT).is_none());
    let (SurfaceGeometryChanges::Columns {fine_size:asize,fine:af,planner_size:aps,coarse:ac},
         SurfaceGeometryChanges::Columns {fine_size:dsize,fine:df,planner_size:dps,coarse:dc})=
        (s2.changes.as_ref(),s3.changes.as_ref()) else {panic!("actual delete must report complete Columns")};
    assert_eq!(asize.to_bits(),dsize.to_bits());assert_eq!(aps.to_bits(),dps.to_bits());
    assert_eq!(af,df);assert_eq!(ac,dc);
    assert_eq!(df,&vec![[283,47]]);assert_eq!(dc,&vec![[141,23]]);
    assert_ne!(asize.to_bits(),aps.to_bits(),"fine/coarse indices use different units");
    let mut proof=RuntimeTicket::new(Some(a.source.clone()));
    for update in [&s1,&s2,&s3] {proof.observe(&update.snapshot,update.change_basis.as_deref(),&update.changes);}
    assert!(!proof.unknown && proof.changes.len()==2);
    assert!(!proof.permits_path(&s3.snapshot,&a.path),"deletion must not erase the intermediate on-A change");
    let restored=case.audit_edges(&a,&s3.snapshot);
    assert!(restored.old_all_valid && restored.latest_first_rejection.is_none(),
        "physical restoration and conservative accumulated proof are separate facts");
    case.publish(s3.clone()).unwrap();
    assert_eq!(case.state().epoch,epoch,"original affected must leave the fixed Q1 Writer update disjoint");
    let before_replace=case.state();
    let old_a=before_replace.tickets.iter().find(|t|t.id==a_id).unwrap();
    let work=case.pending_source_work();case.tick().unwrap();
    let after=case.pending_source_work();assert_eq!((after.0-work.0,after.1-work.1),(1,1));
    let latest=case.state();let p3=latest.submitted.unwrap().0;
    assert_eq!(p3,p2+1);assert_eq!(latest.calls,4);assert_eq!(latest.epoch,epoch);
    assert_eq!(latest.retry_after,deadline);
    assert_eq!(latest.tickets.iter().map(|t|t.id).collect::<Vec<_>>(),vec![a_id,p3]);
    let new_a=latest.tickets.iter().find(|t|t.id==a_id).unwrap();
    assert_eq!((new_a.columns,new_a.batches,new_a.unknown),(old_a.columns,old_a.batches,old_a.unknown));
    assert!(Arc::ptr_eq(new_a.basis.as_ref().unwrap(),old_a.basis.as_ref().unwrap()));
    for (before,after) in old_a.changes.iter().zip(&new_a.changes) {assert!(Arc::ptr_eq(before,after));}
    let p3_proof=latest.tickets.iter().find(|t|t.id==p3).unwrap();
    assert_eq!((p3_proof.columns,p3_proof.batches,p3_proof.unknown),(0,0,false));
    assert!(Arc::ptr_eq(p3_proof.basis.as_ref().unwrap(),&s3.snapshot));
    assert!(case.no_entry_ready(),"P1 and P2 must remain never-start");
    let (final_id,final_source,expected_calls)=if readd {
        // Second selector freezes the same add/delete/add sequence. It also
        // covers deletion followed by addition, without moving any hit point.
        fixture.cells.insert(SCOPE_DETOUR_HIT);
        let s4=scope_runtime(scope_build(&mut cache,&mut fixture,SCOPE_DETOUR_HIT));
        scope_require_columns(&s4,SCOPE_DETOUR_HIT);
        case.publish(s4.clone()).unwrap();
    assert_eq!(case.state().epoch,epoch,"original affected must leave the fixed Q1 Writer update disjoint");
        let before=case.state();
        let work=case.pending_source_work();case.tick().unwrap();
        let after=case.pending_source_work();assert_eq!((after.0-work.0,after.1-work.1),(1,1));
        let replaced=case.state();let p4=replaced.submitted.unwrap().0;
        assert_eq!(p4,p3+1);assert_eq!(replaced.calls,5);assert_eq!(replaced.retry_after,deadline);
        assert_eq!(replaced.tickets.iter().map(|t|t.id).collect::<Vec<_>>(),vec![a_id,p4]);
        let old_a=before.tickets.iter().find(|t|t.id==a_id).unwrap();
        let kept_a=replaced.tickets.iter().find(|t|t.id==a_id).unwrap();
        assert_eq!((kept_a.columns,kept_a.batches,kept_a.unknown),(old_a.columns,3,false));
        assert!(Arc::ptr_eq(kept_a.basis.as_ref().unwrap(),old_a.basis.as_ref().unwrap()));
        for (a,b) in old_a.changes.iter().zip(&kept_a.changes) {assert!(Arc::ptr_eq(a,b));}
        let new=replaced.tickets.iter().find(|t|t.id==p4).unwrap();
        assert_eq!((new.columns,new.batches,new.unknown),(0,0,false));
        assert!(Arc::ptr_eq(new.basis.as_ref().unwrap(),&s4.snapshot));
        assert!(case.no_entry_ready());
        (p4,s4.snapshot,5)
    } else {(p3,s3.snapshot.clone(),4)};
    case.release_actual(a_entry.compute_ordinal).unwrap();
    case.pump_until_consumed(a_id).unwrap();
    assert_eq!(case.state().plans,0,"latest restored geometry cannot wash A's historical proof");
    assert!(case.state().admission.is_none());
    let entry=case.entered().unwrap();
    assert_eq!(entry.compute_ordinal,a_entry.compute_ordinal+1);
    assert_eq!(q1_match_real_entry(&entry,case.submissions()),final_id);
    assert!(Arc::ptr_eq(&entry.source,&final_source));
    let computed=case.computed().unwrap();
    assert_eq!(q1_match_real_entry(&computed.entry,case.submissions()),final_id);
    let actual=computed.actual.unwrap_or_else(|e|panic!("real latest-source request failed: {e:?}"));
    q1_require_full("WRITER_HISTORY_LATEST_P",&actual,&entry,13.0,13.55);
    let audit=case.audit_current_candidate(&actual,final_source).unwrap();
    scope_require_current_safety(&audit);assert!(audit.latest_edges_valid);
    case.release_actual(entry.compute_ordinal).unwrap();
    let step=case.pump_until_consumed_with_step(final_id).unwrap();
    assert!(matches!(step,nuwax_nav_runtime::PlannerStep::Command(c) if c!=Twist2::ZERO),"{step:?}");
    assert!(case.no_entry_ready());
    let done=case.finish();
    assert_eq!((done.calls,done.plans,done.epoch),(expected_calls,1,epoch));
    assert!(done.tickets.is_empty());assert_eq!(done.admission,Some(done.generation));
    eprintln!("PENDING_ONLY_V2_WRITER_HISTORY add_then_delete=true readd={readd} fine=(283,47) coarse=(141,23) A_geometry_restored_at_S3=true A_historical_proof_rejected=true retired_pending_starts=0 latest_request={final_id} latest_full=true latest_admitted=1 actual_kernel_starts=2 accepted={expected_calls} replaced={} no_performance_claim=true",expected_calls-2);
}

#[test]
#[ignore="explicit real Writer add/delete history and latest-pending qualification"]
fn pending_only_v2_real_writer_delete_keeps_a_history_and_runs_only_latest_p3() {
    pending_only_v2_writer_history(false);
}
#[test]
#[ignore="explicit real Writer delete/add history and latest-pending qualification"]
fn pending_only_v2_real_writer_delete_then_add_keeps_all_batches_and_only_latest_p4() {
    pending_only_v2_writer_history(true);
}

#[test]
#[ignore = "R11 combined fixed-point geometry precheck; real kernel hold only here, never timing"]
fn pending_only_v2_combined_writer_damage_geometry_precheck() {
    let setup = configured(true, true);
    let mut prepared = r11_prepare_source(&setup).unwrap();
    let mut case = HeldKernelCase::capture(setup.follower.clone(), prepared.initial.update.snapshot.clone(),
        scope_pose(12.55), scope_route(), scope_pose(30.55)).unwrap();
    scope_require_full_current_window(&case);
    assert!(Arc::ptr_eq(&case.actual().source, &prepared.initial.update.snapshot));
    let proof = scope_proof(&case, &prepared.damage.update);
    assert!(!proof.unknown);
    assert!(!proof.permits_path(&prepared.damage.update.snapshot, case.global_route()),
        "combined fixture must trigger the ORIGINAL global-related cancellation");
    assert!(!proof.permits_path(&prepared.damage.update.snapshot, &case.actual().path));
    let audit = case.audit_on(prepared.damage.update.snapshot.clone()).unwrap();
    scope_require_current_safety(&audit);
    assert!(!audit.latest_edges_valid && audit.first_new_edge_rejection.is_some(),
        "combined fixture must physically damage the actual real A, not only overlap its AABB");
    let before = case.lifecycle();
    case.publish(prepared.damage.update.clone()).unwrap();
    let after = case.lifecycle();
    assert!(after.epoch > before.epoch,
        "pending-only v2 must preserve ORIGINAL related whole-epoch cancellation");
    assert!(after.submitted.is_none() && after.ticket_ids.is_empty());
    let finished = case.finish().unwrap();
    assert_eq!(finished.lifecycle_before_cleanup.plans, 0);
    assert!(finished.cancelled_before_owner_return && !finished.completion_consumed);
    // Separate actual same-geometry latest-source recovery prerequisite. Obtain
    // new genuine Empty observations; do not change the held damage timestamp.
    let latest = r11_build(&mut prepared.cache, &mut prepared.fixture).unwrap();
    assert!(matches!(latest.update.changes.as_ref(), SurfaceGeometryChanges::Empty));
    assert!(nuwax_nav_pct::same_source_geometry(&prepared.damage.update.snapshot, &latest.update.snapshot));
    let recovery = HeldKernelCase::capture(setup.follower, latest.update.snapshot.clone(),
        scope_pose(12.55), scope_route(), scope_pose(30.55)).unwrap();
    scope_require_full_current_window(&recovery);
    let recovery_audit = recovery.audit_on(latest.update.snapshot).unwrap();
    scope_require_current_safety(&recovery_audit);
    assert!(recovery_audit.latest_edges_valid);
    let recovered = recovery.finish().unwrap();
    assert_eq!(recovered.lifecycle_before_cleanup.plans, 1);
    assert!(matches!(recovered.command_after_admission, Some(nuwax_nav_runtime::PlannerStep::Command(c)) if c != Twist2::ZERO));
    eprintln!("PENDING_ONLY_V2_COMBINED_PRECHECK fixture=R11_M1_M3_COMBINED_DAMAGE same_batch_writer=true original_points_unchanged=true actual_A_full=true actual_A_hit={:?} current_body_connector_stop_allowed=true latest_real_recovery_admitted=true all_precheck_times_excluded=true hardware_delivery_not_claimed=true", audit.first_new_edge_rejection);
}


// New retained-local-only cancellation witness, original fixed Q1 geometry.
// It needs one held background request; the separate F tests cover two-slot
// whole-epoch cancellation. No additional pose beyond original 12.55/13.55.
#[test]
#[ignore="explicit retained-detour relevance after real guard rejection; original map cancellation must remain"]
fn pending_only_v2_global_disjoint_retained_local_hit_cancels_background() {
    let setup=configured(true,true);
    assert_eq!(setup.follower.max_path_deviation_m.to_bits(),0.9_f64.to_bits());
    let same_path_bits=|left:&Path3D,right:&Path3D| {
        let bits=|p:&PoseStamped| [p.pose.position.x,p.pose.position.y,p.pose.position.z,
            p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w].map(f64::to_bits);
        left.poses.len()==right.poses.len() && left.poses.iter().zip(&right.poses).all(|(a,b)|
            a.frame_id==b.frame_id && a.stamp==b.stamp && bits(a)==bits(b))
    };
    let mut fixture=scope_floor();
    let mut cache=SurfaceGeometryCache::configured(&setup);
    let _floor=scope_build(&mut cache,&mut fixture,[251,31,40]);
    fixture.cells.extend((280..=283).flat_map(|x|(0..=42).flat_map(move |y|(44..=54).map(move |z|[x,y,z]))));
    let s0=scope_build(&mut cache,&mut fixture,[280,0,44]).snapshot;
    let mut case=Q1Case::start(setup.follower.clone(),s0.clone(),scope_pose(12.55),scope_route(),scope_pose(30.55)).unwrap();
    let first_entry=case.entered().unwrap();
    let first_id=q1_match_real_entry(&first_entry,case.submissions());
    let computed=case.computed().unwrap();
    assert_eq!(q1_match_real_entry(&computed.entry,case.submissions()),first_id);
    let admitted_path=computed.actual.unwrap();
    q1_require_full("LOCAL_CANCEL_INITIAL",&admitted_path,&first_entry,12.0,12.55);
    assert_eq!(admitted_path.path.poses.len(),448,"original Q1 geometry must not drift");
    let initial_audit=case.audit_current_candidate(&admitted_path,s0.clone()).unwrap();
    scope_require_current_safety(&initial_audit);assert!(initial_audit.latest_edges_valid);
    case.release_actual(first_entry.compute_ordinal).unwrap();
    let _first_step=case.pump_until_consumed_with_step(first_id).unwrap();
    let admitted=case.state();
    assert_eq!(admitted.plans,1);
    assert_eq!(admitted.admission,Some(admitted.generation));
    assert!(admitted.tickets.is_empty());

    // Original legal 1m odometry input and original deadline create a real
    // background job while the first full detour remains the retained path.
    case.update_pose_with_body_check(scope_pose(13.55)).unwrap();
    let rejected_step=case.tick_after_original_retry().unwrap();
    assert!(matches!(rejected_step,nuwax_nav_runtime::PlannerStep::Pending(_)),
        "fixed yaw-0 pose jump must revoke permission through original current_rejection");
    let background=case.entered().unwrap();
    let background_id=q1_match_real_entry(&background,case.submissions());
    assert_ne!(background_id,first_id);
    let _background_computed=case.computed().unwrap();
    let before=case.state();
    assert_eq!(before.plans,1);
    // The first path was truly admitted above. Fixed 13.55/yaw0 is one metre
    // from its still-unconsumed 12.55 rotation segment: original 0.9m deviation
    // guard revokes execution permission but deliberately retains that path.
    assert_eq!(before.admission,None);
    assert!(before.dirty);
    assert!(!background.detour,"ordinary refresh was accepted before current guard rejection");
    let (retained,reason)=case.retained_local_guard_state();
    assert!(retained.is_some_and(|path|same_path_bits(path,&admitted_path.path)),"original full path must retain every pose bit, frame and stamp");
    assert_eq!(reason,Some("当前轨迹受阻，保持零速等待局部接续：机体偏离连续三维路线"));
    assert_eq!(before.tickets.iter().map(|ticket|ticket.id).collect::<Vec<_>>(),vec![background_id]);
    assert_eq!(before.submitted.unwrap().0,background_id);

    fixture.cells.insert(SCOPE_DETOUR_HIT);
    let s2=scope_runtime(scope_build(&mut cache,&mut fixture,SCOPE_DETOUR_HIT));
    scope_require_columns(&s2,SCOPE_DETOUR_HIT);
    let mut proof=RuntimeTicket::new(Some(s0.clone()));
    proof.observe(&s2.snapshot,s2.change_basis.as_deref(),&s2.changes);
    assert!(!proof.unknown && proof.permits_path(&s2.snapshot,&scope_route()),
        "global must be disjoint; cancellation must exercise retained local_path");
    assert!(!proof.permits_path(&s2.snapshot,case.retained_local_guard_state().0.unwrap()),
        "S2 must hit the actual retained local_path even after admission was revoked");
    let physical=case.audit_edges(&admitted_path,&s2.snapshot);
    assert!(physical.old_all_valid && physical.latest_first_rejection.is_some());
    let published_before=Instant::now();
    case.publish(s2).unwrap();
    let published_after=Instant::now();
    let after=case.state();
    assert!(after.epoch>before.epoch,"retained local hit must keep original whole-epoch cancellation");
    assert!(after.submitted.is_none() && after.tickets.is_empty());
    assert!(after.retry_after>=published_before && after.retry_after<=published_after);
    assert_eq!((after.calls,after.plans,after.generation),(before.calls,1,before.generation));
    // Original map cancellation does not re-admit the guard-rejected path.
    assert_eq!(after.admission,before.admission);
    assert_eq!(after.admission,None);
    let (retained,reason)=case.retained_local_guard_state();
    assert!(retained.is_some_and(|path|same_path_bits(path,&admitted_path.path)));
    assert_eq!(reason,Some("当前轨迹受阻，保持零速等待局部接续：机体偏离连续三维路线"));
    assert!(case.current_body_ok());
    case.release_cancelled_actual(background.compute_ordinal).unwrap();
    let final_state=case.finish(); // original Stop, held-job release, owner join
    assert_eq!(final_state.plans,1,"cancelled background must not re-admit");
    eprintln!("PENDING_ONLY_V2_LOCAL_CANCEL global_disjoint=true initial_real_full_admitted=true original_guard_deviation_rejected=true retained_real_path_hit=true background_epoch_cancelled=true source_tickets_cleared=true original_retry_now=true no_performance_claim=true");
}
