// Explicit qualification tests. Included inside the existing surface_selective_refresh_tests
// module, after FineFixture/configured are available. Requires the existing
// non-default surface-recovery-qualification feature and the companion driver.
// Direct RouteFollower qualification only: not a bridge/scheduler timing test.
use nuwax_nav_runtime::route_change_scope_qualification::{HeldKernelCase, GeometryAudit};
use nuwax_nav_runtime::surface_changes::{SurfaceJobTicket as RuntimeTicket,
    SurfaceMapUpdate as RuntimeUpdate, ReferenceCorridor as RuntimeCorridor};

const SCOPE_PREFIX: [i32;3] = [31,31,51];
const SCOPE_FUTURE: [i32;3] = [591,31,51];
const SCOPE_DETOUR_HIT: [i32;3] = [283,47,51];

fn scope_pose(x:f64) -> PoseStamped {
    use nuwax_nav_types_messages::{NavFrameId,Point3,Pose3,Quaternion,Timestamp};
    let ns=current_unix_timestamp_ns().unwrap();
    PoseStamped {frame_id:NavFrameId::Map,stamp:Timestamp::from_unix(ns/1_000_000_000,(ns%1_000_000_000)as u32),
        pose:Pose3{position:Point3::new(x,1.55,2.5),orientation:Quaternion::IDENTITY}}
}
fn scope_route() -> Path3D {Path3D {poses:vec![scope_pose(0.55),scope_pose(30.55)]}}
fn scope_floor() -> FineFixture {
    let mut fixture=FineFixture::new();
    fixture.cells=(0..640).flat_map(|x|(0..60).map(move |y|[x,y,40])).collect();
    fixture
}
fn scope_build(cache: &mut SurfaceGeometryCache, fixture: &mut FineFixture,
    probe: [i32; 3]) -> crate::surface_map::SurfaceMapUpdate {
    let message = fixture.next();
    let stamp = message.source_timestamp_ns;
    assert!(stamp > 0);
    assert_eq!(message.committed_map.source_timestamp_ns, stamp);
    assert_eq!(i64::from(message.cloud.header.stamp_sec) * 1_000_000_000
        + i64::from(message.cloud.header.stamp_nsec), stamp);
    let root = nuwax_map_chunks::Manifest::decode(&message.fine.as_ref().unwrap().root).unwrap();
    assert_eq!(root.source_timestamp_ns, stamp as u64);
    assert_eq!(root.revision, message.cloud_revision as u64);
    assert_eq!(root.domain, schema::navigation_map_3d_domain(&message));
    assert_eq!(root.voxel_size_m.to_bits(), message.source_voxel_size.to_bits());
    let update = cache.build_with_changes(&message).unwrap();
    let observed = update.snapshot.observations().unwrap().get(probe).unwrap().observed_ns;
    assert_eq!(observed, stamp as u64);
    eprintln!("SCOPE_SOURCE revision={} source_ns={} root_ns={} observed_ns={} probe={:?} fine_size={}",
        message.cloud_revision, stamp, root.source_timestamp_ns, observed, probe, root.voxel_size_m);
    update
}
fn scope_runtime(update:crate::surface_map::SurfaceMapUpdate) -> RuntimeUpdate {
    RuntimeUpdate {snapshot:update.snapshot,change_basis:update.change_basis,changes:update.changes,planar_reference:None}
}
fn scope_proof(case:&HeldKernelCase,update:&RuntimeUpdate)->RuntimeTicket {
    let mut proof=RuntimeTicket::new(Some(case.actual().source.clone()));
    proof.observe(&update.snapshot,update.change_basis.as_deref(),&update.changes);
    assert!(!proof.unknown,"actual Writer publication must retain the complete source chain");
    proof
}
fn scope_require_columns(update:&RuntimeUpdate,cell:[i32;3]) {
    assert!(update.snapshot.observations().unwrap().get(cell).is_some(),"fixed fine cell not installed");
    let SurfaceGeometryChanges::Columns{fine_size,fine,planner_size,coarse}=update.changes.as_ref()
        else {panic!("fixture must produce actual complete Columns, not an injected change kind")};
    assert_eq!(fine_size.to_bits(),update.snapshot.source_voxel_size_m().to_bits());
    assert_eq!(planner_size.to_bits(),update.snapshot.config().voxel_size.to_bits());
    assert_eq!(fine,&vec![[cell[0],cell[1]]],"metadata must not invent or omit spatial changes");
    // Preserve the actual Reader conversion, including its f32 rounding.
    let xyz=cell.map(|v| ((f64::from(v)+0.5)*f64::from(*fine_size)) as f32);
    let quantized=xyz.map(|v| (v / *planner_size).floor() as i32);
    assert_eq!(coarse, &vec![[quantized[0],quantized[1]]],
        "actual coarse changes must match original center-to-planner quantization");
    eprintln!("SCOPE_CELL_WORLD center_f32={xyz:?} coarse_xyz={quantized:?}");
    // Use every actual fine/coarse column downstream. Never copy fine indices
    // into the coarse proof or infer completeness from these two resolutions.
    eprintln!("SCOPE_WRITER fixed_cell={cell:?} fine_size={fine_size} fine={fine:?} planner_size={planner_size} coarse={coarse:?}");
}
fn scope_require_full_current_window(case:&HeldKernelCase) {
    let actual=case.actual();
    let poses: Vec<_> = actual.path.poses.iter().map(|pose| {
        let p=pose.pose.position; let q=pose.pose.orientation;
        [p.x,p.y,p.z,q.x,q.y,q.z,q.w]
    }).collect();
    eprintln!("SCOPE_CANDIDATE partial={} pose_stamp_ns={} poses_xyz_xyzw={poses:?}",
        actual.partial, actual.path.poses.first().map_or(0,|p|p.stamp.nanos));
    assert!(!actual.partial,"M1/M2/M3 full-candidate prerequisite failed; do not synthesize a full path");
    assert!((actual.progress-12.0).abs()<1e-6,"current route progress was not established: {}",actual.progress);
    assert!(actual.end>actual.progress && actual.end<30.0);
    assert!(actual.horizon.is_finite() && actual.horizon>0.0);
    assert_eq!(actual.generation,case.lifecycle().generation);
    assert_eq!(actual.epoch,case.lifecycle().epoch);
    assert!(actual.path.poses.len()>1);
    let first=actual.path.poses.first().unwrap().pose.position;
    assert!((first.x-12.55).abs()<1e-6 && (first.y-1.55).abs()<1e-6);
    eprintln!("SCOPE_ACTUAL progress={} end={} effective_horizon={} poses={} kernel_ms={} (barrier excluded, not benchmark)",
        actual.progress,actual.end,actual.horizon,actual.path.poses.len(),actual.kernel_elapsed_ms);
}
fn scope_require_current_safety(audit:&GeometryAudit) {
    assert!(audit.old_edges_valid,"real candidate no longer passes its captured source");
    assert!(audit.latest_body_valid,"changed fixture blocks the actual current body");
    assert!(audit.latest_connector_valid,"changed fixture invalidates current path connector");
    assert!(audit.current_command.is_some_and(|c|c!=Twist2::ZERO),"fixture lacks actual nonzero currently checked command");
    assert!(audit.complete_stop_valid,"independent full current stopping trajectory failed");
}

// Baseline behavior witness: expected PASS before any production change.
// A later keep-job candidate must intentionally change the cancellation assertions;
// this file by itself does not implement or authorize that policy.
fn scope_prefix_or_future_baseline(cell:[i32;3]) {
    // Independent control arm: real kernel result really can be consumed.
    let setup=configured(true,true);
    let mut fixture=scope_floor();
    let mut cache=SurfaceGeometryCache::configured(&setup);
    let initial=scope_build(&mut cache, &mut fixture, [251,31,40]);
    let control=HeldKernelCase::capture(setup.follower.clone(),initial.snapshot,
        scope_pose(12.55),scope_route(),scope_pose(30.55)).unwrap();
    scope_require_full_current_window(&control);
    let audit=control.audit_on(control.actual().source.clone()).unwrap();
    scope_require_current_safety(&audit);assert!(audit.latest_edges_valid);
    let control_done=control.finish().unwrap(); // cleanup/join before core assertions
    assert!(!control_done.cancelled_before_owner_return && control_done.completion_consumed);
    assert_eq!(control_done.lifecycle_before_cleanup.plans,1);
    assert!(matches!(control_done.command_after_admission,Some(nuwax_nav_runtime::PlannerStep::Command(c)) if c!=Twist2::ZERO));

    // Change arm: a fresh real Writer and kernel, same fixed geometry and route.
    let setup=configured(true,true);
    let mut fixture=scope_floor();
    let mut cache=SurfaceGeometryCache::configured(&setup);
    let initial=scope_build(&mut cache, &mut fixture, [251,31,40]);
    let mut case=HeldKernelCase::capture(setup.follower.clone(),initial.snapshot,
        scope_pose(12.55),scope_route(),scope_pose(30.55)).unwrap();
    scope_require_full_current_window(&case);
    let before=case.lifecycle();
    assert!(before.admission.is_none() && before.plans==0,"this first-candidate witness must not masquerade as admitted background refresh");
    fixture.cells.insert(cell);
    let update=scope_runtime(scope_build(&mut cache, &mut fixture, cell));
    scope_require_columns(&update,cell);
    let proof=scope_proof(&case,&update);
    assert!(!proof.permits_path(&update.snapshot,case.global_route()),"fixed point is not global-route related");
    assert!(proof.permits_path(&update.snapshot,&case.actual().path),"fixed point affects the real candidate; this MRE prerequisite failed");
    let SurfaceGeometryChanges::Columns{fine_size,fine,planner_size,coarse}=update.changes.as_ref() else {unreachable!()};
    let actual_corridor=RuntimeCorridor::new(&update.snapshot,&case.actual().path).unwrap();
    assert!(!actual_corridor.intersects_columns(*fine_size,fine));
    assert!(!actual_corridor.intersects_columns(*planner_size,coarse));
    let audit=case.audit_on(update.snapshot.clone()).unwrap();
    scope_require_current_safety(&audit);assert!(audit.latest_edges_valid);
    let publish_started=Instant::now();
    case.publish(update).unwrap();
    let publish_finished=Instant::now();
    let after=case.lifecycle();let retained=case.retained_goal_and_route();
    let changed_done=case.finish().unwrap(); // original result goes through original owner, then joins
    assert!(after.epoch>before.epoch,"baseline no longer cancels global-related work; reassess witness");
    assert!(after.submitted.is_none() && after.ticket_ids.is_empty());
    assert_eq!((after.calls,after.plans,after.generation,after.progress),(before.calls,before.plans,before.generation,before.progress));
    assert!(after.retry_after>=publish_started && after.retry_after<=publish_finished);
    assert!(retained);
    assert!(changed_done.cancelled_before_owner_return && !changed_done.completion_consumed);
    assert_eq!(changed_done.lifecycle_before_cleanup.plans,0);
    assert!(changed_done.command_after_admission.is_none());
    eprintln!("SCOPE_BASELINE fixed_cell={cell:?} control_real_admission=1 changed_real_admission=0 candidate_proof_allowed=true epoch_cancelled=true cleanup_returned=true no_speedup_claim=true");
}
#[test]
#[ignore="explicit tests-only source-scope qualification; not a timing benchmark"]
fn scope_m1_real_completed_candidate_lost_on_historical_column() {scope_prefix_or_future_baseline(SCOPE_PREFIX);}
#[test]
#[ignore="explicit tests-only source-scope qualification; not a timing benchmark"]
fn scope_m2_real_completed_candidate_lost_on_distant_future_column() {scope_prefix_or_future_baseline(SCOPE_FUTURE);}

#[test]
#[ignore="fixed real detour prerequisite may fail; never move the hit point to make it pass"]
fn scope_m3_global_disjoint_real_detour_hit_is_rejected_at_completion() {
    let setup=configured(true,true);
    let mut fixture=scope_floor();
    let mut cache=SurfaceGeometryCache::configured(&setup);
    let _clear=scope_build(&mut cache, &mut fixture, [251,31,40]);
    // The job.source already includes this fixed wall. It is not the tested delta.
    fixture.cells.extend((280..=283).flat_map(|x|(0..=42).flat_map(move |y|(44..=54).map(move |z|[x,y,z]))));
    let walled=scope_build(&mut cache, &mut fixture, [280,0,44]);
    let mut case=HeldKernelCase::capture(setup.follower.clone(),walled.snapshot,
        scope_pose(12.55),scope_route(),scope_pose(30.55)).unwrap();
    scope_require_full_current_window(&case);
    let before=case.lifecycle();
    fixture.cells.insert(SCOPE_DETOUR_HIT);
    let update=scope_runtime(scope_build(&mut cache, &mut fixture, SCOPE_DETOUR_HIT));
    scope_require_columns(&update,SCOPE_DETOUR_HIT);
    let proof=scope_proof(&case,&update);
    assert!(proof.permits_path(&update.snapshot,case.global_route()),"fixed hit is not outside global proof scope");
    assert!(!proof.permits_path(&update.snapshot,&case.actual().path),"fixed hit misses real candidate scope");
    let audit=case.audit_on(update.snapshot.clone()).unwrap();
    scope_require_current_safety(&audit);
    assert!(!audit.latest_edges_valid && audit.first_new_edge_rejection.is_some(),
        "AABB overlap is insufficient: actual old-valid candidate motion must now be blocked");
    eprintln!("SCOPE_M3_PHYSICAL_REJECTION {:?}",audit.first_new_edge_rejection);
    case.publish(update).unwrap();
    let after=case.lifecycle();let retained=case.retained_goal_and_route();
    let done=case.finish().unwrap();
    assert_eq!(after.epoch,before.epoch,"global-disjoint case must reach candidate proof rather than early cancel");
    assert_eq!(after.submitted,before.submitted);
    assert_eq!(after.ticket_ids,before.ticket_ids);
    assert!(retained);
    assert!(!done.cancelled_before_owner_return && done.completion_consumed);
    assert_eq!(done.lifecycle_before_cleanup.plans,0);
    assert!(done.command_after_admission.is_none());
    assert!(done.lifecycle_before_cleanup.ticket_ids.is_empty());
    eprintln!("SCOPE_M3 global_disjoint=true real_candidate_physically_blocked=true epoch_kept=true completion_consumed=true admission=0 cleanup_returned=true");
}
