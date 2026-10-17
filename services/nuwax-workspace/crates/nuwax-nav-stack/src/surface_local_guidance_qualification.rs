// Reuses the existing 5 cm ordinary-body Box/Wall owner and motion chain unchanged.
use super::*;
use nuwax_nav_runtime::local_guidance_qualification::CaptureSession;

fn exact_wire_route(wire:&schema::PathEnvelope,route:&nuwax_nav_types_messages::Path3D)->bool {
    let Ok(nuwax_nav_runtime::ExecutionPath::Surface(decoded))=
        execution_path_from_schema(&wire.path,true) else {return false;};
    decoded.poses.len()==route.poses.len() && decoded.poses.iter().zip(&route.poses).all(|(a,b)| {
        let p=|v:&nuwax_nav_types_messages::PoseStamped| {
            let p=v.pose.position;let q=v.pose.orientation;
            [p.x,p.y,p.z,q.x,q.y,q.z,q.w].map(f64::to_bits)
        };
        a.frame_id==b.frame_id && p(a)==p(b)
    })
}
async fn qualify_existing_chain(scene:Scene) {
    let capture=CaptureSession::start([3.5,1.5,0.5]);
    let mut harness=OccupancyHarness::start(true,scene).await;
    let result=harness.exercise().await;
    let cleanup=harness.shutdown().await;
    harness.plant.lock().unwrap().sample();
    let audit=harness.audit_after_cleanup();
    harness.dump_evidence();
    // Never repair a failing physical/control fixture by changing qualification gates.
    assert!(result.is_ok() && cleanup.is_ok() && audit.is_ok(),
        "original fixture failed: result={result:?} cleanup={cleanup:?} audit={audit:?}");
    let (counts,captured)=capture.finish();
    eprintln!("LOCAL_GUIDANCE_COUNTS scene={scene:?} {counts:?}");
    let Some(c)=captured else {
        let reason=if counts.target_calls==0 {"no_matching_target_plan_observed"}
            else if counts.global_curved==0 {"no_actual_global_curved_nominal_window"}
            else if counts.remaining_search==0 {"no_curved_nominal_remaining_search"}
            else {"cancelled_root_only_or_nondefault_input"};
        eprintln!("LOCAL_GUIDANCE_STATUS scene={scene:?} status=NOT_QUALIFIED reason={reason} no_replay=true fixture_unchanged=true");
        return;
    };
    let goal=harness.goal.as_ref().unwrap();
    let evidence=harness.evidence.lock().unwrap();
    let began=c.started;
    let Some(wire)=evidence.paths.iter().find(|p|exact_wire_route(p,&c.global_route)
        && validate_path_goal_correlation(p,goal).is_ok()
        && evidence.path_receipts.iter().any(|(at,revision,_)|*at<=began && *revision==p.path_revision)) else {
        eprintln!("LOCAL_GUIDANCE_STATUS scene={scene:?} status=NOT_QUALIFIED reason=no_preceding_correlated_wire_payload_receipt no_replay=true");
        return;
    };
    let Some(source)=evidence.maps.values().find(|row|row.published<=began && evidence.built.get(&row.source)
        .and_then(|built|built.snapshot.as_ref().ok()).is_some_and(|map|
            map.same_geometry(&c.source) && match(map.observations(),c.source.observations()) {
                (Some(a),Some(b))=>Arc::ptr_eq(a,b), (None,None)=>true, _=>false,
            })) else {
        eprintln!("LOCAL_GUIDANCE_STATUS scene={scene:?} status=NOT_QUALIFIED reason=no_exact_static_geometry_observation_owner_ledger no_replay=true");
        return;
    };
    // The ledger above matches the original request owner. planning_source is
    // the actual private Scan fork retained at compute_inner's callsite; its
    // cache/snapshot owner intentionally differs. No cross-owner identity claim.
    eprintln!("LOCAL_GUIDANCE_CAPTURE scene={scene:?} source={source:?} pose={:?} start={:?} generation={} began={:?} finished={:?} progress={} end={} reference={:?} global_path={wire:?} motion={:?} config={:?} final_yaw={:?} arrival={:?} boundary={:?} prefix={:?} minimum_progress={:?} static_source_owner_matched=true planning_view_origin=compute_inner_actual_fork immutable_inputs=retained_by_fork_contract snapshot_owner_equality=not_claimed recent_wire_timestamp_attribution=not_exposed",
        c.pose,c.start,c.generation,c.started,c.finished,c.reference_progress,c.reference_end,c.reference,c.motion,c.config,c.final_yaw,c.arrival,c.boundary,c.progress_reference,c.minimum_progress);
    drop(evidence);
    c.audit_output(&c.original).unwrap();
    eprintln!("LOCAL_GUIDANCE_ORIGINAL output={:?} observation={:?}",c.original,c.observation);
    // Owners are gone; no replay changes live scheduling or issues motion.
    let (baseline,baseline_observation)=c.replay(false);
    let (guided,guided_observation)=c.replay(true);
    eprintln!("LOCAL_GUIDANCE_REPLAY follow_reference=false output={baseline:?} observation={baseline_observation:?}");
    eprintln!("LOCAL_GUIDANCE_REPLAY follow_reference=true output={guided:?} observation={guided_observation:?}");
    // No assertion that closer reference following is better or that partial is forbidden.
    // A replay that unexpectedly avoids the target search must not score an h comparison.
    let both_target=[&baseline_observation,&guided_observation].iter().all(|o|
        o.phases.iter().any(|p|p.phase=="search_remaining") && o.work[0]>1);
    let baseline_same=c.original.partial==baseline.partial
        && c.original.poses.iter().map(|p|(p.0.map(f64::to_bits),p.1.map(f64::to_bits))).eq(
            baseline.poses.iter().map(|p|(p.0.map(f64::to_bits),p.1.map(f64::to_bits))))
        && c.original.error==baseline.error;
    eprintln!("LOCAL_GUIDANCE_STATUS scene={scene:?} status={} baseline_pose_bits_and_error_contract_reproduced={baseline_same} both_target={both_target} offline_only=true live_ab=false default_budget=256 prefer_straight=false",
        if baseline_same && both_target {"QUALIFIED_OFFLINE_AB"} else {"NOT_QUALIFIED"});
}
#[tokio::test(flavor="multi_thread",worker_threads=2)]
#[ignore="explicit qualification; no fixture tuning if NOT_QUALIFIED"]
async fn existing_box_nominal_guidance_qualification() {qualify_existing_chain(Scene::Box).await;}
#[tokio::test(flavor="multi_thread",worker_threads=2)]
#[ignore="explicit qualification; wall negative may correctly remain NOT_QUALIFIED"]
async fn existing_wall_nominal_guidance_qualification() {qualify_existing_chain(Scene::Wall).await;}
