// Real owner wake qualification. All computations call the original PCT and all
// actual contexts come from the original worker/scheduler. This tests feedback
// recovery after an injected refusal, not the geometry that caused a refusal.
use super::*;

struct HeldHarness {
    // Drop releases blocking channels before Harness joins either owner.
    _cleanup: ReleaseOnDrop,
    harness: Harness,
    probe: Arc<Probe>,
    goal: schema::GoalEnvelope,
    published: schema::PathEnvelope,
    frozen: schema::NavigationMap3D,
    gate: usize,
    calls: usize,
}
impl HeldHarness {
    fn unpause(&self) {
        self.probe.free_running.store(true, Order::SeqCst);
        self.probe.release(self.gate);
    }
    fn last_input(&self) -> SurfaceReplanInput {
        self.probe.captures.lock().unwrap().last().unwrap().clone()
    }
    async fn input_after(&self, previous: usize,
        predicate: impl Fn(&SurfaceReplanInput) -> bool) -> SurfaceReplanInput {
        self.probe.wait(|_| self.probe.captures.lock().unwrap()
            .iter().skip(previous).any(&predicate)).await;
        self.probe.captures.lock().unwrap().iter().skip(previous)
            .find(|input| predicate(input)).unwrap().clone()
    }
    async fn real_result_after(&mut self, old_revision: i64) {
        self.probe.wait(|events| events.iter().any(|event|
            matches!(event, Event::Computed{call, ok:true,..} if *call>self.calls))).await;
        let path = self.harness.path_for(&self.goal).await;
        assert!(path.path_revision > old_revision);
        assert_eq!(path.goal_key(), self.goal.goal_key());
        assert!(self.probe.normal.load(Order::SeqCst) > self.calls);
        self.published = path;
    }
    async fn finish(mut self, label: &str) {
        // A replacement publication can leave the real outer path handler
        // waiting for scheduler acceptance. Release only test synchronization
        // before sending Cancel through that same original LCM handler.
        self.probe.free_running.store(true,Order::SeqCst);
        self.probe.release_all();
        self.harness.cancel(&self.goal).await;
        status_when(&self.harness, |s| s.last_terminal == Some(TerminalOutcome::Cancelled)).await;
        self.probe.release_all();
        self.harness.shutdown().await;
        eprintln!("HELD_WAKE case={label} real_normal_calls={} real_quick_calls={} cancel=Cancelled cleanup_returned=true",
            self.probe.normal.load(Order::SeqCst), self.probe.quick.load(Order::SeqCst));
    }
}

async fn start_held(initial: [f64;3], target: [f64;3]) -> HeldHarness {
    let probe = Probe::with_position(Some(initial));
    let observed = probe.clone();
    let normal: SurfacePathComputer = Arc::new(move |surface, pose, goal, configured| {
        *observed.planner_pct.lock().unwrap() = Some(configured.follower.pct.clone());
        if observed.normal.load(Order::SeqCst)>0 && !observed.free_running.load(Order::SeqCst) {
            observed.permits.lock().unwrap().recv_timeout(Duration::from_secs(5))
                .expect("original refusal gate must precede initial replacement");
        }
        let call=observed.normal.fetch_add(1,Order::SeqCst)+1;
        let result=body_path(surface,pose,goal,configured);
        observed.record(Event::Computed{call,ok:result.is_ok(),gate:observed.gate.load(Order::SeqCst)});
        if observed.hold_next_normal_return.swap(false,Order::SeqCst) {
            let (sender,receiver)=std_mpsc::channel();
            observed.return_releases.lock().unwrap().insert(call,sender);
            observed.record(Event::NormalReturnHeld(call));
            receiver.recv_timeout(Duration::from_secs(5)).expect("release the real PCT result");
        }
        observed.record(Event::NormalReturned{call,cancelled:configured.planning_cancelled.as_ref().unwrap().load(Order::Relaxed)});
        result
    });
    let observed=probe.clone();
    let quick:SurfacePathComputer=Arc::new(move |surface,pose,goal,configured| {
        let call=observed.quick.fetch_add(1,Order::SeqCst)+1;
        let result=body_path_with_budget(surface,pose,goal,configured,0);
        observed.record(Event::QuickComputed{call,ok:result.is_ok()});
        result
    });
    let mut configured=config();
    configured.follower.local_planning_enabled=false;
    configured.apply_calibration(Some(nuwax_nav_types_messages::RobotGeometry {
        self_collision:None,
        footprint_xy_m:vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
        z_min_m:-0.05,z_max_m:0.2,source:"OFF soft-refresh owner fixture".into(),
    }),None).unwrap();
    let mut harness=Harness::start_with_components_and_feedback_probe(normal,
        Duration::from_millis(25),configured,true,Some(quick),None,
        nuwax_nav_runtime::SchedulerConfig::default(),Some(probe.clone())).await;
    let cleanup=ReleaseOnDrop(probe.clone());
    status_when(&harness,|s|s.planner_inputs_fresh).await;
    let mut maps=harness.hub.events.subscribe();
    let frozen={let mut feed=harness.feed.lock().unwrap();feed.map_enabled=false;
        feed.last_published.clone().unwrap()};
    tokio::time::timeout(Duration::from_secs(5),async {
        loop {
            if harness.hub.latest.lock().unwrap().as_ref().is_some_and(|map|
                map.authority.cloud_revision==frozen.cloud_revision&&map.snapshot.is_ok()) {break;}
            maps.recv().await.unwrap();
        }
    }).await.expect("original frozen source installation");
    status_when(&harness,|s|s.costmap_source_timestamp_ns==frozen.source_timestamp_ns).await;
    let goal=goal_for(&frozen,1,target);
    harness.goal(&goal).await;
    let mut published=harness.path_for(&goal).await;
    let mut gate=1;
    let mut calls=1;
    loop {
        probe.wait(|events|events.contains(&Event::Gate(gate))).await;
        let feedback=harness.scheduler.status().surface_replan_feedback.unwrap();
        assert!(feedback.zero_confirmed_rejection);
        assert_eq!(feedback.rejected_context,feedback.current_context);
        assert_eq!(feedback.current_context.as_ref().unwrap().applied,Twist2::ZERO);
        probe.wait(|events|events.contains(&Event::Reuse{complete:true})||
            events.contains(&Event::Computed{call:calls+1,ok:true,gate})).await;
        if probe.events.lock().unwrap().contains(&Event::Reuse{complete:true}) {break;}
        let next=harness.path_for(&goal).await;
        assert_eq!(next.path_revision,published.path_revision+1);
        assert_eq!(probe.gate.load(Order::SeqCst),gate);
        published=next;calls+=1;probe.release(gate);gate+=1;
    }
    probe.wait(|events|events.iter().filter(|event|**event==Event::Held).count()>=2).await;
    assert_eq!(probe.normal.load(Order::SeqCst),calls);
    assert_eq!(probe.quick.load(Order::SeqCst),0);
    let captures=probe.captures.lock().unwrap();
    assert_eq!(captures.len(),calls);
    assert!(captures.last().unwrap().actual.is_some());
    assert!(captures[..calls-1].iter().all(|input|input.actual.is_none()));
    drop(captures);
    HeldHarness{_cleanup:cleanup,harness,probe,goal,published,frozen,gate,calls}
}

async fn standard_held()->HeldHarness {start_held([0.55,1.55,2.5],[1.55,1.55,2.5]).await}
fn xyz(input:&SurfaceReplanInput)->[f64;3] {
    let p=input.actual.as_ref().unwrap().odometry.pose.position;[p.x,p.y,p.z]
}

#[tokio::test]
async fn held_real_pct_wakes_on_actual_pose() {
    let mut held=standard_held().await;
    let old=held.last_input();let n=held.calls;let revision=held.published.path_revision;
    held.harness.feed.lock().unwrap().position=[0.56,1.55,2.5];
    held.unpause();
    let changed=held.input_after(n,|input|input.actual.is_some()&&xyz(input)==[0.56,1.55,2.5]).await;
    assert!(!old.same_complete_input(&changed));
    assert!(old.source.same_body_query_view(&changed.source));
    // Known ground state carries the actual XY reference, so this real pose
    // event also updates reference_origin.x. The fixed flat source keeps h and
    // the actual query classification, not the entire association, unchanged.
    assert_eq!(old.actual.as_ref().unwrap().ground.height().to_bits(),changed.actual.as_ref().unwrap().ground.height().to_bits());
    assert_eq!(old.actual.as_ref().unwrap().ground.query_state(),changed.actual.as_ref().unwrap().ground.query_state());
    held.real_result_after(revision).await;
    held.finish("actual-pose").await;
}

#[tokio::test]
async fn held_real_pct_wakes_on_planner_costs() {
    let mut held=standard_held().await;
    let old=held.last_input();let n=held.calls;let revision=held.published.path_revision;
    let pct=held.probe.planner_pct.lock().unwrap().as_ref().unwrap().clone();
    pct.report_local_blockage([3.55,0.55,2.1],0.1);
    // The owner itself reads its original cost snapshot on its original tick.
    let changed=held.input_after(n,|input|!same_penalties(&old.penalties,&input.penalties)).await;
    assert_eq!(old.actual,changed.actual);
    assert!(old.source.same_body_query_view(&changed.source));
    assert!(!old.same_complete_input(&changed));
    held.real_result_after(revision).await;
    held.finish("planner-costs").await;
}

fn changed_terrain(original:&schema::NavigationMap3D)->schema::NavigationMap3D {
    let mut map=original.clone();
    for value in [3.55f32,0.55,2.35] {map.cloud.data.extend_from_slice(&value.to_le_bytes());}
    map.cloud.width+=1;map.cloud.row_step+=12;
    let now=current_unix_timestamp_ns().unwrap();
    map.cloud_revision+=1;map.cloud_occupancy_revision+=1;map.commit_sequence+=1;
    map.source_timestamp_ns=now;map.committed_map.revision+=1;
    map.committed_map.commit_sequence=map.commit_sequence;
    map.committed_map.source_timestamp_ns=now;
    map.cloud.header.seq+=1;map.cloud.header.stamp_sec=(now/1_000_000_000) as i32;
    map.cloud.header.stamp_nsec=(now%1_000_000_000) as i32;
    map
}
#[tokio::test]
async fn held_real_pct_wakes_on_terrain_publication() {
    let mut held=standard_held().await;
    let old=held.last_input();let n=held.calls;let revision=held.published.path_revision;
    let changed=changed_terrain(&held.frozen);
    held.harness.map(&changed).await;
    held.unpause();
    let input=held.input_after(n,|input|!input.source.same_body_query_view(&old.source)).await;
    assert!(!old.same_complete_input(&input));
    held.real_result_after(revision).await;
    held.finish("terrain-publication").await;
}

async fn publish_recent(held:&HeldHarness) {
    use nuwax_codec::lcm::{lio,recent_cloud};
    let map=&held.frozen;let now=current_unix_timestamp_ns().unwrap();
    let mut cloud=map.cloud.clone();cloud.data.clear();
    for value in [3.55f32,0.55,2.35] {cloud.data.extend_from_slice(&value.to_le_bytes());}
    cloud.width=1;cloud.row_step=12;cloud.header.seq=1;
    cloud.header.stamp_sec=(now/1_000_000_000)as i32;cloud.header.stamp_nsec=(now%1_000_000_000)as i32;
    let packet=recent_cloud::RecentCloud3D {
        producer_runtime_generation:map.producer_runtime_generation,
        session_id:map.committed_map.session_id.clone(),session_generation:map.committed_map.session_generation,
        map_frame_generation:map.map_frame_generation,commit_boot_nonce:map.commit_boot_nonce,
        ground_point_indices:vec![],cloud:Some(lio::LioCloudEnvelope{
            schema_version:1,backend:lio::LIO_BACKEND_DEVICE,runtime_generation:2,source_sequence:1,
            source_timestamp_ns:now,correlated_odometry_source_sequence:1,correlated_odometry_source_timestamp_ns:now,
            sensor_frame_id:"sensor".into(),body_frame_id:"body".into(),odom_frame_id:"odom".into(),map_frame_id:"map".into(),
            registration_state:lio::LIO_REGISTRATION_REGISTERED_MAP,source_point_count:1,emitted_point_count:1,cloud,
        }),
    };
    publish_body(&held.harness.transport,recent_cloud::DEFAULT_TOPIC,recent_cloud::TYPE_NAME,
        recent_cloud::encode_body(&packet).unwrap()).await.unwrap();
}
#[tokio::test]
async fn held_real_pct_wakes_on_actual_recent_observation() {
    let mut held=standard_held().await;
    let old=held.last_input();let n=held.calls;let revision=held.published.path_revision;
    publish_recent(&held).await;held.unpause();
    let input=held.input_after(n,|input|input.actual.as_ref().is_some_and(|actual|
        !actual.source.same_body_query_view(&old.actual.as_ref().unwrap().source))).await;
    assert!(input.source.same_body_query_view(&old.source),"planner terrain is unchanged");
    assert!(nuwax_nav_pct::same_source_geometry(&input.actual.as_ref().unwrap().source,
        &old.actual.as_ref().unwrap().source),"recent-only observation preserves terrain geometry");
    assert!(!old.same_complete_input(&input));
    held.real_result_after(revision).await;
    held.finish("recent-observation").await;
}

#[tokio::test]
async fn held_real_pct_tracks_actual_carried_ground_history() {
    // A is unknown from the very first input, never preseeded by default known pose.
    let a=[4.55,1.55,2.6];let b=[3.55,1.55,2.6];
    let mut held=start_held(a,[5.55,1.55,2.6]).await;
    let initial=held.last_input();let n=held.calls;let revision=held.published.path_revision;
    assert_eq!(initial.actual.as_ref().unwrap().ground.query_state(),nuwax_nav_mls_planner::surface::GroundQueryState::Unknown);
    held.harness.feed.lock().unwrap().position=b;held.unpause();
    status_when(&held.harness,|s|s.surface_replan_feedback.as_ref().and_then(|f|f.current_context.as_ref())
        .is_some_and(|actual| {let p=actual.odometry.pose.position;
            [p.x,p.y,p.z]==b&&actual.ground!=initial.actual.as_ref().unwrap().ground})).await;
    let known=held.harness.scheduler.status().surface_replan_feedback.unwrap().current_context.unwrap();
    held.harness.feed.lock().unwrap().position=a;
    let input=held.input_after(n,|input|input.actual.as_ref().is_some_and(|actual|
        xyz(input)==a&&actual.ground!=initial.actual.as_ref().unwrap().ground)).await;
    assert_ne!(known.ground.query_state(),nuwax_nav_mls_planner::surface::GroundQueryState::Unknown);
    assert_eq!(input.actual.as_ref().unwrap().ground.query_state(),nuwax_nav_mls_planner::surface::GroundQueryState::Unknown);
    assert_eq!(input.actual.as_ref().unwrap().ground.height().to_bits(),known.ground.height().to_bits());
    assert_ne!(input.actual.as_ref().unwrap().ground.height().to_bits(),initial.actual.as_ref().unwrap().ground.height().to_bits());
    assert!(same_pose(&initial.actual.as_ref().unwrap().odometry,&input.actual.as_ref().unwrap().odometry));
    assert_eq!(initial.actual.as_ref().unwrap().applied,input.actual.as_ref().unwrap().applied);
    assert!(!initial.same_complete_input(&input));
    held.real_result_after(revision).await;
    eprintln!("GROUND_HISTORY initial={:?} known={:?} final={:?} attribution=pose-history-plus-carried-ground",
        initial.actual.as_ref().unwrap().ground,known.ground,input.actual.as_ref().unwrap().ground);
    held.finish("carried-ground-history").await;
}

#[tokio::test]
async fn held_real_pct_wakes_after_real_nonzero_applied_and_new_path() {
    let mut held=standard_held().await;
    let baseline=held.last_input();let n=held.calls;
    let pct=held.probe.planner_pct.lock().unwrap().as_ref().unwrap().clone();
    held.probe.allow_commands.store(1,Order::SeqCst);
    // Wake the original owner once so it issues its own next real path revision.
    // The following APPLIED/refusal cycle has these same already-captured costs.
    pct.report_local_blockage([3.55,0.55,2.1],0.1);
    let first=held.input_after(n,|input|!same_penalties(&baseline.penalties,&input.penalties)).await;
    assert_eq!(first.actual.as_ref().unwrap().applied,Twist2::ZERO);
    held.probe.wait(|events|events.contains(&Event::Computed{call:n+1,ok:true,gate:held.gate})).await;
    let candidate=interleaving_path(&mut held.harness,&held.goal,held.published.path_revision+1).await;
    held.probe.release(held.gate);
    held.probe.wait(|events|events.contains(&Event::Applied)).await;
    let next_gate=held.gate+1;
    held.probe.wait(|events|events.contains(&Event::Gate(next_gate))).await;
    let observed=held.probe.applied.lock().unwrap().clone();
    assert_eq!(observed.len(),1);assert_ne!(observed[0],Twist2::ZERO);
    let feedback=held.harness.scheduler.status().surface_replan_feedback.unwrap();
    assert_eq!(feedback.path_identity.path_revision,candidate.path_revision as u64);
    assert!(feedback.zero_confirmed_rejection);
    let rejected=feedback.rejected_context.as_ref().unwrap();
    assert_eq!(rejected.applied,observed[0]);
    let before=first.actual.as_ref().unwrap();
    assert!(same_pose(&rejected.odometry,&before.odometry));
    assert_eq!(rejected.ground,before.ground);
    assert!(rejected.source.same_body_query_view(&before.source));
    // The new real ZERO invalidates the nonzero worker version. A lifecycle
    // publication may not yet exist at the immediate post-receipt read. None
    // is deliberately inconclusive; never synthesize an actual ZERO context.
    if let Some(current)=&feedback.current_context {
        assert_eq!(current.applied,Twist2::ZERO);
        assert!(same_pose(&rejected.odometry,&current.odometry));
        assert_eq!(rejected.ground,current.ground);
        assert!(rejected.source.same_body_query_view(&current.source));
    }
    let second=held.input_after(n+1,|_|true).await;
    assert!(same_penalties(&first.penalties,&second.penalties));
    assert!(first.source.same_body_query_view(&second.source));
    assert!(same_pose(&first.start,&second.start)&&same_pose(&first.goal,&second.goal));
    // State and feedback watches can race. Any available context must still
    // be one of this test's two actual receipts at the identical physical input.
    if let Some(actual)=&second.actual {
        assert!(actual.applied==Twist2::ZERO||actual.applied==observed[0]);
        assert!(same_pose(&actual.odometry,&before.odometry));
        assert_eq!(actual.ground,before.ground);
        assert!(actual.source.same_body_query_view(&before.source));
    }
    eprintln!("APPLIED_REFUSAL actual={:?} rejected={:?} immediate_current={:?} recaptured_actual={:?}",
        observed[0],rejected,feedback.current_context,second.actual);
    held.probe.wait(|events|events.contains(&Event::Computed{call:n+2,ok:true,gate:next_gate})).await;
    let replacement=interleaving_path(&mut held.harness,&held.goal,candidate.path_revision+1).await;
    assert_eq!(held.probe.normal.load(Order::SeqCst),n+2);
    assert_eq!(held.probe.quick.load(Order::SeqCst),0);
    let reuses=interleaving_reuses(&held.probe);
    held.published=replacement;
    held.calls=n+2;
    held.gate=next_gate+1;
    held.probe.release(next_gate);
    interleaving_converge(&mut held,reuses).await;
    let restored=held.harness.scheduler.status().surface_replan_feedback.unwrap();
    assert_eq!(restored.current_context.as_ref().unwrap().applied,Twist2::ZERO);
    assert_eq!(restored.rejected_context,restored.current_context);
    eprintln!("APPLIED_WAKE actual={:?} real_candidate_revision={} final_revision={} rejected_applied_changed=true final_complete_held=true",
        observed[0],candidate.path_revision,held.published.path_revision);
    held.finish("real-applied-after-real-owner-path").await;
}

fn interleaving_reuses(probe: &Probe) -> usize {
    probe.events.lock().unwrap().iter()
        .filter(|event| **event == Event::Reuse { complete: true }).count()
}

// Unlike Harness::path_for, this cannot silently skip obsolete-goal publications.
async fn interleaving_path(h: &mut Harness, goal: &schema::GoalEnvelope, revision: i64)
    -> schema::PathEnvelope {
    let frame = tokio::time::timeout(Duration::from_secs(5), h.paths.next())
        .await.expect("original path publication").unwrap().unwrap();
    let path = schema::decode_path_envelope_body(&frame.payload).unwrap();
    assert_eq!(path.goal_key(), goal.goal_key());
    assert_eq!(path.session_id, goal.session_id);
    assert_eq!(path.path_revision, revision);
    assert!(!path.path.poses.is_empty());
    path
}

// Start with HH.published already delivered and HH.gate = the next expected
// refusal gate. Keep the original source/goal fixture and continue stage-67
// convergence; no arbitrary sleep is used as evidence of a stable state.
async fn interleaving_converge(hh: &mut HeldHarness, old_reuses: usize) {
    loop {
        let gate = hh.gate;
        let next_call = hh.calls + 1;
        hh.probe.wait(|events| events.contains(&Event::Gate(gate))).await;
        let feedback = hh.harness.scheduler.status().surface_replan_feedback.unwrap();
        assert!(feedback.zero_confirmed_rejection);
        assert!(feedback.rejected_context.is_some());
        assert_eq!(feedback.current_context, feedback.rejected_context);
        assert_eq!(feedback.path_identity.path_revision, hh.published.path_revision as u64);
        assert!(feedback.path_identity.has_same_goal_identity(
            &guarded_goal_execution_identity(&hh.goal)));
        hh.probe.wait(|events|
            events.iter().filter(|e| **e == Event::Reuse { complete: true }).count() > old_reuses
            || events.contains(&Event::Computed { call: next_call, ok: true, gate })).await;
        if interleaving_reuses(&hh.probe) > old_reuses {
            let held = hh.probe.events.lock().unwrap().iter().filter(|e| **e == Event::Held).count();
            hh.probe.wait(|events| events.iter().filter(|e| **e == Event::Held).count() >= held + 2).await;
            assert_eq!(hh.probe.normal.load(Order::SeqCst), hh.calls);
            assert_eq!(hh.harness.scheduler.status().phase, SchedulerPhase::AwaitingPath);
            return;
        }
        hh.published = interleaving_path(&mut hh.harness, &hh.goal,
            hh.published.path_revision + 1).await;
        hh.calls += 1;
        hh.probe.release(gate);
        hh.gate += 1;
    }
}

#[tokio::test]
async fn held_changed_goal_late_old_feedback_is_missing_actual_not_old_owner_reuse() {
    let mut hh = standard_held().await;
    let old_feedback = hh.harness.scheduler.status().surface_replan_feedback.unwrap();
    let old_goal = hh.goal.clone();
    let old_reuses = interleaving_reuses(&hh.probe);
    let b = goal_for(&hh.frozen, old_goal.goal_generation + 1, [1.65, 1.55, 2.5]);
    // A genuine Held means the gate's permit was emitted but no normal job
    // consumed it. Reserve that exact token, then pass it to B's initial work
    // after the typed goal publication. Do not wait for B's first refusal:
    // B cannot refuse before its first path exists.
    let permit = hh.probe.permits.lock().unwrap().try_recv()
        .expect("held A gate retains its unused real-computation permit");
    assert_eq!(permit, hh.gate);
    assert!(hh.probe.permits.lock().unwrap().try_recv().is_err());
    hh.harness.goal(&b).await;
    hh.probe.permit.send(permit).unwrap();
    hh.probe.release(hh.gate); // Ordinary goal input cannot interrupt renew.
    hh.calls += 1;
    hh.gate += 1;
    hh.published = interleaving_path(&mut hh.harness, &b, 1).await;
    hh.goal = b;
    interleaving_converge(&mut hh, old_reuses).await;
    let active_feedback = hh.harness.scheduler.status().surface_replan_feedback.unwrap();
    assert_ne!(active_feedback.path_identity, old_feedback.path_identity);

    let captures_before = hh.probe.captures.lock().unwrap().len();
    let events_before = hh.probe.events.lock().unwrap().len();
    let call = hh.calls + 1;
    let gate = hh.gate;
    // Replay the actual saved A value on the real watch while B is held.
    // Missing current B context must conservatively recompute B; never assert
    // zero work here, because foreign feedback intentionally removes proof.
    hh.harness.hub.execution_feedback.send_replace(Some(old_feedback));
    hh.probe.wait(|events| events.contains(&Event::Computed { call, ok: true, gate })).await;
    {
        let captures = hh.probe.captures.lock().unwrap();
        assert_eq!(captures.len(), captures_before + 1);
        assert!(captures[captures_before].actual.is_none());
        assert!(same_pose(&captures[captures_before].goal, &pose_from_schema(&hh.goal.goal).unwrap()));
        let events = hh.probe.events.lock().unwrap();
        assert!(events[events_before..].contains(&Event::Dispatch { complete: false }));
        assert!(!events[events_before..].contains(&Event::Reuse { complete: true }));
    }
    let published = interleaving_path(&mut hh.harness, &hh.goal, hh.published.path_revision + 1).await;
    assert_eq!(published.planner_runtime_generation, hh.published.planner_runtime_generation);
    assert_eq!(hh.probe.normal.load(Order::SeqCst), call);
    assert_eq!(hh.probe.quick.load(Order::SeqCst), 0);
    assert_eq!(hh.probe.gate.load(Order::SeqCst), gate);
    assert_eq!(hh.harness.scheduler.status().path_revision, hh.published.path_revision as u64);
    hh.finish("changed-goal-late-old-feedback").await;
}

#[tokio::test]
async fn held_real_quick_publication_precedes_late_normal_and_full_budget_wakes() {
    let mut hh = standard_held().await;
    hh.probe.hold_next_normal_return.store(true, Order::SeqCst);
    let old_call = hh.calls + 1;
    let old_gate = hh.gate;
    hh.harness.feed.lock().unwrap().position[0] += 0.1; // Original typed pose feed.
    hh.probe.wait(|events| events.contains(&Event::NormalReturnHeld(old_call))).await;
    hh.probe.wait(|events| events.contains(&Event::Computed {
        call: old_call, ok: true, gate: old_gate,
    })).await;
    let source = changed_terrain(&hh.frozen);
    hh.harness.map(&source).await;
    // Normal's genuine Ok is still held. Thus this original LCM envelope can
    // only be the original quick branch's real budget-0 result.
    hh.probe.wait(|events| events.contains(&Event::QuickComputed { call: 1, ok: true })).await;
    let quick = interleaving_path(&mut hh.harness, &hh.goal, hh.published.path_revision + 1).await;
    assert_eq!(hh.probe.quick.load(Order::SeqCst), 1);
    assert_eq!(hh.probe.normal.load(Order::SeqCst), old_call);
    assert_eq!(hh.probe.gate.load(Order::SeqCst), old_gate);
    hh.probe.release(old_gate);
    hh.gate += 1;
    let quick_gate = hh.gate;
    hh.probe.wait(|events| events.contains(&Event::Gate(quick_gate))).await;
    let feedback = hh.harness.scheduler.status().surface_replan_feedback.unwrap();
    assert_eq!(feedback.path_identity.path_revision, quick.path_revision as u64);
    assert!(feedback.zero_confirmed_rejection);
    assert_eq!(feedback.current_context, feedback.rejected_context);
    let old_reuses = interleaving_reuses(&hh.probe);
    hh.probe.release_return(old_call);
    hh.probe.wait(|events| events.contains(&Event::NormalReturned {
        call: old_call, cancelled: true,
    })).await;
    let full_call = old_call + 1;
    hh.probe.wait(|events| events.contains(&Event::Computed {
        call: full_call, ok: true, gate: quick_gate,
    })).await;
    // A new normal dispatch is a causal join fence: the older normal JoinSet
    // must already have been consumed. No timing-based "no extra path" check.
    hh.published = interleaving_path(&mut hh.harness, &hh.goal, quick.path_revision + 1).await;
    hh.calls = full_call;
    assert_eq!(hh.probe.quick.load(Order::SeqCst), 1);
    assert_eq!(hh.probe.normal.load(Order::SeqCst), full_call);
    hh.probe.release(quick_gate);
    hh.gate += 1;
    interleaving_converge(&mut hh, old_reuses).await;
    hh.finish("quick-wins-late-normal-full-budget-wake").await;
}

// Scope limit: this proves held-memo wakeup + real quick/full interleaving +
// obsolete-result cancellation/publication dominance. Because the original
// quick branch cancels the older normal ticket, it does NOT independently
// isolate a hypothetical memo-only late-write check. Say that explicitly.
