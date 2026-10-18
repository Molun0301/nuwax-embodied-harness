// Selective geometry owner tests. B causal inputs remain in the original file.
// Independent causal owner tests. Real same-context liblcm memq and production
// owner/scheduler are retained; computation hooks count real calls and hold only
// their return boundary. These are not dynamic-plant or release qualification.
use super::*;
use std::collections::BTreeSet;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

const A: [i32; 3] = [71, 11, 47];
// Explicit related changes for held-search tests; the original unrelated A is
// unchanged above and separately required to produce zero normal/quick work.
const RELATED: [i32; 3] = [71, 25, 47];
const B: [i32; 3] = [71, 27, 47];
const C: [i32; 3] = [71, 29, 47];

type Cells = Arc<Mutex<BTreeSet<[i32; 3]>>>;

fn configured(local: bool, full_body: bool) -> SurfaceNavigationConfig {
    let mut configured = config();
    configured.follower.local_planning_enabled = local;
    if full_body {
        configured.apply_calibration(Some(nuwax_nav_types_messages::RobotGeometry {
            footprint_xy_m: vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
            z_min_m: -0.05, z_max_m: 0.2, source: "OFF soft-refresh owner fixture".into(),
        }), None).unwrap();
    }
    if local && full_body {
        // Only the ON control arm needs Scan execution capabilities. Preserve
        // the existing body/terrain/goal fixture and declare synthetic limits.
        let motion=nuwax_nav_types_messages::MotionLimits {
            model:nuwax_nav_types_messages::MotionModel::Differential,
            max_forward_mps:0.5,max_reverse_mps:0.0,max_lateral_mps:0.0,max_angular_rps:0.8,
            linear_acceleration_mps2:1.0,guaranteed_braking_mps2:1.0,
            angular_acceleration_rps2:2.0,guaranteed_angular_braking_rps2:2.0,
            reaction_time_sec:0.05,minimum_turning_radius_m:0.0,can_rotate_in_place:true,
        };
        motion.validate().unwrap();
        configured.follower.motion_limits=Some(motion);
        assert!(configured.follower.kinematics().is_some());
        assert!(!configured.follower.controller.simulation);
    }
    configured
}

type Layers = Arc<Mutex<std::collections::BTreeMap<[i32; 3], nuwax_map_chunks::LayerObservations>>>;
async fn start(normal: SurfacePathComputer, quick: SurfacePathComputer,
    local: bool, full_body: bool) -> (Harness, Cells)
{
    start_with_layers(normal, quick, local, full_body, Arc::default()).await
}
async fn start_with_layers(normal: SurfacePathComputer, quick: SurfacePathComputer,
    local: bool, full_body: bool, layers: Layers) -> (Harness, Cells)
{
    let harness = Harness::start_with_test_transport(normal, Duration::from_millis(25),
        configured(local, full_body), true, Some(quick)).await;
    let cells: Cells = Arc::new(Mutex::new((0..80).flat_map(|x|(0..60).map(move|y|[x,y,40])).collect()));
    let observed = cells.clone();
    let repository = nuwax_map_chunks::Repository::temporary().unwrap();
    let mut writer = nuwax_map_chunks::Writer::new(repository.clone(),0.05,4*1024*1024).unwrap();
    let mut previous = BTreeSet::new();
    harness.feed.lock().unwrap().map_update = Some(Box::new(move |map| {
        let cells = observed.lock().unwrap().clone();
        writer.remove(previous.difference(&cells).copied()).unwrap();
        writer.observe(cells.iter().copied(),map.source_timestamp_ns as u64).unwrap();
        for (block, records) in layers.lock().unwrap().iter() {
            writer.replace_layer_observations(*block, records.clone()).unwrap();
        }
        map.source_voxel_size = 0.05;
        map.cloud.width = 0; map.cloud.row_step = 0; map.cloud.data.clear();
        let root = writer.snapshot(schema::navigation_map_3d_domain(map),map.cloud_revision as u64,
            map.source_timestamp_ns as u64).unwrap();
        let included = root.blocks.iter().map(|block|(block.object,
            repository.read_object(block.object,nuwax_map_chunks::MAX_BLOCK_BYTES).unwrap())).collect();
        map.fine = Some(nuwax_map_chunks::Transfer {root:root.encode().unwrap(),included});
        previous = cells;
    }));
    wait(|| latest(&harness).is_some_and(|surface| surface.observations().is_some()) || !full_body
        && harness.hub.latest.lock().unwrap().as_ref().is_some_and(|m|m.snapshot.is_ok())).await;
    harness.wait_ready().await;
    (harness,cells)
}

fn latest(harness: &Harness) -> Option<Arc<SurfaceSnapshot>> {
    harness.hub.latest.lock().unwrap().as_ref().and_then(|map|map.snapshot.as_ref().ok().cloned())
}

async fn wait(condition: impl Fn() -> bool) {
    tokio::time::timeout(Duration::from_secs(4),async {
        while !condition() {tokio::time::sleep(Duration::from_millis(5)).await;}
    }).await.expect("owner barrier was not reached");
}

async fn evidence_updates(harness: &Harness, count: i64) {
    let before = harness.hub.latest.lock().unwrap().as_ref().unwrap().authority.cloud_revision;
    wait(|| harness.hub.latest.lock().unwrap().as_ref().unwrap().authority.cloud_revision >= before+count).await;
}

fn contains(surface: &SurfaceSnapshot, cell: [i32;3]) -> bool {
    surface.observations().is_some_and(|observations|observations.get(cell).is_some())
}

fn hold(release: &AtomicBool) {
    let deadline = Instant::now()+Duration::from_secs(10);
    while !release.load(Ordering::Acquire) {
        assert!(Instant::now()<deadline,"test computation return barrier expired");
        std::thread::sleep(Duration::from_millis(1));
    }
}

fn counted(counter: Arc<AtomicUsize>, quick: bool) -> SurfacePathComputer {
    Arc::new(move |map,pose,goal,configured| {
        counter.fetch_add(1,Ordering::SeqCst);
        if quick {body_path_with_budget(map,pose,goal,configured,0)}
        else {body_path(map,pose,goal,configured)}
    })
}

async fn active(harness: &mut Harness) -> (schema::GoalEnvelope, schema::PathEnvelope) {
    let goal = goal_for(&harness.feed.lock().unwrap().map,1,[3.2,1.55,2.5]);
    harness.goal(&goal).await;
    let path = harness.path_for(&goal).await;
    executed_revision(harness,&goal,path.path_revision).await;
    (goal,path)
}

async fn wait_execution(harness: &Harness, label: &str, condition: impl Fn() -> bool) {
    tokio::time::timeout(Duration::from_secs(4),async {
        while !condition() {tokio::time::sleep(Duration::from_millis(5)).await;}
    }).await.unwrap_or_else(|_|panic!("{label}: scheduler={:?}",harness.scheduler.status()));
}

async fn executed_revision(harness: &Harness, goal: &schema::GoalEnvelope, revision: i64) {
    wait_execution(harness,"matching goal/path revision was not accepted",|| {
        let status=harness.scheduler.status();
        status.session_id.as_deref()==Some(goal.session_id.as_str()) && status.path_revision==revision as u64
    }).await;
    let accepted_commands=harness.scheduler.status().applied_commands;
    wait_execution(harness,"accepted revision produced no newer proved nonzero command",|| {
        let status=harness.scheduler.status();
        status.session_id.as_deref()==Some(goal.session_id.as_str()) && status.path_revision==revision as u64
            && status.phase==SchedulerPhase::Controlling && status.applied_commands>accepted_commands
            && status.last_applied_twist.is_some_and(|twist|twist!=Twist2::ZERO)
    }).await;
}

async fn unchanged_path(harness: &mut Harness, revision: i64) {
    assert!(tokio::time::timeout(Duration::from_millis(80),harness.paths.next()).await.is_err(),
        "a held/stale result must not publish a replacement");
    let status = harness.scheduler.status();
    assert_eq!(status.path_revision,revision as u64);
    assert_eq!(status.phase,SchedulerPhase::Controlling);
    assert!(status.planner_inputs_fresh);
}

#[tokio::test]
async fn selective_refresh_original_unrelated_a_and_evidence_add_zero_planner_work() {
    let normal = Arc::new(AtomicUsize::new(0));
    let quick = Arc::new(AtomicUsize::new(0));
    let (mut harness,cells) = start(counted(normal.clone(),false),counted(quick.clone(),true),false,true).await;
    let (goal,initial) = active(&mut harness).await;
    let first = latest(&harness).unwrap();
    let first_evidence = first.observations().unwrap().get([40,30,40]).unwrap();
    evidence_updates(&harness,3).await;
    let refreshed = latest(&harness).unwrap();
    let new_evidence = refreshed.observations().unwrap().get([40,30,40]).unwrap();
    assert!(new_evidence.observed_ns>first_evidence.observed_ns && new_evidence.hits>first_evidence.hits);
    let evidence_observation_owner_changed = !Arc::ptr_eq(first.observations().unwrap(), refreshed.observations().unwrap());
    let evidence_query_view_changed = !first.same_body_query_view(&refreshed);
    assert!(nuwax_nav_pct::same_source_geometry(&first,&refreshed));
    assert_eq!(harness.hub.reference_checks.load(Ordering::SeqCst),0);
    cells.lock().unwrap().insert(A);
    wait(||latest(&harness).is_some_and(|surface|contains(&surface,A))).await;
    wait(||harness.hub.reference_checks.load(Ordering::SeqCst)==1).await;
    evidence_updates(&harness,3).await;
    unchanged_path(&mut harness,initial.path_revision).await;
    assert_eq!((normal.load(Ordering::SeqCst),quick.load(Ordering::SeqCst)),(1,0));
    assert_eq!(harness.hub.reference_checks.load(Ordering::SeqCst),1);
    eprintln!("SELECTIVE_A coordinate={A:?} selector=1 normal_extra=0 quick_extra=0 evidence_selector_extra=0 path_revision={} body_calls=structurally_absent_not_profiled",initial.path_revision);
    cells.lock().unwrap().insert(RELATED);
    let replacement=harness.path_for(&goal).await;
    executed_revision(&harness,&goal,replacement.path_revision).await;
    evidence_updates(&harness,2).await;
    assert_eq!((normal.load(Ordering::SeqCst),quick.load(Ordering::SeqCst)),(2,0));
    assert_eq!(harness.hub.reference_checks.load(Ordering::SeqCst),2);
    assert_eq!(replacement.path_revision,initial.path_revision+1);
    eprintln!("SELECTIVE_RELATED coordinate={RELATED:?} normal_extra=1 quick_extra=0 path_extra=1");
    harness.cancel(&goal).await; harness.terminal(TerminalOutcome::Cancelled).await; harness.shutdown().await;
    assert!(evidence_observation_owner_changed);
    assert!(evidence_query_view_changed, "truthful evidence must invalidate a complete query view");
}

#[tokio::test]
async fn selective_refresh_source_watermarks_add_zero_geometry_or_planner_work() {
    let normal = Arc::new(AtomicUsize::new(0));
    let quick = Arc::new(AtomicUsize::new(0));
    let mut harness = Harness::start_with_test_transport(counted(normal.clone(), false),
        Duration::from_millis(25), configured(false, true), true,
        Some(counted(quick.clone(), true))).await;
    let source = Arc::new(nuwax_map_chunks::LayerSource::Auxiliary {
        sensor_id: "depth-front".into(), clock_id: "camera-clock".into(),
        runtime_generation: 7, session_generation: 1,
    });
    let marks = Arc::new(Mutex::new(vec![nuwax_map_chunks::SourceWatermark {
        source: source.clone(), source_timestamp_ns: 90, source_sequence: 0,
    }]));
    let observed_marks = marks.clone();
    let published: Arc<Mutex<Option<Arc<nuwax_map_chunks::Manifest>>>> = Arc::default();
    let observed_root = published.clone();
    let repository = nuwax_map_chunks::Repository::temporary().unwrap();
    let mut writer = nuwax_map_chunks::Writer::new(repository.clone(), 0.05, 4 * 1024 * 1024).unwrap();
    let mut initialized = false;
    harness.feed.lock().unwrap().map_update = Some(Box::new(move |map| {
        if !initialized {
            writer.observe((0..80).flat_map(|x| (0..60).map(move |y| [x, y, 40])),
                map.source_timestamp_ns as u64).unwrap();
            initialized = true;
        }
        for mark in observed_marks.lock().unwrap().iter() {
            writer.record_source_watermark(mark.source.clone(), mark.source_timestamp_ns,
                mark.source_sequence).unwrap();
        }
        map.source_voxel_size = 0.05;
        map.cloud.width = 0; map.cloud.row_step = 0; map.cloud.data.clear();
        let root = writer.snapshot(schema::navigation_map_3d_domain(map), map.cloud_revision as u64,
            map.source_timestamp_ns as u64).unwrap();
        let included = root.blocks.iter().map(|block| (block.object,
            repository.read_object(block.object, nuwax_map_chunks::MAX_BLOCK_BYTES).unwrap())).collect();
        map.fine = Some(nuwax_map_chunks::Transfer { root: root.encode().unwrap(), included });
        *observed_root.lock().unwrap() = Some(root);
    }));
    wait(|| latest(&harness).is_some_and(|map| map.observations().is_some())).await;
    harness.wait_ready().await;
    let (goal, initial) = active(&mut harness).await;
    let first = latest(&harness).unwrap();
    let first_root = published.lock().unwrap().as_ref().unwrap().clone();
    let first_evidence = first.observations().unwrap().get([40, 30, 40]);
    let before_commands = harness.scheduler.status().applied_commands;
    {
        let mut marks = marks.lock().unwrap();
        marks[0].source_timestamp_ns = 900;
        marks[0].source_sequence = 1;
        marks.push(nuwax_map_chunks::SourceWatermark {
            source: Arc::new(nuwax_map_chunks::LayerSource::Auxiliary {
                sensor_id: "depth-front".into(), clock_id: "other-clock".into(),
                runtime_generation: 7, session_generation: 1,
            }), source_timestamp_ns: 2, source_sequence: 0,
        });
    }
    evidence_updates(&harness, 3).await;
    let refreshed = latest(&harness).unwrap();
    let refreshed_root = published.lock().unwrap().as_ref().unwrap().clone();
    assert_eq!(refreshed_root.source_watermarks.len(), 2);
    assert!(refreshed_root.source_watermarks.iter().any(|mark| mark.source == source
        && mark.source_timestamp_ns == 900 && mark.source_sequence == 1));
    assert_ne!(first_root.source_watermarks, refreshed_root.source_watermarks);
    assert_eq!(first_root.blocks, refreshed_root.blocks);
    assert_eq!(first_root.geometry_revision, refreshed_root.geometry_revision);
    assert_eq!(first_evidence, refreshed.observations().unwrap().get([40, 30, 40]));
    let same_observation_owner = Arc::ptr_eq(first.observations().unwrap(), refreshed.observations().unwrap());
    let same_query_view = first.same_body_query_view(&refreshed);
    assert!(nuwax_nav_pct::same_source_geometry(&first, &refreshed));
    unchanged_path(&mut harness, initial.path_revision).await;
    assert_eq!((normal.load(Ordering::SeqCst), quick.load(Ordering::SeqCst)), (1, 0));
    assert_eq!(harness.hub.reference_checks.load(Ordering::SeqCst), 0);
    assert!(harness.scheduler.status().applied_commands > before_commands);
    harness.cancel(&goal).await;
    harness.terminal(TerminalOutcome::Cancelled).await;
    harness.shutdown().await;
    assert!(same_observation_owner,
        "F2: watermark-only Writer publication must retain observations through the real map owner");
    assert!(same_query_view);
}

#[tokio::test]
async fn selective_refresh_fences_both_old_completions_and_coalesces_held_burst() {
    let normal = Arc::new(AtomicUsize::new(0));
    let quick = Arc::new(AtomicUsize::new(0));
    let normal_release = Arc::new(AtomicBool::new(false));
    let quick_old_release = Arc::new(AtomicBool::new(false));
    let quick_new_release = Arc::new(AtomicBool::new(false));
    let normal_returned = Arc::new(AtomicBool::new(false));
    let (n,nr,returned) = (normal.clone(),normal_release.clone(),normal_returned.clone());
    let computer: SurfacePathComputer = Arc::new(move |map,pose,goal,configured| {
        let call = n.fetch_add(1,Ordering::SeqCst)+1;
        let result = body_path(map,pose,goal,configured);
        if call==2 {assert!(contains(map,RELATED) && !contains(map,B));hold(&nr);returned.store(true,Ordering::Release);}
        result
    });
    let (q,old,new) = (quick.clone(),quick_old_release.clone(),quick_new_release.clone());
    let quick_computer: SurfacePathComputer = Arc::new(move |map,pose,goal,configured| {
        let call = q.fetch_add(1,Ordering::SeqCst)+1;
        let result = body_path_with_budget(map,pose,goal,configured,0);
        assert!(result.is_ok(),"barrier fixture requires real successful quick calculation");
        match call {
            1 => {assert!(contains(map,B) && !contains(map,C));hold(&old);}
            2 => {assert!(contains(map,C));hold(&new);}
            _ => panic!("burst was not coalesced"),
        }
        result
    });
    let (mut harness,cells) = start(computer,quick_computer,false,true).await;
    let (goal,initial) = active(&mut harness).await;
    cells.lock().unwrap().insert(RELATED);wait(||normal.load(Ordering::SeqCst)==2).await;
    cells.lock().unwrap().insert(B);wait(||quick.load(Ordering::SeqCst)==1).await;
    cells.lock().unwrap().insert(C);wait(||latest(&harness).is_some_and(|surface|contains(&surface,C))).await;
    evidence_updates(&harness,2).await;
    unchanged_path(&mut harness,initial.path_revision).await;
    quick_old_release.store(true,Ordering::Release);
    wait(||quick.load(Ordering::SeqCst)==2).await;
    unchanged_path(&mut harness,initial.path_revision).await;
    normal_release.store(true,Ordering::Release);
    wait(||normal_returned.load(Ordering::Acquire)).await;
    unchanged_path(&mut harness,initial.path_revision).await;
    quick_new_release.store(true,Ordering::Release);
    let replacement = harness.path_for(&goal).await;
    executed_revision(&harness,&goal,replacement.path_revision).await;
    assert_eq!(replacement.path_revision,initial.path_revision+1);
    assert_eq!((normal.load(Ordering::SeqCst),quick.load(Ordering::SeqCst)),(2,2));
    eprintln!("OFF_SOFT_BURST geometry_events=3 normal=2 quick=2 old_normal_published=0 old_quick_published=0 latest_source_C_executed=true");
    harness.cancel(&goal).await;harness.terminal(TerminalOutcome::Cancelled).await;harness.shutdown().await;
}

#[tokio::test]
async fn selective_refresh_failure_keeps_path_and_cancel_retires_late_result() {
    let calls = Arc::new(AtomicUsize::new(0));
    let fail = Arc::new(AtomicBool::new(true));
    let release = Arc::new(AtomicBool::new(false));
    let retired = Arc::new(AtomicBool::new(false));
    let (n,f,r,c) = (calls.clone(),fail.clone(),release.clone(),retired.clone());
    let computer: SurfacePathComputer = Arc::new(move |map,pose,goal,configured| {
        let call = n.fetch_add(1,Ordering::SeqCst)+1;
        let result = body_path(map,pose,goal,configured);
        if call==2 && f.load(Ordering::Acquire) {return Err(module_error("soft-refresh test","injected completed-search failure"));}
        if call==3 {
            let deadline=Instant::now()+Duration::from_secs(10);
            while !configured.planning_cancelled.as_ref().unwrap().load(Ordering::Acquire) {
                assert!(Instant::now()<deadline);std::thread::sleep(Duration::from_millis(1));
            }
            c.store(true,Ordering::Release);hold(&r);
        }
        result
    });
    let (mut harness,cells) = start(computer,counted(Arc::new(AtomicUsize::new(0)),true),false,true).await;
    let (goal,initial) = active(&mut harness).await;
    cells.lock().unwrap().insert(RELATED);wait(||calls.load(Ordering::SeqCst)==2).await;
    unchanged_path(&mut harness,initial.path_revision).await;
    assert!(harness.hub.planning.borrow().as_ref().is_some_and(|(_,_,message)|message.contains("injected completed-search failure")));
    fail.store(false,Ordering::Release);
    wait(||calls.load(Ordering::SeqCst)==3).await;
    harness.cancel(&goal).await;harness.terminal(TerminalOutcome::Cancelled).await;
    wait(||retired.load(Ordering::Acquire)).await;
    release.store(true,Ordering::Release);
    assert!(tokio::time::timeout(Duration::from_millis(150),harness.paths.next()).await.is_err());
    let next = goal_for(&harness.feed.lock().unwrap().map,2,[3.2,1.55,2.5]);
    harness.goal(&next).await;let path=harness.path_for(&next).await;
    assert_eq!(path.goal_key(),next.goal_key());
    executed_revision(&harness,&next,path.path_revision).await;
    eprintln!("OFF_SOFT_FAILURE kept_active_revision={} cancel_retired_late_result=true next_goal_executed=true normal={}",initial.path_revision,calls.load(Ordering::SeqCst));
    harness.cancel(&next).await;harness.terminal(TerminalOutcome::Cancelled).await;harness.shutdown().await;
}

#[tokio::test]
async fn selective_refresh_records_inherited_weak_ticket_quick_repeat_cost() {
    let normal = Arc::new(AtomicUsize::new(0));
    let quick = Arc::new(AtomicUsize::new(0));
    let release = Arc::new(AtomicBool::new(false));
    let (n,r)=(normal.clone(),release.clone());
    let computer: SurfacePathComputer=Arc::new(move |map,pose,goal,configured| {
        let call=n.fetch_add(1,Ordering::SeqCst)+1;
        let result=body_path(map,pose,goal,configured);
        if call==2 {hold(&r);}result
    });
    let q=quick.clone();
    let quick_computer: SurfacePathComputer=Arc::new(move |map,pose,goal,configured| {
        q.fetch_add(1,Ordering::SeqCst);
        assert!(contains(map,B));
        let _real_result=body_path_with_budget(map,pose,goal,configured,0);
        Err(module_error("soft-refresh cost probe","injected quick failure after real calculation"))
    });
    let (mut harness,cells)=start(computer,quick_computer,false,true).await;
    let (goal,initial)=active(&mut harness).await;
    cells.lock().unwrap().insert(RELATED);wait(||normal.load(Ordering::SeqCst)==2).await;
    cells.lock().unwrap().insert(B);wait(||quick.load(Ordering::SeqCst)>=1).await;
    let before=quick.load(Ordering::SeqCst);
    evidence_updates(&harness,4).await;
    unchanged_path(&mut harness,initial.path_revision).await;
    let after=quick.load(Ordering::SeqCst);
    assert!(after>=before);
    eprintln!("OFF_SOFT_WEAK_COST held_normal=1 geometry_events=2 evidence_updates=4 quick_before={before} quick_after={after} repeated_same_geometry_quick={} release_qualified=false",after-before);
    harness.cancel(&goal).await;harness.terminal(TerminalOutcome::Cancelled).await;
    release.store(true,Ordering::Release);harness.shutdown().await;
}

#[tokio::test]
async fn selective_refresh_scope_preserves_on_and_legacy_no_early_replan() {
    // The mature distant-route oracle remains unchanged. This explicitly extends
    // its no-early-replan expectation to a full-body ON control arm.
    for (local,full_body) in [(true,true),(false,false)] {
        let normal=Arc::new(AtomicUsize::new(0));let quick=Arc::new(AtomicUsize::new(0));
        let (mut harness,cells)=start(counted(normal.clone(),false),counted(quick.clone(),true),local,full_body).await;
        let (goal,initial)=active(&mut harness).await;
        cells.lock().unwrap().insert(A);evidence_updates(&harness,4).await;
        unchanged_path(&mut harness,initial.path_revision).await;
        assert_eq!((normal.load(Ordering::SeqCst),quick.load(Ordering::SeqCst)),(1,0));
        eprintln!("OFF_SOFT_SCOPE local={local} full_body={full_body} normal=1 quick=0");
        harness.cancel(&goal).await;harness.terminal(TerminalOutcome::Cancelled).await;harness.shutdown().await;
    }
}

#[test]
fn selective_refresh_source_predicate_detects_fine_only_membership() {
    use nuwax_nav_mls_planner::{SurfaceObservation, SurfaceObservations};
    let configured=configured(false,true);
    let base=SurfaceGeometryCache::configured(&configured).build(&map_message(1,"fine-predicate")).unwrap();
    let mut observations=SurfaceObservations::new(0.05,8).unwrap();
    let first_evidence=SurfaceObservation {observed_ns:10,hits:1};
    observations.replace_block([0,0,0],std::collections::BTreeMap::from([([1,1,1],first_evidence)])).unwrap();
    observations.mark_complete_geometry_source();
    let initial=base.with_observations(Some(Arc::new(observations.clone())));
    let new_evidence=SurfaceObservation {observed_ns:20,hits:2};
    observations.replace_block([0,0,0],std::collections::BTreeMap::from([([1,1,1],new_evidence)])).unwrap();
    observations.mark_complete_geometry_source();
    let evidence_only=base.with_observations(Some(Arc::new(observations.clone())));
    assert!(initial.same_geometry(&evidence_only));
    assert!(!off_source_geometry_changed(&initial,&evidence_only,&configured));
    observations.replace_block([0,0,0],std::collections::BTreeMap::from([
        ([1,1,1],new_evidence),([1,1,2],new_evidence)])).unwrap();
    observations.mark_complete_geometry_source();
    let fine_changed=base.with_observations(Some(Arc::new(observations)));
    assert!(initial.same_geometry(&fine_changed),"coarse surface alone misses the changed fine source");
    assert!(off_source_geometry_changed(&initial,&fine_changed,&configured));
}

struct FineFixture {
    repository: nuwax_map_chunks::Repository,
    writer: nuwax_map_chunks::Writer,
    cells: BTreeSet<[i32; 3]>,
    previous: BTreeSet<[i32; 3]>,
    revision: i64,
}
impl FineFixture {
    fn new() -> Self {
        let repository=nuwax_map_chunks::Repository::temporary().unwrap();
        Self { writer:nuwax_map_chunks::Writer::new(repository.clone(),0.05,4*1024*1024).unwrap(),
            repository, cells:(0..80).flat_map(|x|(0..60).map(move|y|[x,y,40])).collect(),
            previous:BTreeSet::new(),revision:0 }
    }
    fn next(&mut self) -> schema::NavigationMap3D {
        self.revision+=1;
        let mut message=map_message(self.revision,"selective-cache-fixture");
        self.writer.remove(self.previous.difference(&self.cells).copied()).unwrap();
        self.writer.observe(self.cells.iter().copied(),message.source_timestamp_ns as u64).unwrap();
        message.source_voxel_size=0.05;
        message.cloud.width=0;message.cloud.row_step=0;message.cloud.data.clear();
        let root=self.writer.snapshot(schema::navigation_map_3d_domain(&message),self.revision as u64,
            message.source_timestamp_ns as u64).unwrap();
        let included=root.blocks.iter().map(|block|(block.object,self.repository
            .read_object(block.object,nuwax_map_chunks::MAX_BLOCK_BYTES).unwrap())).collect();
        message.fine=Some(nuwax_map_chunks::Transfer {root:root.encode().unwrap(),included});
        self.previous=self.cells.clone();
        schema::validate_navigation_map_3d(&message).unwrap();
        message
    }
}

#[test]
fn selective_refresh_fine_delta_accumulates_all_blocks_and_last_layer_deletions() {
    let configured=configured(false,true);
    let mut cache=SurfaceGeometryCache::configured(&configured);
    let mut fixture=FineFixture::new();
    fixture.cells.extend([[-1,-1,70],[33,33,70],[33,33,100]]);
    let first=cache.build_with_changes(&fixture.next()).unwrap();
    assert!(matches!(first.changes.as_ref(),SurfaceGeometryChanges::Unknown));
    fixture.cells.remove(&[-1,-1,70]);
    fixture.cells.remove(&[33,33,70]);fixture.cells.remove(&[33,33,100]);
    fixture.cells.remove(&[40,30,40]);fixture.cells.insert([65,1,70]);
    let second=cache.build_with_changes(&fixture.next()).unwrap();
    assert!(nuwax_nav_pct::same_source_geometry(second.change_basis.as_ref().unwrap(),&first.snapshot));
    let SurfaceGeometryChanges::Columns {fine,coarse,..}=second.changes.as_ref() else {panic!("complete compatible delta required")};
    assert_eq!(fine.iter().copied().collect::<BTreeSet<_>>(),BTreeSet::from([[-1,-1],[33,33],[40,30],[65,1]]));
    assert!(coarse.contains(&[-1,-1]),"removed final fine column must remove its coarse column");
    assert!(second.snapshot.observations().unwrap().get([-1,-1,70]).is_none());
    let metadata=cache.build_with_changes(&fixture.next()).unwrap();
    assert!(matches!(metadata.changes.as_ref(),SurfaceGeometryChanges::Empty));
    assert!(nuwax_nav_pct::same_source_geometry(&second.snapshot,&metadata.snapshot));
    eprintln!("SELECTIVE_MULTIBLOCK fine_columns={fine:?} coarse_columns={coarse:?} deletion_and_metadata=true");
}

#[test]
fn selective_refresh_fine_only_delta_survives_unchanged_coarse_snapshot() {
    let configured=configured(false,true);
    let mut cache=SurfaceGeometryCache::configured(&configured);
    let mut fixture=FineFixture::new();
    let first=cache.build_with_changes(&fixture.next()).unwrap();
    fixture.cells.remove(&[40,30,40]);
    let second=cache.build_with_changes(&fixture.next()).unwrap();
    assert!(first.snapshot.same_geometry(&second.snapshot));
    assert!(!nuwax_nav_pct::same_source_geometry(&first.snapshot,&second.snapshot));
    let SurfaceGeometryChanges::Columns {fine,coarse,..}=second.changes.as_ref() else {panic!("fine-only scope lost")};
    assert_eq!(fine,&vec![[40,30]]);assert!(coarse.is_empty());
}

#[test]
fn selective_refresh_failed_physical_advance_is_unknown_once_then_recovers_known() {
    let configured=configured(false,true);
    let mut cache=SurfaceGeometryCache::configured(&configured);
    let mut fixture=FineFixture::new();
    let first=cache.build_with_changes(&fixture.next()).unwrap();
    let physical_before=cache.physical_snapshot().unwrap();
    fixture.cells.insert(A);
    let mut broken=fixture.next();
    let mut mask=virtual_mask(&broken,false);
    mask.info.origin.position[0]=f64::MAX;
    broken.virtual_obstacles=Some(mask);
    // This is a legal wire message but the mask's finite extent is not
    // representable by the derived map. It fails after physical advances.
    schema::validate_navigation_map_3d(&broken).unwrap();
    assert!(cache.build_with_changes(&broken).is_err());
    assert!(!Arc::ptr_eq(&physical_before,&cache.physical_snapshot().unwrap()));
    let recovered=cache.build_with_changes(&fixture.next()).unwrap();
    assert!(nuwax_nav_pct::same_source_geometry(recovered.change_basis.as_ref().unwrap(),&first.snapshot));
    assert!(matches!(recovered.changes.as_ref(),SurfaceGeometryChanges::Unknown));
    let metadata=cache.build_with_changes(&fixture.next()).unwrap();
    assert!(matches!(metadata.changes.as_ref(),SurfaceGeometryChanges::Empty));
    fixture.cells.insert(RELATED);
    let known=cache.build_with_changes(&fixture.next()).unwrap();
    let SurfaceGeometryChanges::Columns {fine,..}=known.changes.as_ref() else {panic!("successful recovery must restore known basis")};
    assert_eq!(fine,&vec![[RELATED[0],RELATED[1]]]);
    eprintln!("SELECTIVE_BASIS fail_after_physical=true recovered_unknown=true next_metadata_empty=true next_geometry_known=true");
}

#[tokio::test]
async fn selective_refresh_removed_obstacle_and_fine_only_hole_each_request_once() {
    let normal=Arc::new(AtomicUsize::new(0));let quick=Arc::new(AtomicUsize::new(0));
    let (mut harness,cells)=start(counted(normal.clone(),false),counted(quick.clone(),true),false,true).await;
    // Install the related obstacle before the active reference is computed.
    cells.lock().unwrap().insert(RELATED);
    wait(||latest(&harness).is_some_and(|map|contains(&map,RELATED))).await;
    let (goal,initial)=active(&mut harness).await;
    cells.lock().unwrap().remove(&RELATED);
    let removed=harness.path_for(&goal).await;
    executed_revision(&harness,&goal,removed.path_revision).await;
    assert_eq!(removed.path_revision,initial.path_revision+1);
    let before=latest(&harness).unwrap();
    cells.lock().unwrap().remove(&[40,30,40]);
    wait(||latest(&harness).is_some_and(|map|!contains(&map,[40,30,40]))).await;
    let after=latest(&harness).unwrap();
    assert!(before.same_geometry(&after),"fine-only test must retain coarse geometry identity");
    assert!(!nuwax_nav_pct::same_source_geometry(&before,&after));
    let fine=harness.path_for(&goal).await;
    executed_revision(&harness,&goal,fine.path_revision).await;
    evidence_updates(&harness,3).await;
    assert_eq!(fine.path_revision,removed.path_revision+1);
    assert_eq!((normal.load(Ordering::SeqCst),quick.load(Ordering::SeqCst)),(3,0));
    assert_eq!(harness.hub.reference_checks.load(Ordering::SeqCst),2);
    eprintln!("SELECTIVE_DELETION removed_obstacle_request=1 fine_only_request=1 normal_total=3 quick_total=0 selector_total=2");
    harness.cancel(&goal).await;harness.terminal(TerminalOutcome::Cancelled).await;harness.shutdown().await;
}

#[test]
fn selective_refresh_bbox_work_is_bounded_before_scanning_and_basis_is_required() {
    let configured=configured(false,true);
    let mut cache=SurfaceGeometryCache::configured(&configured);
    let mut fixture=FineFixture::new();
    let message=fixture.next();
    let surface=cache.build(&message).unwrap();
    let goal=goal_for(&message,1,[3.2,1.55,2.5]);
    let pose=pose_from_schema(&goal.goal).unwrap();
    let vertices=surface.body_envelope().unwrap().footprint_xy_m.len();
    let changes=SurfaceGeometryChanges::Columns {fine_size:0.05,fine:vec![[A[0],A[1]]],planner_size:0.1,coarse:vec![]};
    let mut path=Path3D {poses:vec![pose;MAX_REFERENCE_SCOPE_WORK-vertices-1]};
    let started=Instant::now();
    assert!(!reference_changes_affect_path(&surface,&surface,Some(&surface),&changes,&path));
    let within_us=started.elapsed().as_micros();
    path.poses.push(path.poses[0].clone());
    let started=Instant::now();
    assert!(reference_changes_affect_path(&surface,&surface,Some(&surface),&changes,&path));
    let exceeded_us=started.elapsed().as_micros();
    path.poses.truncate(1);
    let many_columns=SurfaceGeometryChanges::Columns {fine_size:0.05,
        fine:vec![[A[0],A[1]];MAX_REFERENCE_SCOPE_WORK-vertices-1],planner_size:0.1,coarse:vec![]};
    let many_poses=Path3D {poses:vec![path.poses[0].clone();MAX_REFERENCE_SCOPE_WORK-vertices-1]};
    for (label,scope,reference) in [("many_poses",&changes,&many_poses),("many_columns",&many_columns,&path)] {
        let start=Instant::now();let mut max_ns=0;
        for _ in 0..64 {
            let call=Instant::now();
            let affected=std::hint::black_box(reference_changes_affect_path(
                std::hint::black_box(surface.as_ref()),std::hint::black_box(surface.as_ref()),
                Some(std::hint::black_box(surface.as_ref())),std::hint::black_box(scope),std::hint::black_box(reference)));
            assert!(!affected);
            max_ns=max_ns.max(call.elapsed().as_nanos());
        }
        eprintln!("SELECTIVE_BUDGET_BATCH arm={label} calls=64 work_per_call=4096 max_ns={max_ns} total_ns={} hard_realtime_claim=false",start.elapsed().as_nanos());
    }
    if let SurfaceGeometryChanges::Columns {fine_size,fine,planner_size,coarse}=many_columns {
        let mut fine=fine;fine.push([A[0],A[1]]);
        assert!(reference_changes_affect_path(&surface,&surface,Some(&surface),
            &SurfaceGeometryChanges::Columns {fine_size,fine,planner_size,coarse},&path));
    }
    assert!(reference_changes_affect_path(&surface,&surface,None,&changes,&path));
    assert!(!reference_changes_affect_path(&surface,&surface,Some(&surface),&SurfaceGeometryChanges::Empty,&path));
    assert!(reference_changes_affect_path(&surface,&surface,Some(&surface),&SurfaceGeometryChanges::Unknown,&path));
    eprintln!("SELECTIVE_BOUNDARY exact_work={} exact_work_us={within_us} excess_work={} excess_unknown_us={exceeded_us} broken_basis_unknown=true",MAX_REFERENCE_SCOPE_WORK,MAX_REFERENCE_SCOPE_WORK+1);
}

#[tokio::test]
async fn selective_refresh_map_delivery_keeps_original_route_cancel_responsive() {
    let normal=Arc::new(AtomicUsize::new(0));let quick=Arc::new(AtomicUsize::new(0));
    let (mut harness,cells)=start(counted(normal.clone(),false),counted(quick.clone(),true),false,true).await;
    let (goal,_)=active(&mut harness).await;
    let zero_before=harness.scheduler.status().last_zero_ack_id.clone();
    cells.lock().unwrap().insert(A);
    wait(||harness.hub.reference_checks.load(Ordering::SeqCst)==1).await;
    assert!(latest(&harness).is_some_and(|map|contains(&map,A)));
    assert_eq!((normal.load(Ordering::SeqCst),quick.load(Ordering::SeqCst)),(1,0));
    let cancel_started=Instant::now();
    harness.cancel(&goal).await;harness.terminal(TerminalOutcome::Cancelled).await;
    let status=harness.scheduler.status();
    assert!(status.last_zero_ack_id.is_some() && status.last_zero_ack_id!=zero_before);
    eprintln!("SELECTIVE_CANCEL original_route=true selector=1 cancel_to_terminal_ms={} new_zero_proven=true full_budget_end_to_end_claim=false",cancel_started.elapsed().as_secs_f64()*1000.0);
    harness.shutdown().await;
}

const EXPECT_COMPLETION_REUSE:bool=true;

// New causal reproduction. Holds a successful actual calculation at its return
// boundary; the controlled quick error prevents a different lane winning.
#[tokio::test]
async fn completion_fence_single_disjoint_geometry_and_metadata() {
    let calls=Arc::new(AtomicUsize::new(0));
    let held=Arc::new(AtomicBool::new(false));
    let release_old=Arc::new(AtomicBool::new(false));
    let release_fresh=Arc::new(AtomicBool::new(false));
    let (n,h,old,fresh)=(calls.clone(),held.clone(),release_old.clone(),release_fresh.clone());
    let computer:SurfacePathComputer=Arc::new(move |map,pose,goal,configured| {
        let call=n.fetch_add(1,Ordering::SeqCst)+1;
        let result=body_path(map,pose,goal,configured);
        assert!(result.is_ok(),"causal fixture needs actual successful normal route");
        if call==2 {assert!(contains(map,RELATED) && !contains(map,A)); h.store(true,Ordering::Release);hold(&old);}
        if call==3 {assert!(contains(map,A));hold(&fresh);}
        result
    });
    let quick:SurfacePathComputer=Arc::new(|_,_,_,_| Err(module_error("completion-fence fixture","quick intentionally unavailable")));
    let (mut harness,cells)=start(computer,quick,false,true).await;
    let (goal,initial)=active(&mut harness).await;
    cells.lock().unwrap().insert(RELATED);
    wait(||held.load(Ordering::Acquire)).await;
    let planned=latest(&harness).unwrap();
    cells.lock().unwrap().insert(A);
    wait(||latest(&harness).is_some_and(|map|contains(&map,A))).await;
    evidence_updates(&harness,3).await;
    let latest_map=latest(&harness).unwrap();
    assert!(!nuwax_nav_pct::same_source_geometry(&planned,&latest_map));
    release_old.store(true,Ordering::Release);
    if EXPECT_COMPLETION_REUSE {
        let replacement=harness.path_for(&goal).await;
        assert_eq!(replacement.path_revision,initial.path_revision+1);
        assert_eq!(calls.load(Ordering::SeqCst),2,"disjoint update discarded held normal result");
        executed_revision(&harness,&goal,replacement.path_revision).await;
        eprintln!("COMPLETION_REUSE normal=2 unrelated=A metadata_after=3 held_normal_published=true");
    } else {
        wait(||calls.load(Ordering::SeqCst)==3).await;
        unchanged_path(&mut harness,initial.path_revision).await;
        release_fresh.store(true,Ordering::Release);
        let replacement=harness.path_for(&goal).await;
        assert_eq!(replacement.path_revision,initial.path_revision+1);
        executed_revision(&harness,&goal,replacement.path_revision).await;
        eprintln!("COMPLETION_BASELINE normal=3 unrelated=A metadata_after=3 held_normal_published=false fresh_normal_published=true");
    }
    release_fresh.store(true,Ordering::Release);
    harness.cancel(&goal).await;harness.terminal(TerminalOutcome::Cancelled).await;harness.shutdown().await;
}

async fn completion_normal_chain(related: bool, cancel: bool, unknown_mask: bool) {
    let calls=Arc::new(AtomicUsize::new(0));
    let held=Arc::new(AtomicBool::new(false));
    let release=Arc::new(AtomicBool::new(false));
    let fresh_release=Arc::new(AtomicBool::new(false));
    let retired=Arc::new(AtomicBool::new(false));
    let (n,h,r,f,c)=(calls.clone(),held.clone(),release.clone(),fresh_release.clone(),retired.clone());
    let computer:SurfacePathComputer=Arc::new(move |map,pose,goal,configured| {
        let call=n.fetch_add(1,Ordering::SeqCst)+1;
        let result=body_path(map,pose,goal,configured);
        assert!(result.is_ok());
        if call==2 {h.store(true,Ordering::Release);hold(&r);
            c.store(configured.planning_cancelled.as_ref().unwrap().load(Ordering::Acquire),Ordering::Release);}
        if call==3 {hold(&f);}
        result
    });
    let quick:SurfacePathComputer=Arc::new(|_,_,_,_|Err(module_error("completion-fence fixture","quick intentionally unavailable")));
    let (mut harness,cells)=start(computer,quick,false,true).await;
    let (goal,initial)=active(&mut harness).await;
    cells.lock().unwrap().insert(RELATED);wait(||held.load(Ordering::Acquire)).await;
    if related {cells.lock().unwrap().insert(B);wait(||latest(&harness).is_some_and(|s|contains(&s,B))).await;}
    if unknown_mask {
        { let mut feed=harness.feed.lock().unwrap();feed.map.virtual_obstacles=Some(virtual_mask(&feed.map,false)); }
        wait(||harness.hub.latest.lock().unwrap().as_ref().is_some_and(|map|
            matches!(map.geometry_changes.as_ref(),SurfaceGeometryChanges::Unknown))).await;
    }
    for cell in [A,[61,11,47],[51,11,47]] {
        cells.lock().unwrap().insert(cell);wait(||latest(&harness).is_some_and(|s|contains(&s,cell))).await;
        evidence_updates(&harness,2).await;
    }
    cells.lock().unwrap().remove(&A);wait(||latest(&harness).is_some_and(|s|!contains(&s,A))).await;
    cells.lock().unwrap().insert(A);wait(||latest(&harness).is_some_and(|s|contains(&s,A))).await;
    evidence_updates(&harness,3).await;
    if cancel {harness.cancel(&goal).await;harness.terminal(TerminalOutcome::Cancelled).await;}
    release.store(true,Ordering::Release);
    if cancel {
        wait(||retired.load(Ordering::Acquire)).await;
        assert!(tokio::time::timeout(Duration::from_millis(100),harness.paths.next()).await.is_err());
        fresh_release.store(true,Ordering::Release);
        let next=goal_for(&harness.feed.lock().unwrap().map,2,[3.2,1.55,2.5]);
        harness.goal(&next).await;let path=harness.path_for(&next).await;
        assert_eq!(path.goal_key(),next.goal_key());
        executed_revision(&harness,&next,path.path_revision).await;
        harness.cancel(&next).await;harness.terminal(TerminalOutcome::Cancelled).await;
    } else {
        if related || unknown_mask {wait(||calls.load(Ordering::SeqCst)==3).await;unchanged_path(&mut harness,initial.path_revision).await;fresh_release.store(true,Ordering::Release);}
        let path=harness.path_for(&goal).await;
        assert_eq!(path.path_revision,initial.path_revision+1);
        assert_eq!(calls.load(Ordering::SeqCst),if related||unknown_mask {3}else{2});
        executed_revision(&harness,&goal,path.path_revision).await;
        harness.cancel(&goal).await;harness.terminal(TerminalOutcome::Cancelled).await;
    }
    fresh_release.store(true,Ordering::Release);
    eprintln!("COMPLETION_NORMAL_CHAIN related={related} cancel={cancel} unknown_mask={unknown_mask} normal={} geometry_events=5 metadata_events_at_least=9",calls.load(Ordering::SeqCst));
    harness.shutdown().await;
}
#[tokio::test] async fn completion_fence_multiple_disjoint_normal() {completion_normal_chain(false,false,false).await;}
#[tokio::test] async fn completion_fence_earlier_related_normal_is_not_forgotten() {completion_normal_chain(true,false,false).await;}
#[tokio::test] async fn completion_fence_disjoint_then_cancel_and_new_goal() {completion_normal_chain(false,true,false).await;}
#[tokio::test] async fn completion_fence_unknown_mask_then_disjoint_normal() {completion_normal_chain(false,false,true).await;}

async fn completion_quick_chain(related: bool, normal_wins: bool, unknown_mask: bool) {
    let normal=Arc::new(AtomicUsize::new(0));let quick=Arc::new(AtomicUsize::new(0));
    let normal_held=Arc::new(AtomicBool::new(false));let quick_held=Arc::new(AtomicBool::new(false));
    let nr=Arc::new(AtomicBool::new(false));let qr=Arc::new(AtomicBool::new(false));let fresh=Arc::new(AtomicBool::new(false));
    let normal_cancelled=Arc::new(AtomicBool::new(false));
    let (n,h,r,c)=(normal.clone(),normal_held.clone(),nr.clone(),normal_cancelled.clone());
    let computer:SurfacePathComputer=Arc::new(move |map,pose,goal,configured| {
        let call=n.fetch_add(1,Ordering::SeqCst)+1;let result=body_path(map,pose,goal,configured);assert!(result.is_ok());
        if call==2 {h.store(true,Ordering::Release);hold(&r);c.store(configured.planning_cancelled.as_ref().unwrap().load(Ordering::Acquire),Ordering::Release);}result
    });
    let (q,h,r,f)=(quick.clone(),quick_held.clone(),qr.clone(),fresh.clone());
    let quick_computer:SurfacePathComputer=Arc::new(move |map,pose,goal,configured| {
        let call=q.fetch_add(1,Ordering::SeqCst)+1;let result=body_path_with_budget(map,pose,goal,configured,0);assert!(result.is_ok());
        if call==1 {h.store(true,Ordering::Release);hold(&r);}else{hold(&f);}result
    });
    let (mut harness,cells)=start(computer,quick_computer,false,true).await;
    let (goal,initial)=active(&mut harness).await;
    cells.lock().unwrap().insert(RELATED);wait(||normal_held.load(Ordering::Acquire)).await;
    // In normal-wins arm A triggers quick without making normal's proof related.
    let trigger=if normal_wins {A}else{B};
    cells.lock().unwrap().insert(trigger);wait(||quick_held.load(Ordering::Acquire)).await;
    if related {cells.lock().unwrap().insert(C);wait(||latest(&harness).is_some_and(|s|contains(&s,C))).await;}
    if unknown_mask {
        { let mut feed=harness.feed.lock().unwrap();feed.map.virtual_obstacles=Some(virtual_mask(&feed.map,false)); }
        wait(||harness.hub.latest.lock().unwrap().as_ref().is_some_and(|map|
            matches!(map.geometry_changes.as_ref(),SurfaceGeometryChanges::Unknown))).await;
    }
    cells.lock().unwrap().insert([61,11,47]);wait(||latest(&harness).is_some_and(|s|contains(&s,[61,11,47]))).await;
    evidence_updates(&harness,3).await;
    if normal_wins {nr.store(true,Ordering::Release);}else{qr.store(true,Ordering::Release);}
    if related || unknown_mask {wait(||quick.load(Ordering::SeqCst)>=2).await;unchanged_path(&mut harness,initial.path_revision).await;fresh.store(true,Ordering::Release);}
    let path=harness.path_for(&goal).await;assert_eq!(path.path_revision,initial.path_revision+1);
    executed_revision(&harness,&goal,path.path_revision).await;
    nr.store(true,Ordering::Release);qr.store(true,Ordering::Release);fresh.store(true,Ordering::Release);
    if !normal_wins {wait(||normal_cancelled.load(Ordering::Acquire)).await;}
    unchanged_path(&mut harness,path.path_revision).await;
    assert_eq!(normal.load(Ordering::SeqCst),2);
    assert_eq!(quick.load(Ordering::SeqCst),if related || unknown_mask {2}else{1});
    eprintln!("COMPLETION_QUICK_CHAIN related={related} normal_wins={normal_wins} unknown_mask={unknown_mask} normal=2 quick={} single_replacement=true",quick.load(Ordering::SeqCst));
    harness.cancel(&goal).await;harness.terminal(TerminalOutcome::Cancelled).await;harness.shutdown().await;
}
#[tokio::test] async fn completion_fence_disjoint_quick_wins_and_retires_normal() {completion_quick_chain(false,false,false).await;}
#[tokio::test] async fn completion_fence_earlier_related_quick_is_not_forgotten() {completion_quick_chain(true,false,false).await;}
#[tokio::test] async fn completion_fence_disjoint_normal_wins_and_fences_quick() {completion_quick_chain(false,true,false).await;}

fn completion_fixture() -> (SurfaceNavigationConfig, SurfaceGeometryCache, FineFixture, PreparedSurfaceMap, NavStackResult<Path3D>) {
    let configured=configured(false,true);let mut cache=SurfaceGeometryCache::configured(&configured);let mut fixture=FineFixture::new();
    let message=fixture.next();let goal=goal_for(&message,1,[3.2,1.55,2.5]);
    let mut first=pose_from_schema(&goal.goal).unwrap();first.pose.position.x=0.55;
    let path=Ok(Path3D {poses:vec![first,pose_from_schema(&goal.goal).unwrap()]});
    let update=prepare_surface_map(message,Instant::now(),&configured,&mut cache);
    assert!(update.snapshot.is_ok());(configured,cache,fixture,update,path)
}
fn completion_ticket(configured:&SurfaceNavigationConfig,update:&PreparedSurfaceMap)->SurfaceJobTicket {
    SurfaceJobTicket::new(7,GoalKey {producer_runtime_generation:8,goal_generation:1},Arc::new(AtomicBool::new(false)),update.snapshot.as_ref().unwrap(),configured)
}
#[test]
fn completion_fence_proof_preserves_multiframe_basis_and_metadata_cost() {
    let (configured,mut cache,mut fixture,first,path)=completion_fixture();
    let mut ticket=completion_ticket(&configured,&first);
    let mut latest=first;
    for cell in [A,[61,11,47],[51,11,47]] {
        fixture.cells.insert(cell);latest=prepare_surface_map(fixture.next(),Instant::now(),&configured,&mut cache);
        ticket.observe(&latest);assert!(!ticket.unknown);
    }
    assert_eq!(ticket.changes.len(),3);let columns=ticket.columns;
    for _ in 0..20 {
        latest=prepare_surface_map(fixture.next(),Instant::now(),&configured,&mut cache);
        assert!(matches!(latest.geometry_changes.as_ref(),SurfaceGeometryChanges::Empty));
        ticket.observe(&latest);
    }
    assert_eq!(ticket.changes.len(),3);assert_eq!(ticket.columns,columns);
    assert!(ticket.permits_changed_result(latest.snapshot.as_ref().unwrap(),&path));
    assert!(!ticket.permits_changed_result(latest.snapshot.as_ref().unwrap(),&Err(module_error("test","old source failure"))));
    fixture.cells.insert(B);latest=prepare_surface_map(fixture.next(),Instant::now(),&configured,&mut cache);ticket.observe(&latest);
    assert!(!ticket.permits_changed_result(latest.snapshot.as_ref().unwrap(),&path));
    for _ in 0..3 {latest=prepare_surface_map(fixture.next(),Instant::now(),&configured,&mut cache);ticket.observe(&latest);}
    assert!(!ticket.permits_changed_result(latest.snapshot.as_ref().unwrap(),&path));
    eprintln!("COMPLETION_PROOF disjoint_batches=3 metadata_updates=20 retained_columns={columns} related_then_empty_rejected=true stale_error_rejected=true");
}
#[test]
fn completion_fence_proof_broken_basis_unknown_and_failure_are_absorbing() {
    let (configured,mut cache,mut fixture,first,path)=completion_fixture();
    let mut broken=completion_ticket(&configured,&first);
    fixture.cells.insert(A);let skipped=prepare_surface_map(fixture.next(),Instant::now(),&configured,&mut cache);
    fixture.cells.insert([61,11,47]);let second=prepare_surface_map(fixture.next(),Instant::now(),&configured,&mut cache);
    broken.observe(&second);assert!(broken.unknown);broken.observe(&skipped);
    assert!(!broken.permits_changed_result(skipped.snapshot.as_ref().unwrap(),&path));
    let mut unknown=completion_ticket(&configured,&first);let mut injected=skipped.clone();
    injected.geometry_changes=Arc::new(SurfaceGeometryChanges::Unknown);unknown.observe(&injected);unknown.observe(&second);
    assert!(unknown.unknown);assert!(!unknown.permits_changed_result(second.snapshot.as_ref().unwrap(),&path));
    let mut failed=completion_ticket(&configured,&first);injected.snapshot=Err("test build failure".into());failed.observe(&injected);failed.observe(&skipped);
    assert!(failed.unknown);assert!(!failed.permits_changed_result(skipped.snapshot.as_ref().unwrap(),&path));
    eprintln!("COMPLETION_PROOF broken_basis=true unknown_absorbs=true failed_build_absorbs=true");
}
#[test]
fn completion_fence_proof_budget_is_total_and_repeated_columns_are_conservative() {
    let (configured,_cache,_fixture,first,path)=completion_fixture();
    let surface=first.snapshot.as_ref().unwrap();let vertices=surface.body_envelope().unwrap().footprint_xy_m.len();
    let poses=path.as_ref().unwrap().poses.len();let allowed=MAX_REFERENCE_SCOPE_WORK-vertices-poses;
    let mut update=first.clone();update.change_basis=Some(Arc::clone(surface));
    update.geometry_changes=Arc::new(SurfaceGeometryChanges::Columns {fine_size:0.05,fine:vec![[A[0],A[1]];allowed],planner_size:0.1,coarse:vec![]});
    let mut exact=completion_ticket(&configured,&first);exact.observe(&update);
    assert_eq!(exact.columns,allowed);assert!(exact.permits_changed_result(surface,&path));
    update.geometry_changes=Arc::new(SurfaceGeometryChanges::Columns {fine_size:0.05,fine:vec![[A[0],A[1]]],planner_size:0.1,coarse:vec![]});
    exact.observe(&update);assert_eq!(exact.columns,allowed+1);assert!(!exact.permits_changed_result(surface,&path));
    update.geometry_changes=Arc::new(SurfaceGeometryChanges::Columns {fine_size:0.05,fine:vec![[A[0],A[1]];MAX_REFERENCE_SCOPE_WORK],planner_size:0.1,coarse:vec![]});
    exact.observe(&update);assert!(exact.unknown);
    eprintln!("COMPLETION_PROOF exact_total=4096 pass=true excess_total=4097 rejected=true repeated_columns_count=true overflow_unknown=true");
}
#[test]
fn completion_fence_proof_disabled_outside_off_full_body() {
    let (mut on_config,_cache,_fixture,first,path)=completion_fixture();on_config.follower.local_planning_enabled=true;
    let mut on=completion_ticket(&on_config,&first);on.observe(&first);
    assert!(on.basis.is_none());assert_eq!(on.columns,0);assert!(on.changes.is_empty());
    assert!(!on.permits_changed_result(first.snapshot.as_ref().unwrap(),&path));
    let legacy=configured(false,false);let mut cache=SurfaceGeometryCache::configured(&legacy);
    let legacy_update=prepare_surface_map(map_message(1,"legacy"),Instant::now(),&legacy,&mut cache);
    let mut off_legacy=completion_ticket(&legacy,&legacy_update);off_legacy.observe(&legacy_update);
    assert!(off_legacy.basis.is_none());assert_eq!(off_legacy.columns,0);assert!(off_legacy.changes.is_empty());
    eprintln!("COMPLETION_PROOF on_and_legacy_no_basis_no_delta_retention=true");
}

async fn completion_epoch_boundary(clear: bool) {
    let calls=Arc::new(AtomicUsize::new(0));let held=Arc::new(AtomicBool::new(false));let release=Arc::new(AtomicBool::new(false));let retired=Arc::new(AtomicBool::new(false));
    let (n,h,r,c)=(calls.clone(),held.clone(),release.clone(),retired.clone());
    let computer:SurfacePathComputer=Arc::new(move |map,pose,goal,configured| {
        let call=n.fetch_add(1,Ordering::SeqCst)+1;let result=body_path(map,pose,goal,configured);assert!(result.is_ok());
        if call==2 {h.store(true,Ordering::Release);hold(&r);c.store(configured.planning_cancelled.as_ref().unwrap().load(Ordering::Acquire),Ordering::Release);}result
    });
    let quick:SurfacePathComputer=Arc::new(|_,_,_,_|Err(module_error("completion-fence fixture","quick intentionally unavailable")));
    let (mut harness,cells)=start(computer,quick,false,true).await;
    let (goal,_initial)=active(&mut harness).await;
    cells.lock().unwrap().insert(RELATED);wait(||held.load(Ordering::Acquire)).await;
    cells.lock().unwrap().insert(A);wait(||latest(&harness).is_some_and(|s|contains(&s,A))).await;evidence_updates(&harness,2).await;
    let mut update=harness.freeze_map().await;
    if clear {
        update.cloud_revision+=1;update.committed_map.revision+=1;update.committed_map.commit_sequence+=1;
        update.commit_sequence=update.committed_map.commit_sequence;update.cloud_occupancy_revision=update.committed_map.revision;
        update.cleared=true;update.virtual_obstacles=None;update.fine=None;
        let stamp=current_unix_timestamp_ns().unwrap();update.source_timestamp_ns=stamp;update.committed_map.source_timestamp_ns=stamp;
        update.cloud=schema::PointCloud2 {header:schema::Header {seq:update.cloud_revision as i32,stamp_sec:(stamp/1_000_000_000) as i32,
            stamp_nsec:(stamp%1_000_000_000) as i32,frame_id:"map".into()},height:1,..schema::PointCloud2::default()};
    } else {
        let mut next=map_message(update.cloud_revision+1,"completion-new-domain");
        next.producer_runtime_generation=update.producer_runtime_generation+1;
        next.committed_map.session_generation=update.committed_map.session_generation+1;
        next.map_frame_generation=update.map_frame_generation+1;
        update=next;
    }
    schema::validate_navigation_map_3d(&update).unwrap();harness.map(&update).await;
    wait(||harness.hub.latest.lock().unwrap().as_ref().is_some_and(|m|
        if clear {m.snapshot.is_err()}else{m.authority.committed_map.session_id=="completion-new-domain"})).await;
    // The owner marks the running ticket cancelled on its next iteration.
    tokio::time::sleep(Duration::from_millis(30)).await;release.store(true,Ordering::Release);
    wait(||retired.load(Ordering::Acquire)).await;
    assert!(tokio::time::timeout(Duration::from_millis(100),harness.paths.next()).await.is_err());
    harness.goal(&goal).await;
    assert!(tokio::time::timeout(Duration::from_millis(100),harness.paths.next()).await.is_err());
    assert_eq!(calls.load(Ordering::SeqCst),2);
    eprintln!("COMPLETION_EPOCH clear={clear} held_success_cancelled=true old_goal_replay_did_not_publish=true");
    harness.shutdown().await;
}
#[tokio::test] async fn completion_fence_disjoint_then_clear_cannot_revive_goal() {completion_epoch_boundary(true).await;}
#[tokio::test] async fn completion_fence_disjoint_then_domain_switch_cannot_revive_goal() {completion_epoch_boundary(false).await;}

#[tokio::test] async fn completion_fence_unknown_mask_then_disjoint_quick() {completion_quick_chain(false,false,true).await;}

#[tokio::test]
async fn quick_zero_budget_is_inconclusive_and_preserves_route_and_full_job() {
    const DETOUR_DISJOINT: [i32;3] = [110,5,47];
    let normal = Arc::new(AtomicUsize::new(0));
    let quick = Arc::new(AtomicUsize::new(0));
    let release = Arc::new(AtomicBool::new(false));
    let full_flag: Arc<Mutex<Option<Arc<AtomicBool>>>> = Arc::new(Mutex::new(None));
    let (n, r, flag) = (normal.clone(), release.clone(), full_flag.clone());
    let computer: SurfacePathComputer = Arc::new(move |map, pose, goal, configured| {
        let call = n.fetch_add(1, Ordering::SeqCst) + 1;
        let result = body_path(map, pose, goal, configured);
        assert!(result.is_ok(), "positive-budget fixture must find the real detour");
        if call == 2 {
            let corridor = reference_scope::ReferenceCorridor::new(map, result.as_ref().unwrap()).unwrap();
            assert!(!corridor.intersects_columns(0.05, &[[DETOUR_DISJOINT[0],DETOUR_DISJOINT[1]]])
                && !corridor.intersects_columns(0.1, &[[55,2]]),
                "quick-probe refresh must be disjoint from the actual detour, not merely a straight-line fixture");
            *flag.lock().unwrap() = configured.planning_cancelled.clone();
            hold(&r);
        }
        result
    });
    let q = quick.clone();
    let quick_computer: SurfacePathComputer = Arc::new(move |map, pose, goal, configured| {
        let result = body_path_with_budget(map, pose, goal, configured, 0);
        assert!(result.as_ref().err().is_some_and(|error|
            error.to_string().contains("BudgetExhausted")), "quick fixture must require a detour");
        q.fetch_add(1, Ordering::SeqCst);
        result
    });
    let (mut harness, cells) = start(computer, quick_computer, false, true).await;
    // Existing static obstacle forces an actual detour before either refresh.
    // The later disjoint change then requests a quick probe of that same detour.
    let obstacle = [40, 30, 51];
    cells.lock().unwrap().insert(obstacle);
    wait(|| latest(&harness).is_some_and(|surface| contains(&surface, obstacle))).await;
    let (goal, initial) = active(&mut harness).await;
    cells.lock().unwrap().insert(RELATED);
    wait(|| full_flag.lock().unwrap().is_some()).await;
    cells.lock().unwrap().insert(DETOUR_DISJOINT);
    wait(|| quick.load(Ordering::SeqCst) > 0).await;
    harness.freeze_map().await;
    unchanged_path(&mut harness, initial.path_revision).await;
    assert!(!full_flag.lock().unwrap().as_ref().unwrap().load(Ordering::Acquire),
        "an inconclusive quick probe cannot cancel the full job");
    assert_eq!(normal.load(Ordering::SeqCst), 2);
    release.store(true, Ordering::Release);
    let replacement = harness.path_for(&goal).await;
    executed_revision(&harness, &goal, replacement.path_revision).await;
    assert_eq!(replacement.path_revision, initial.path_revision + 1);
    assert_eq!(normal.load(Ordering::SeqCst), 2, "the valid held full result must survive");
    harness.cancel(&goal).await;
    harness.terminal(TerminalOutcome::Cancelled).await;
    harness.shutdown().await;
}

fn measured_flat_layer(cell: [i32; 3]) -> nuwax_map_chunks::LayerObservation {
    let point = cell.map(|v| (f64::from(v) + 0.5) * f64::from(0.05_f32));
    nuwax_map_chunks::LayerObservation {
        members: vec![nuwax_map_chunks::local_key(cell)], plane_origin: point,
        normal: [0.0, 0.0, 1.0], samples: vec![nuwax_map_chunks::LayerSample {source: Default::default(),
            point, source_sequence: 1, source_timestamp_ns: 1, point_index: 0,
        }],
    }
}

#[tokio::test]
async fn selective_refresh_added_ground_and_classification_keep_route_and_progress() {
    compatible_ground_owner(0.0).await;
}
#[tokio::test]
async fn selective_refresh_pitched_body_keeps_route_and_progress_on_ground_evidence() {
    compatible_ground_owner(4.86_f64.to_radians()).await;
}
async fn compatible_ground_owner(pitch: f64) {
    let normal = Arc::new(AtomicUsize::new(0));
    let quick = Arc::new(AtomicUsize::new(0));
    let layers: Layers = Arc::default();
    let (mut harness, cells) = start_with_layers(counted(normal.clone(), false),
        counted(quick.clone(), true), false, true, layers.clone()).await;
    // The active reference crosses genuinely missing ground, while its endpoint
    // remains on the same existing plane. This is not a timestamp-only update.
    cells.lock().unwrap().retain(|c| !(25..=50).contains(&c[0]) || !(27..=35).contains(&c[1]));
    wait(|| latest(&harness).is_some_and(|map| !contains(&map, [40,30,40]))).await;
    harness.feed.lock().unwrap().pitch = pitch;
    tokio::time::sleep(Duration::from_millis(60)).await;
    let (goal, initial) = active(&mut harness).await;
    if pitch != 0.0 { assert!(initial.path.poses[0].pose.orientation_xyzw[1].abs() > 0.04); }
    harness.feed.lock().unwrap().integrate = true;
    let start_x = harness.feed.lock().unwrap().position[0];
    wait(|| harness.feed.lock().unwrap().position[0] > start_x + 0.03).await;
    let before_commands = harness.scheduler.status().applied_commands;
    cells.lock().unwrap().insert([40,30,40]);
    wait(|| latest(&harness).is_some_and(|map| contains(&map, [40,30,40]))).await;
    evidence_updates(&harness, 2).await;
    unchanged_path(&mut harness, initial.path_revision).await;
    assert_eq!((normal.load(Ordering::SeqCst), quick.load(Ordering::SeqCst)), (1,0));
    let after_ground_x = harness.feed.lock().unwrap().position[0];
    assert!(after_ground_x > start_x + 0.03);
    assert!(harness.scheduler.status().applied_commands > before_commands);

    // Existing physical floor occupancy now acquires classified ground evidence.
    // The source geometry identity changes, although its occupied cells do not.
    let column = [20,30,40];
    let before = latest(&harness).unwrap();
    layers.lock().unwrap().entry(nuwax_map_chunks::block_key(column)).or_default()
        .insert(nuwax_map_chunks::local_key(column), measured_flat_layer(column));
    wait(|| latest(&harness).is_some_and(|map| map.observations().unwrap()
        .ground_layers([20,30]).next().is_some())).await;
    let classified = latest(&harness).unwrap();
    assert!(before.same_geometry(&classified));
    assert!(!nuwax_nav_pct::same_source_geometry(&before, &classified));
    wait(|| harness.feed.lock().unwrap().position[0] > after_ground_x + 0.03).await;
    evidence_updates(&harness, 2).await;
    unchanged_path(&mut harness, initial.path_revision).await;
    assert_eq!((normal.load(Ordering::SeqCst), quick.load(Ordering::SeqCst)), (1,0));

    // A previously known plane changing is not covered by the additions proof.
    layers.lock().unwrap().get_mut(&nuwax_map_chunks::block_key(column)).unwrap()
        .get_mut(&nuwax_map_chunks::local_key(column)).unwrap().plane_origin[2] += 0.01;
    let changed = harness.path_for(&goal).await;
    assert_eq!(changed.path_revision, initial.path_revision + 1);
    executed_revision(&harness, &goal, changed.path_revision).await;
    assert_eq!(normal.load(Ordering::SeqCst), 2);
    // Actual occupancy at body height on the remaining corridor still replans.
    // This is stronger than the older RELATED advisory-only below-body fixture.
    cells.lock().unwrap().insert([50,30,51]);
    let obstacle = harness.path_for(&goal).await;
    assert_eq!(obstacle.path_revision, changed.path_revision + 1);
    executed_revision(&harness, &goal, obstacle.path_revision).await;
    assert_eq!(normal.load(Ordering::SeqCst), 3);
    harness.cancel(&goal).await;
    harness.terminal(TerminalOutcome::Cancelled).await;
    harness.shutdown().await;
}

#[tokio::test]
async fn selective_refresh_committed_empty_fine_source_reaches_unknown_planner_and_execution() {
    let normal = Arc::new(AtomicUsize::new(0));
    let quick = Arc::new(AtomicUsize::new(0));
    let (mut harness,cells) = start(counted(normal.clone(),false),counted(quick,true),false,true).await;
    let original = harness.hub.latest.lock().unwrap().as_ref().unwrap().authority.clone();
    cells.lock().unwrap().clear();
    wait(|| latest(&harness).is_some_and(|map| {
        let (_, mut columns) = map.navigation_source_columns(None);
        columns.next().is_none()
    })).await;
    let map = harness.hub.latest.lock().unwrap().as_ref().unwrap().clone();
    assert!(map.authority.same_domain(&original));
    assert!(!map.awaiting_build && map.snapshot.is_ok());
    let (goal,path) = active(&mut harness).await;
    assert_eq!(normal.load(Ordering::SeqCst),1);
    harness.feed.lock().unwrap().integrate = true;
    let before = harness.feed.lock().unwrap().position[0];
    wait(|| harness.feed.lock().unwrap().position[0] > before + 0.03).await;
    unchanged_path(&mut harness,path.path_revision).await;
    harness.cancel(&goal).await;
    harness.terminal(TerminalOutcome::Cancelled).await;
    harness.shutdown().await;
}
