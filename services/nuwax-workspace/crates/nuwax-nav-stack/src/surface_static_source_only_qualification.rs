// New opt-in source-isolation cases. Include beside dense_recent in the existing
// dynamic occupancy qualification module. The original geometry, goal, owner,
// plant, continuous oracle, exercise, cancellation and zero fence are reused.
// This is persistent-Fine-source-only obstacle insertion, not a claim that the
// derived coarse Arc stays identical. Recent geometry stays ground-only; source
// timestamps and sequences must continue to advance from real acquisitions.
use super::*;
use nuwax_codec::lcm::recent_cloud;

const RECENT_TTL: Duration = Duration::from_secs(1);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RecentQualification {
    // Preserve the original stronger diagnostic and its selector/history.
    StrictRawEmpty,
    // Original measurement bytes stay fixed; source-local interpretation may change.
    FixedRecentGeometry,
}
impl RecentQualification {
    fn label(self) -> &'static str {
        match self {
            Self::StrictRawEmpty => "strict_raw_empty",
            Self::FixedRecentGeometry => "fixed_recent_geometry",
        }
    }
}

#[derive(Debug)]
struct FirstNonemptyHints {
    body: BodyPose,
    raw_count: usize,
    surface_points: usize,
    occupied_volumes: usize,
    unique_surface_points: usize,
    surface_multiplicity_histogram: BTreeMap<usize, usize>,
    bounds: ([f64; 3], [f64; 3]),
    sample_surface_points: Vec<([f64; 3], usize)>,
}

#[derive(Debug, Default)]
struct RawHintReport {
    queries: usize,
    nonempty_queries: usize,
    sum_per_query_hints: usize,
    max_per_query_hints: usize,
    first_nonempty: Option<FirstNonemptyHints>,
}
impl RawHintReport {
    fn is_empty(&self) -> bool { self.nonempty_queries == 0 }
}

#[derive(Clone, Debug)]
struct ObservedRecent {
    received: Instant,
    received_unix_ns: i64,
    payload: Vec<u8>,
}

#[derive(Clone, Debug)]
struct ObservedHealth {
    at: Instant,
    source: i64,
    pose_source: i64,
    observed_unix_ns: i64,
    fresh: bool,
    session: Option<String>,
}

// The unchanged production-shaped publisher determines field layout and point
// ordering. Compare actual wire bytes/indices, not only a point count or hash.
fn verify_ground_packet(
    packet: &recent_cloud::RecentCloud3D,
    authority: &GuardedMapAuthority,
    expected_source: i64,
) -> Result<Vec<[f64; 3]>, String> {
    if packet.producer_runtime_generation != authority.producer_runtime_generation
        || packet.session_id != authority.committed_map.session_id
        || packet.session_generation != authority.committed_map.session_generation
        || packet.map_frame_generation != authority.map_frame_generation
        || packet.commit_boot_nonce != authority.commit_boot_nonce
    {
        return Err("recent wire navigation domain differs from persistent source".into());
    }
    let cloud = packet.cloud.as_ref().ok_or("recent source was revoked")?;
    if cloud.source_timestamp_ns != expected_source || cloud.source_sequence <= 0
        || cloud.runtime_generation != 17 || cloud.map_frame_id != "map"
        || cloud.correlated_odometry_source_sequence != cloud.source_sequence
        || cloud.correlated_odometry_source_timestamp_ns <= 0
        || cloud.correlated_odometry_source_timestamp_ns > cloud.source_timestamp_ns
    {
        return Err("recent source/pose identity is not the original live acquisition".into());
    }
    let expected_fields: Vec<_> = ["x", "y", "z"].into_iter().enumerate().map(|(i, name)|
        schema::PointField { name: name.into(), offset: (i * 4) as i32, datatype: 7, count: 1 }).collect();
    if cloud.cloud.height != 1 || cloud.cloud.width != 6000 || cloud.cloud.point_step != 12
        || cloud.cloud.row_step != 6000 * 12 || cloud.cloud.is_bigendian || !cloud.cloud.is_dense
        || cloud.cloud.fields != expected_fields || cloud.cloud.header.frame_id != "map"
        || i64::from(cloud.cloud.header.seq) != cloud.source_sequence
        || i64::from(cloud.cloud.header.stamp_sec) * 1_000_000_000
            + i64::from(cloud.cloud.header.stamp_nsec) != cloud.source_timestamp_ns
    { return Err("recent PointCloud2 layout/header changed from the original publisher".into()); }
    let mut expected_bytes = Vec::with_capacity(6000 * 12);
    for key in ground_keys() {
        for coordinate in key {
            expected_bytes.extend_from_slice(&((coordinate as f32 + 0.5) * RESOLUTION).to_le_bytes());
        }
    }
    if cloud.source_point_count != 6000 || cloud.emitted_point_count != 6000
        || cloud.cloud.data != expected_bytes
        || packet.ground_point_indices != (0..6000).collect::<Vec<u32>>()
    {
        return Err("recent geometry or ground classification changed from original clear scan".into());
    }
    let points = point_cloud_from_schema(cloud.cloud.clone()).map_err(|e| e.to_string())?;
    if points.points.len() != 6000 {
        return Err("materialized recent wire point count changed".into());
    }
    let points: Vec<_> = points.points.into_iter().map(|p| [p.x, p.y, p.z]).collect();
    if points.iter().zip(ground_keys()).any(|(point, key)|
        point.map(f64::to_bits) != key.map(|v| f64::from((v as f32 + 0.5) * RESOLUTION).to_bits()))
    { return Err("decoded recent geometry differs despite its raw byte checks".into()); }
    Ok(points)
}

fn attach_captured_ground(
    map: &SurfaceSnapshot,
    packet: &recent_cloud::RecentCloud3D,
    points: Vec<[f64; 3]>,
) -> Result<SurfaceSnapshot, String> {
    let cloud = packet.cloud.as_ref().ok_or("missing recent cloud")?;
    map.with_recent_obstacle_points(
        cloud.source_sequence as u64,
        cloud.source_timestamp_ns as u64,
        RECENT_TTL.as_nanos() as u64,
        &packet.ground_point_indices,
        points.into_iter().enumerate().map(|(i, p)| (i as u32, p)),
    ).map_err(|e| e.to_string())
}

// Independent stronger diagnostic, retained under the original ignored selectors.
// A ground label does not guarantee empty hints: current Fine interprets the point.
// Failure never authorizes removing points or changing the fixed measurement.
fn verify_no_recent_hints(map: &SurfaceSnapshot, bodies: &[BodyPose]) -> Result<usize, String> {
    let r = f64::from(RESOLUTION);
    let bounds = [[0.0, 0.0], [100.0 * r, 60.0 * r]];
    for body in bodies {
        let hints = map.recent_obstacle_hints(bounds, *body).map_err(|e| e.to_string())?;
        if !hints.is_empty() {
            return Err(format!("unchanged ground scan exports {} recent hints at {body:?}", hints.len()));
        }
    }
    Ok(bodies.len())
}

// FixedRecentGeometry executes every query and treats every query error as fatal.
// Its first nonempty result alone gets detail; no map/view history is retained.
// StrictRawEmpty keeps its original first-nonempty early failure.
fn inspect_recent_hints(
    map: &SurfaceSnapshot, bodies: &[BodyPose], qualification: RecentQualification,
) -> Result<RawHintReport, String> {
    if qualification == RecentQualification::StrictRawEmpty {
        return Ok(RawHintReport { queries: verify_no_recent_hints(map, bodies)?, ..Default::default() });
    }
    let r = f64::from(RESOLUTION);
    let bounds = [[0.0, 0.0], [100.0 * r, 60.0 * r]];
    let mut report = RawHintReport::default();
    for &body in bodies {
        let hints = map.recent_obstacle_hints(bounds, body)
            .map_err(|e| format!("raw_hint_query_failed raw_hints_empty=unknown: {e}"))?;
        report.queries += 1;
        report.sum_per_query_hints += hints.len();
        report.max_per_query_hints = report.max_per_query_hints.max(hints.len());
        if hints.is_empty() { continue; }
        report.nonempty_queries += 1;
        if report.first_nonempty.is_some() { continue; }
        let mut points = BTreeMap::<[u64; 3], usize>::new();
        let mut surface_points = 0usize;
        let mut low = [f64::INFINITY; 3];
        let mut high = [f64::NEG_INFINITY; 3];
        for &(a, b) in &hints {
            for axis in 0..3 { low[axis] = low[axis].min(a[axis]); high[axis] = high[axis].max(b[axis]); }
            if a == b {
                surface_points += 1;
                *points.entry(a.map(f64::to_bits)).or_default() += 1;
            }
        }
        let mut histogram = BTreeMap::new();
        for &count in points.values() { *histogram.entry(count).or_default() += 1; }
        report.first_nonempty = Some(FirstNonemptyHints {
            body, raw_count: hints.len(), surface_points,
            occupied_volumes: hints.len() - surface_points,
            unique_surface_points: points.len(), surface_multiplicity_histogram: histogram,
            bounds: (low, high),
            sample_surface_points: points.iter().take(3)
                .map(|(bits, &count)| (bits.map(f64::from_bits), count)).collect(),
        });
    }
    Ok(report)
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
#[ignore = "original strict raw-empty diagnostic; retain its NOT_QUALIFIED history separately"]
async fn static_source_only_wire_geometry_preflight() {
    static_source_wire_preflight(RecentQualification::StrictRawEmpty).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
#[ignore = "fixed recent geometry preflight; raw hints are reported independently, never erased"]
async fn fixed_recent_geometry_wire_preflight() {
    static_source_wire_preflight(RecentQualification::FixedRecentGeometry).await;
}

async fn static_source_wire_preflight(qualification: RecentQualification) {
    let transport = LcmTransport::new_real("memq://").unwrap();
    let mut wire = typed_stream(&transport, recent_cloud::DEFAULT_TOPIC, recent_cloud::TYPE_NAME).await.unwrap();
    let mut bodies = witness_poses();
    bodies.push(body_pose(3.5, 1.5, 0.0));
    let mut checked = 0usize;
    let mut raw_hints_empty = true;
    for scene in [Scene::Box, Scene::Wall] {
        let world = if qualification == RecentQualification::FixedRecentGeometry && scene == Scene::Wall {
            wall_world::World::ExplicitVirtualMaskDomain
        } else { wall_world::World::LegacyObservedSupport };
        let repository = Repository::temporary().unwrap();
        let mut writer = Writer::new(repository.clone(), RESOLUTION, 4 * 1024 * 1024).unwrap();
        let mut cache = SurfaceGeometryCache::configured(&calibrated_config(true));
        let mut prior_sample: Option<Instant> = None;
        let mut prior_geometry = None;
        let mut prior_live: Option<SurfaceSnapshot> = None;
        let mut prior_source = 0;
        for (index, stage) in [Stage::Clear, Stage::Inserted, Stage::Inserted].into_iter().enumerate() {
            if let Some(at) = prior_sample {
                let elapsed = at.elapsed();
                if elapsed < SAMPLE_PERIOD { tokio::time::sleep(SAMPLE_PERIOD - elapsed).await; }
            }
            prior_sample = Some(Instant::now());
            let stamp = current_unix_timestamp_ns().unwrap();
            assert!(stamp > prior_source, "NOT_QUALIFIED: source clock did not advance");
            prior_source = stamp;
            let retained = scene_keys(scene, stage);
            writer.observe(retained.iter().copied(), stamp as u64).unwrap();
            let root = writer.snapshot(Domain {
                session: format!("static-source-preflight-{scene:?}"), generation: 1,
                frame_generation: 1, producer_generation: 17,
            }, index as u64 + 1, stamp as u64).unwrap();
            if index == 1 { assert!(Some(root.geometry_revision) > prior_geometry); }
            if index == 2 { assert_eq!(Some(root.geometry_revision), prior_geometry); }
            prior_geometry = Some(root.geometry_revision);
            let mut map = map_from_fine(&root, &repository);
            world.declare_on_wire(&mut map);
            assert!(world.wire_matches(&map));
            let payload = schema::encode_navigation_map_3d_body(&map).unwrap();
            let decoded = schema::decode_navigation_map_3d_body(&payload).unwrap();
            assert_eq!(decoded, map);
            assert!(world.wire_matches(&decoded));
            assert!(decoded.fine.is_some() && !decoded.cleared && decoded.cloud.data.is_empty());
            assert_eq!((decoded.cloud.width, decoded.cloud.row_step), (0, 0));
            let persistent = cache.build(&decoded).unwrap();
            world.audit_built(&persistent).unwrap();
            audit_observations(&persistent, scene, stage, stamp).unwrap();
            original_geometry_queries(&persistent, scene, stage).unwrap();
            // Same original clear returns, newly sampled at every stage; never
            // send an empty packet, an old source timestamp, or a revocation.
            super::super::support_recovery_qualification::publish_sampled_recent(
                &transport, &decoded, index as i32 + 1, stamp, stamp,
                ground_keys().into_iter().map(|key| (key, true)),
            ).await;
            let frame = tokio::time::timeout(Duration::from_secs(2), wire.next()).await
                .expect("NOT_QUALIFIED: recent wire observation timed out")
                .expect("recent wire stream ended").unwrap();
            let packet = recent_cloud::decode_body(&frame.payload).unwrap();
            let points = verify_ground_packet(&packet, &map_authority(&decoded), stamp)
                .unwrap_or_else(|e| panic!("NOT_QUALIFIED: {e}"));
            let basis = prior_live.as_ref().map_or_else(|| persistent.as_ref().clone(), |previous|
                persistent.with_recent_obstacles_from(previous));
            let live = attach_captured_ground(&basis, &packet, points).unwrap();
            world.audit_built(&live).unwrap();
            // Wall poses on the other side are queries, not injected motion or a
            // wall path witness; the unchanged oracle supplies the separator.
            let hints = inspect_recent_hints(&live, &bodies, qualification)
                .unwrap_or_else(|e| panic!("NOT_QUALIFIED: {scene:?}/{stage:?}: {e}"));
            checked += hints.queries;
            raw_hints_empty &= hints.is_empty();
            original_geometry_queries(&live, scene, stage).unwrap();
            eprintln!("STATIC_SOURCE_PREFLIGHT qualification={} world={world:?} scene={scene:?} stage={stage:?} source={stamp} geometry_revision={} fine_cells={} recent_points=6000 fixed_recent_geometry=true recent_geometry=original_clear raw_hints_empty={} raw_hint_scope=offline_preflight_snapshot raw_hint_report={hints:?} recent_guide_interference=unknown exact_plan_kernel_query=not_observed body_queries=original",
                qualification.label(), root.geometry_revision, retained.len(), hints.is_empty());
            prior_live = Some(live);
        }
    }
    stable_plant_preserves_recorded_near_zero_counterexample_and_normal_arcs();
    continuous_oracle_tests::preflight_regressions();
    independent_geometry_preflight().unwrap();
    eprintln!("STATIC_SOURCE_PREFLIGHT_PASS qualification={} exact_wire=true source_time_advances=true fixed_recent_geometry=true raw_hints_empty={raw_hints_empty} raw_hint_scope=offline_preflight_snapshots recent_guide_interference=unknown offline_hint_queries={checked} live_cases_run=0", qualification.label());
}

fn audit_static_source_isolation(
    harness: &OccupancyHarness,
    scans: &[ObservedRecent],
    health: &[ObservedHealth],
    outcome_at: Instant,
    qualification: RecentQualification,
) -> Result<RawHintReport, String> {
    let (inserted_at, insertion_body, cursor) = harness.insertion
        .ok_or("actual moving insertion / real intersecting reference was not qualified")?;
    let goal = harness.goal.as_ref().ok_or("original goal missing")?;
    let events = harness.plant.lock().unwrap().events.clone();
    let evidence = harness.evidence.lock().unwrap();
    let initial = events.iter().filter(|e| e.seq <= cursor && is_nonzero_applied(e))
        .last().ok_or("initial real accepted movement missing")?;
    let old_path = evidence.paths.iter().find(|p| p.path_revision as u64 == initial.revision)
        .ok_or("accepted old reference is absent from real wire")?;
    validate_path_goal_correlation(old_path, goal)?;
    if !future_path_hits_box(old_path, insertion_body) {
        return Err("actual old reference does not cross the fixed new obstacle".into());
    }
    if scans.is_empty() { return Err("no observed recent wire packets".into()); }
    let mut previous: Option<(i64, i64, i64)> = None;
    let mut last_clear = None;
    let mut covered_until: Option<Instant> = None;
    let mut inserted_packets = 0;
    let mut representative = None;
    // Cleanup is audited by the unchanged harness. It cannot supply or revoke
    // source-isolation qualification for the already-observed outcome window.
    for row in scans.iter().filter(|row| row.received <= outcome_at) {
        let packet = recent_cloud::decode_body(&row.payload).map_err(|e| e.to_string())?;
        let cloud = packet.cloud.as_ref().ok_or("recent source revoked")?;
        let source = evidence.maps.get(&cloud.source_timestamp_ns)
            .ok_or("observed recent packet lacks the publisher's persistent source ledger")?;
        let points = verify_ground_packet(&packet, &source.authority, source.source)?;
        eprintln!("STATIC_SOURCE_RECENT source={} persistent_stage={:?} runtime_generation={} sequence={} pose_source={} received={:?} received_unix_ns={} points=6000 ground=6000 exact_original_clear_bytes=true",
            source.source, source.stage, cloud.runtime_generation, cloud.source_sequence,
            cloud.correlated_odometry_source_timestamp_ns, row.received, row.received_unix_ns);
        let source_age_ns = row.received_unix_ns - cloud.source_timestamp_ns;
        if source_age_ns < 0 || source_age_ns >= RECENT_TTL.as_nanos() as i64
        { return Err("observed recent source was future-dated or expired under its unchanged TTL".into()); }
        // Use absolute source expiry, not receipt time + a renewed full TTL.
        // This is wire-observation coverage, not a per-frame scheduler ACK.
        let expires = row.received + (RECENT_TTL - Duration::from_nanos(source_age_ns as u64));
        if row.received <= inserted_at && expires > inserted_at {
            covered_until = Some(covered_until.map_or(expires, |old| old.max(expires)));
        } else if row.received > inserted_at && row.received <= outcome_at {
            if covered_until.is_none_or(|old| row.received >= old) {
                return Err("observed recent wire has an absolute-source-expiry gap in the outcome window".into());
            }
            covered_until = Some(covered_until.unwrap().max(expires));
        }
        if let Some((generation, sequence, stamp)) = previous {
            if cloud.runtime_generation != generation || cloud.source_sequence <= sequence
                || cloud.source_timestamp_ns <= stamp
                || cloud.source_timestamp_ns - stamp > RECENT_TTL.as_nanos() as i64
            { return Err("recent source sequence/time health or generation continuity failed".into()); }
        }
        previous = Some((cloud.runtime_generation, cloud.source_sequence, cloud.source_timestamp_ns));
        if source.stage == Stage::Clear && row.received <= inserted_at { last_clear = Some(row.received); }
        if row.received <= outcome_at {
            if source.stage == Stage::Inserted { inserted_packets += 1; }
        }
        // Capture a real built AND scheduler-consumed inserted persistent source.
        // Every observed frame has the same ground bytes and classification.
        // This one-frame reconstruction samples source-local hint interpretation;
        // it is NOT the live planner's retained-frame set or actual QueryView.
        if source.stage == Stage::Inserted && representative.is_none()
            && row.received >= inserted_at && row.received <= outcome_at
            && (evidence.consumed.iter().any(|(at, s)|
                *at >= inserted_at && *at <= outcome_at && *s == source.source)
                || events.iter().any(|e| e.at >= inserted_at && e.at <= outcome_at && e.source == source.source))
        {
            if let Some(built) = evidence.built.get(&source.source) {
                let map = built.snapshot.as_ref().map_err(|e| e.to_string())?;
                audit_observations(map, harness.scene, Stage::Inserted, source.source)?;
                representative = Some((source.source, attach_captured_ground(map, &packet, points)?));
            }
        }
    }
    if !last_clear.is_some_and(|at| inserted_at.duration_since(at) <= RECENT_TTL)
        || !covered_until.is_some_and(|expiry| expiry > outcome_at)
        || inserted_packets < 2
    { return Err("recent stream did not cover both sides of insertion through the observed outcome".into()); }
    let active_health: Vec<_> = health.iter().filter(|row|
        row.at >= inserted_at && row.at <= outcome_at).collect();
    if active_health.len() < 2
        || active_health.iter().any(|row| !row.fresh || row.pose_source <= 0
            || row.observed_unix_ns < row.pose_source
            || row.observed_unix_ns - row.pose_source >= RECENT_TTL.as_nanos() as i64)
        || !active_health.iter().any(|row|
            row.session.as_deref() == Some(goal.session_id.as_str())
                && evidence.maps.get(&row.source).is_some_and(|source| source.stage == Stage::Inserted))
    { return Err("fresh actual status publications and live pose sources were not observed across the source-isolation window".into()); }
    if active_health.first().is_none_or(|row| row.at.duration_since(inserted_at) > RECENT_TTL)
        || active_health.last().is_none_or(|row| outcome_at.duration_since(row.at) > RECENT_TTL)
        || active_health.windows(2).any(|w| w[1].at.duration_since(w[0].at) > RECENT_TTL)
    { return Err("health observation contains a gap longer than the unchanged source TTL".into()); }
    let (representative_source, map) = representative
        .ok_or("no captured recent packet paired with a built/consumed inserted source")?;
    harness.world.audit_built(&map)?;
    let mut bodies = vec![insertion_body];
    bodies.extend(events.iter().filter(|e| e.at >= inserted_at && e.at <= outcome_at).map(|e| e.body));
    let hints = inspect_recent_hints(&map, &bodies, qualification)?;
    eprintln!("STATIC_SOURCE_MEASUREMENT_AUDIT qualification={} world={:?} scene={:?} persistent_stage=Inserted representative_source={representative_source} old_real_revision={} future_intersection=true inserted_recent_packets={inserted_packets} observed_health={} fixed_recent_geometry=true recent_geometry=original_clear source_time=live raw_hints_empty={} raw_hint_scope=offline_representative_reconstructed_snapshot raw_hint_report={hints:?} recent_guide_interference=unknown exact_plan_kernel_query=not_observed per_frame_consumer_ack=not_recorded coarse_arc_identity=not_claimed",
        qualification.label(), harness.world, harness.scene, initial.revision, active_health.len(), hints.is_empty());
    Ok(hints)
}

async fn qualify_static_source_only(
    scene: Scene, qualification: RecentQualification, world: wall_world::World,
) {
    qualify_static_source_only_then(scene,qualification,world,|_,_|{}).await;
}

// A diagnostic hook only after the unchanged cleanup/audits and failure report.
// Existing selectors use a no-op; first-partial evidence survives a failed loop.
async fn qualify_static_source_only_then(
    scene: Scene, qualification: RecentQualification, world: wall_world::World,
    after_cleanup:impl FnOnce(&OccupancyHarness,bool),
) {
    let mut harness = OccupancyHarness::start_with_recent_stage_and_world(
        true, scene, None, Some(Stage::Clear), world).await;
    let mut recent = typed_stream(&harness.transport, recent_cloud::DEFAULT_TOPIC, recent_cloud::TYPE_NAME).await.unwrap();
    let scans = Arc::new(Mutex::new(Vec::<ObservedRecent>::new()));
    let seen_scans = scans.clone();
    harness.observers.push(tokio::spawn(async move {
        while let Some(frame) = recent.next().await {
            let frame = frame.unwrap();
            // Only copy the received wire. No geometry/query work is added to
            // the running motion boundary or the 200 ms publisher.
            seen_scans.lock().unwrap().push(ObservedRecent {
                received: Instant::now(), received_unix_ns: current_unix_timestamp_ns().unwrap(),
                payload: frame.payload.to_vec(),
            });
        }
    }));
    let health = Arc::new(Mutex::new(Vec::<ObservedHealth>::new()));
    let seen_health = health.clone();
    let mut status_changes = harness.scheduler.subscribe_status();
    harness.observers.push(tokio::spawn(async move {
        // Await actual watch publications; polling status() would merely clone
        // cached freshness and could hide a stalled scheduler. Watch may coalesce
        // publications, so this still does not claim a per-command health proof.
        while status_changes.changed().await.is_ok() {
            let status = status_changes.borrow_and_update().clone();
            seen_health.lock().unwrap().push(ObservedHealth {
                at: Instant::now(), source: status.costmap_source_timestamp_ns,
                pose_source: status.pose_source_timestamp_ns,
                observed_unix_ns: current_unix_timestamp_ns().unwrap(),
                fresh: status.planner_inputs_fresh, session: status.session_id,
            });
        }
    }));
    let result = harness.exercise().await;
    let outcome_at = Instant::now();
    let status = harness.scheduler.status();
    // This is the unchanged cancellation/fence cleanup, including failure paths.
    let cleanup = harness.shutdown().await;
    harness.plant.lock().unwrap().sample();
    let audit = harness.audit_after_cleanup();
    let scans = scans.lock().unwrap();
    let health = health.lock().unwrap();
    let isolation = audit_static_source_isolation(&harness, &scans, &health, outcome_at, qualification);
    harness.dump_evidence();
    for row in health.iter() { eprintln!("STATIC_SOURCE_HEALTH {row:?}"); }
    let classification = if isolation.is_err() { "NOT_QUALIFIED" }
        else if result.is_err() || cleanup.is_err() || audit.is_err() { "QUALIFIED_OBSERVED_FAILURE" }
        else if scene == Scene::Box { "QUALIFIED_ORIGINAL_GOAL_ARRIVAL" }
        else { "QUALIFIED_BOUNDED_ACTIVE_WALL_HOLD" };
    let raw_hints_empty = isolation.as_ref().map_or("unknown", |hints| if hints.is_empty() { "true" } else { "false" });
    eprintln!("STATIC_SOURCE_FINAL qualification={} world={world:?} scene={scene:?} classification={classification} result={result:?} status={status:?} cleanup={cleanup:?} oracle_and_original_audit={audit:?} fixed_recent_measurement_audit={isolation:?} raw_hints_empty={raw_hints_empty} raw_hint_scope=offline_representative_reconstructed_snapshot recent_guide_interference=unknown captured_recent_packets={} motion_fixture_retuning=false", qualification.label(), scans.len());
    after_cleanup(&harness,cleanup.is_ok());
    assert!(result.is_ok() && cleanup.is_ok() && audit.is_ok() && isolation.is_ok(),
        "{classification}: result={result:?}; cleanup={cleanup:?}; audit={audit:?}; isolation={isolation:?}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
#[ignore = "original strict raw-empty Box diagnostic; independent from fixed-measurement qualification"]
async fn static_box_local_planning_on() {
    qualify_static_source_only(Scene::Box, RecentQualification::StrictRawEmpty,
        wall_world::World::LegacyObservedSupport).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
#[ignore = "original strict raw-empty Wall diagnostic; original shared Wall contract is unchanged"]
async fn static_wall_local_planning_on() {
    qualify_static_source_only(Scene::Wall, RecentQualification::StrictRawEmpty,
        wall_world::World::LegacyObservedSupport).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
#[ignore = "fixed recent geometry Box/ON; save new preflight first, all original motion oracles retained"]
async fn fixed_recent_box_local_planning_on() {
    qualify_static_source_only(Scene::Box, RecentQualification::FixedRecentGeometry,
        wall_world::World::LegacyObservedSupport).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
#[ignore = "fixed recent geometry + explicit wire-mask Wall negative; original goal/fence/whole-body oracle"]
async fn fixed_recent_wall_local_planning_on() {
    qualify_static_source_only(Scene::Wall, RecentQualification::FixedRecentGeometry,
        wall_world::World::ExplicitVirtualMaskDomain).await;
}

#[cfg(feature="local-guidance-qualification")]
#[tokio::test(flavor="multi_thread",worker_threads=2)]
#[ignore="first actual partial fact window; unchanged fixed-recent Box may still fail its original whole-loop oracle"]
async fn fixed_recent_box_first_partial_branch() {
    let capture=nuwax_nav_runtime::local_guidance_qualification::CaptureSession::first_partial_branch([3.5,1.5,0.5]);
    qualify_static_source_only_then(Scene::Box,RecentQualification::FixedRecentGeometry,
        wall_world::World::LegacyObservedSupport,move |harness,owners_stopped| {
            let (counts,captured)=capture.finish();
            super::local_guidance::dump_first_partial_branch(harness,owners_stopped,counts,captured);
        }).await;
}
