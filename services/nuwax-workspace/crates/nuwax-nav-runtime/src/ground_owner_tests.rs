//! C1-c3 bounded internal owner/control qualification.
//! Intended module: surface_follower::ground_owner_tests.
use super::*;
use crate::command_dispatch::ObservationProofAccumulator;
use crate::planning_owner::{Cancellation, PlanningOwner};
use crate::replan_feedback::GroundIntent;
use crate::route_planning::{GroundPlanInput, PlanKernel, PlanRequest};
use crate::surface_changes::SurfaceGeometryChanges;
use nuwax_map_chunks::{ColumnGroundRecords, Domain, HeightfieldSurface, Receiver, Repository, Transfer, Writer};
use nuwax_map_chunks::heightfield_observations::HeightfieldColumn;
use nuwax_nav_control::PlanarPose;
use nuwax_nav_mls_planner::{SurfaceBodyEnvelope, SurfaceObservation, SurfaceObservations};
use nuwax_nav_mls_planner::mls_planner::Config;
use nuwax_nav_mls_planner::surface::{BodyGroundState, BodyMotionProjection, SurfaceBodyQueryView};
use nuwax_nav_pct::ground_route::{GroundRouteGoal, GroundRouteStart, PlannedGroundRoute};
use nuwax_nav_pct::{PctPlanner, SearchLimits, Tomogram};
use nuwax_nav_scan::{BodyPose, CandidateFamily, GroundCandidateBinding, GroundReferenceCandidateOutcome, ScanConfig};
use nuwax_nav_types_messages::{NavFrameId, Pose3, Timestamp};
use std::cell::{Cell, RefCell};
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::sync::mpsc;
use std::time::{Duration, Instant};

type ObservationBlocks = BTreeMap<[i32; 3], BTreeMap<[i32; 3], SurfaceObservation>>;

// Existing Repository::temporary() owns Arc<tempfile::TempDir>. This helper
// retains no Repository clone, and verifies cleanup of both real root paths.
fn received_observations(mut writer: Writer, q: f32) -> (SurfaceObservations, ObservationBlocks) {
    let writer_path = writer.repository().path().to_path_buf();
    let root = writer.snapshot(Domain {
        session: "c1c2-native-fixed".into(), generation: 1,
        frame_generation: 0, producer_generation: 1,
    }, 1, 20).unwrap();
    let repository = Repository::temporary().unwrap();
    let receiver_path = repository.path().to_path_buf();
    let mut receiver = Receiver::new(repository);
    receiver.receive(&Transfer { root: root.encode().unwrap(), included: BTreeMap::new() },
        &root.domain, |_, r| writer.repository().read_object(r.object, nuwax_map_chunks::MAX_BLOCK_BYTES)).unwrap();
    let mut observations = SurfaceObservations::new(q, 32).unwrap();
    let mut blocks = BTreeMap::new();
    let mut parts = Vec::new();
    for r in &root.blocks {
        let (block, records) = receiver.ground_parts(r).unwrap();
        let values: BTreeMap<_, _> = block.iter().map(|(&local, e)| (
            nuwax_map_chunks::global_key(r.key, local).unwrap(),
            SurfaceObservation { observed_ns: e.observed_ns, hits: e.hits },
        )).collect();
        observations.replace_block(r.key, values.clone()).unwrap();
        blocks.insert(r.key, values);
        parts.push((r.key, records));
    }
    for xy in parts.iter().map(|(key, _)| [key[0], key[1]]).collect::<BTreeSet<_>>() {
        let columns = parts.iter().filter(|(key, _)| key[..2] == xy).cloned().collect::<Vec<_>>();
        observations.replace_heightfield_columns(xy, &columns);
    }
    observations.mark_complete_geometry_source();
    drop(receiver);
    drop(writer);
    assert!(!receiver_path.exists(), "Receiver temporary root survives teardown");
    assert!(!writer_path.exists(), "Writer temporary root survives teardown");
    (observations, blocks)
}

// Exact historical C1-c2 Direct points, q, model, start and goal. Initial fine
// observations describe precisely those occupied keys, allowing an actual
// metadata-only refresh instead of pretending a newly allocated Arc is one.
pub(crate) fn c3_direct_source() -> (SurfaceSnapshot, ObservationBlocks) {
    let q = 0.1_f32;
    let mut writer = Writer::new(Repository::temporary().unwrap(), q, 4 * 1024 * 1024).unwrap();
    let keys: Vec<_> = (-15..20).flat_map(|x| (-8..15).map(move |y| [x, y, -1])).collect();
    writer.observe(keys, 20).unwrap();
    let points: Vec<_> = (-15..20).flat_map(|x| (-8..15).map(move |y|
        ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, -0.05))).collect();
    let config = Config { world_frame: "map".into(), voxel_size: 0.1, robot_height: 0.55,
        surface_closing_radius: 0.0, node_spacing_m: 0.3, wall_clearance_m: 0.0,
        wall_buffer_m: 0.1, wall_buffer_weight: 2.0, max_slope_deg: 18.0,
        step_threshold_m: 0.2, min_step_tread_depth_m: 0.2, step_penalty_weight: 1.0,
        goal_tolerance: 0.1, viz_publish_hz: 0.0 };
    let model = SurfaceBodyEnvelope { fixed_collision: None,
        footprint_xy_m: vec![[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]],
        z_bounds_m: [-0.1, 0.1], body_to_support_z_m: 0.4,
        max_step_up_m: 0.2, max_step_down_m: 0.2, max_cross_slope_deg: 18.0,
        support_uncertainty_m: 0.0 };
    let (observations, blocks) = received_observations(writer, q);
    (SurfaceSnapshot::from_points(&points, config, q).unwrap().with_body_envelope(model).unwrap()
        .with_observations(Some(Arc::new(observations))), blocks)
}

// Approved third C1-c2 fixture, unchanged: q=.05; lower member0/bin7;
// upper member11/bin0 -> bin7 at x>=1. No fourth two-layer fixture.
pub(crate) fn c3_bin_shift_source() -> SurfaceSnapshot {
    let q = 0.05_f32;
    let mut writer = Writer::new(Repository::temporary().unwrap(), q, 4 * 1024 * 1024).unwrap();
    let mut points = Vec::new();
    let mut columns = Vec::new();
    for x in -8..17 { for y in -8..17 {
        let mut heightfields = Vec::new();
        for (z, bin) in [(0, 7), (11, if x >= 1 { 7 } else { 0 })] {
            writer.observe([[x, y, z]], 20).unwrap();
            let fraction = (bin as f32 + 0.5) / 8.0;
            points.push(((x as f32 + 0.5) * q, (y as f32 + 0.5) * q, (z as f32 + fraction) * q));
            let mut counts = [0; 8];
            counts[bin] = 1000;
            heightfields.push(HeightfieldSurface {
                column: HeightfieldColumn::from_counts([x, y], q,
                    BTreeMap::from([(z, counts)])).unwrap().unwrap(), samples: vec![],
            });
        }
        columns.push(([x, y], ColumnGroundRecords { classified_empty: false, legacy: vec![], heightfields }));
    }}
    writer.replace_ground_columns(columns.iter().map(|(xy, c)| (*xy, c))).unwrap();
    let (observations, _) = received_observations(writer, q);
    let config = Config { world_frame: "map".into(), voxel_size: 0.1, robot_height: 0.2,
        surface_closing_radius: 0.0, node_spacing_m: 0.3, wall_clearance_m: 0.0,
        wall_buffer_m: 0.2, wall_buffer_weight: 2.0, max_slope_deg: 18.0,
        step_threshold_m: 0.21, min_step_tread_depth_m: 0.2, step_penalty_weight: 2.0,
        goal_tolerance: 0.1, viz_publish_hz: 0.0 };
    let model = SurfaceBodyEnvelope { fixed_collision: None,
        footprint_xy_m: vec![[-0.01, -0.01], [0.01, -0.01], [0.01, 0.01], [-0.01, 0.01]],
        z_bounds_m: [-0.1, 0.1], body_to_support_z_m: 0.4,
        max_step_up_m: 0.21, max_step_down_m: 0.21, max_cross_slope_deg: 18.0,
        support_uncertainty_m: 0.0 };
    SurfaceSnapshot::from_points(&points, config, q).unwrap().with_body_envelope(model).unwrap()
        .with_observations(Some(Arc::new(observations)))
}

fn c3_motion() -> MotionKinematics {
    MotionKinematics { model: MotionModel::Differential, max_forward_mps: 1.0,
        max_reverse_mps: 0.0, max_lateral_mps: 0.0, max_angular_rps: 0.8,
        minimum_turning_radius_m: 0.0, can_rotate_in_place: true }
}
pub(crate) fn c3_config() -> SurfaceFollowerConfig {
    let mut config = SurfaceFollowerConfig::new(0.4);
    config.controller.simulation = true;
    config.basic_kinematics = Some(c3_motion());
    config
}
pub(crate) fn c3_start() -> BodyPose { ([-1.0, 0.2, 0.4], [0.0, 0.0, 0.0, 1.0]) }
fn c3_xy_reference() -> Vec<[f64; 2]> { vec![[-1.0, 0.2], [0.1, 0.2]] }
pub(crate) fn c3_stamped(body: BodyPose) -> PoseStamped {
    PoseStamped { frame_id: NavFrameId::Map, stamp: Timestamp::from_unix(1, 0),
        pose: Pose3 { position: Point3::new(body.0[0], body.0[1], body.0[2]),
            orientation: Quaternion { x: body.1[0], y: body.1[1], z: body.1[2], w: body.1[3] } } }
}
fn c3_route(planner: &PctPlanner, source: &SurfaceSnapshot, actual: BodyPose,
    height: Option<f64>, to: [f64; 2]) -> PlannedGroundRoute {
    planner.ground_route_with_limits_and_penalties(source, actual,
        GroundRouteStart { xy: [actual.0[0], actual.0[1]], ground_z: height },
        GroundRouteGoal { xy: to, requested_ground_z: None }, 0.0, &[], None, None,
        SearchLimits::default(), &[]).expect("fixed real PCT route prerequisite")
}
fn c3_direct(source: &SurfaceSnapshot, route: &PlannedGroundRoute) -> nuwax_nav_scan::MappedLocalTrajectory {
    let start = c3_start();
    let ground = source.body_query().start_ground_state(start).unwrap();
    let incoming = route.reference.full_window().unwrap().from().continuation([start.0[0], start.0[1]]).unwrap();
    let fork = incoming.planning_fork(source).unwrap();
    let tomo = Tomogram::from_surface(fork.snapshot()).unwrap();
    let mapped = nuwax_nav_scan::plan_driven_ground_on_fork(&fork, &tomo, start, &ground,
        incoming.walk_state(), &c3_xy_reference(), &c3_motion(), &ScanConfig::default(), None,
        Some(([0.1, 0.2], 0.1)), &[], false, false, None, route.reference.full_window().unwrap()).unwrap();
    assert_eq!(mapped.binding.family(), CandidateFamily::Direct);
    assert_eq!(mapped.binding.edges().len() + 1, mapped.trajectory.poses.len());
    for (edge, pair) in mapped.trajectory.poses.windows(2).enumerate() {
        assert_eq!(mapped.binding.edges()[edge].motion(), [pair[0], pair[1]]);
    }
    mapped
}
fn c3_project(query: &SurfaceBodyQueryView, navigation: nuwax_nav_pct::ground_route::GroundWalkState<'_>,
    actual: BodyPose, ground: &BodyGroundState, to: [f64; 2]) -> BodyMotionProjection {
    let walk = navigation.walk_xy(to, &[], None).unwrap();
    walk.with_body_navigation(|_, segment| {
        let projected = query.project_motion_with_navigation(actual, ground, to, 0.0, segment, &mut None).unwrap();
        query.validate_projected_motion(actual, &projected, &[], &mut None).unwrap();
        projected
    }).unwrap()
}

#[test]
fn c1c3_01_bound_fork_keeps_metadata_and_current_recent() {
    let (source, mut blocks) = c3_direct_source();
    let planner = PctPlanner::default();
    let route = c3_route(&planner, &source, c3_start(), None, [0.1, 0.2]);
    let old_tomo = planner.map(&source).unwrap();
    let mut observations = source.observations().unwrap().as_ref().clone();
    for (&key, block) in &mut blocks {
        for observation in block.values_mut() { observation.observed_ns = 900; observation.hits = 17; }
        assert!(observations.replace_block_with_geometry_changes(key, block.clone()).unwrap().is_empty());
    }
    observations.mark_complete_geometry_source();
    let timed = source.with_observations(Some(Arc::new(observations.clone())));
    assert!(nuwax_nav_pct::same_source_geometry(&source, &timed));
    assert!(!Arc::ptr_eq(source.observations().unwrap(), timed.observations().unwrap()));
    assert_eq!(timed.observations().unwrap().get([-10, 2, -1]).unwrap().observed_ns, 900);
    let reused = planner.map_with_delta(&timed, nuwax_nav_pct::SurfaceDelta::Unchanged { basis: &source }).unwrap();
    assert!(Arc::ptr_eq(&old_tomo, &reused), "real metadata-only update retains original Tomogram");
    let start = c3_start();
    let to = [-0.95, 0.2];
    let incoming = route.reference.full_window().unwrap().from().continuation([-1.0, 0.2]).unwrap();
    let physical = source.body_query().start_ground_state(start).unwrap();
    let ordinary = timed.fork_for_planning();
    assert!(!ordinary.body_query().same_ground_source(&source));
    let walk = incoming.walk_state().walk_xy(to, &[], None).unwrap();
    let rejected = walk.with_body_navigation(|_, segment|
        ordinary.body_query().project_motion_with_navigation(start, &physical, to, 0.0, segment, &mut None)).unwrap();
    assert!(rejected.is_err(), "ordinary private fork must retain strict source-owner rejection");
    let fork = incoming.planning_fork(&timed).unwrap();
    assert!(!fork.body_query().same_ground_source(&source));
    c3_project(fork.body_query(), incoming.walk_state(), start, &physical, to);
    c3_project(&incoming.bind_body_query(&timed).unwrap(), incoming.walk_state(), start, &physical, to);
    // Historical C1-c2 recent point, unchanged. It is attached to CURRENT query,
    // while navigation keeps the old source owner, so a witness is compulsory.
    let recent = timed.with_recent_obstacle_points(1, 1_000_000_000, 0, &[],
        [(0, [-0.95, 0.2, 0.4])]).unwrap();
    assert!(nuwax_nav_pct::same_source_geometry(&source, &recent));
    let current_fork = incoming.planning_fork(&recent).unwrap();
    let mut witness = None;
    let blocked = walk.with_body_navigation(|_, segment| {
        let projected = current_fork.body_query().project_motion_with_navigation(start, &physical,
            to, 0.0, segment, &mut witness)?;
        current_fork.body_query().validate_projected_motion(start, &projected, &[], &mut witness)
    }).unwrap();
    assert!(blocked.is_err());
    assert!(witness.is_some(), "current recent obstacle must survive paired private fork");
    // A true fine-column geometry change keeps coarse data but retires the basis.
    let key = [0, 2, 4];
    let block_key = key.map(|v: i32| v.div_euclid(32));
    let block = blocks.entry(block_key).or_default();
    block.insert(key, SurfaceObservation { observed_ns: 901, hits: 2 });
    assert!(!observations.replace_block_with_geometry_changes(block_key, block.clone()).unwrap().is_empty());
    observations.mark_complete_geometry_source();
    let changed = source.with_observations(Some(Arc::new(observations)));
    assert!(!nuwax_nav_pct::same_source_geometry(&source, &changed));
    assert!(incoming.planning_fork(&changed).is_err());
    assert!(incoming.bind_body_query(&changed).is_err());
    assert!(!route.reference.matches_source(&changed));
}

#[test]
fn c1c3_02_original_sample_and_actual_xy_have_distinct_roles() {
    let (source, _) = c3_direct_source();
    let route = c3_route(&PctPlanner::default(), &source, c3_start(), None, [0.1, 0.2]);
    let mapped = c3_direct(&source, &route);
    let candidate = mapped.binding.candidate();
    let grids:Vec<_>=(0..candidate.connection_count()).map(|i|candidate.connection(i).unwrap()).collect();
    eprintln!("fixed Scan grids={grids:?}");
    // Scan's unchanged .05 m resampling yields two PCT intervals per edge.
    // The original >=3 premise failed before any decoder assertion; retain the
    // actual winner and qualify its real divided grid rather than stretch it.
    assert!(grids.iter().all(|(_,_,steps)|*steps==2));
    let edge = (0..candidate.connection_count()).find(|&i| {
        let (from, to, _) = candidate.connection(i).unwrap();from != to
    }).expect("fixed straight must retain real motion");
    let (from, to, steps) = candidate.connection(edge).unwrap();
    let full = mapped.binding.slice(edge, 0.0, 1.0).unwrap();
    // All decoder consumers must choose the same right-continuous original
    // address, including exactly representable neighbours of each i/steps.
    for i in 1..steps {
        let t = i as f64 / steps as f64;
        for (fraction, index) in [(t.next_down(), i - 1), (t, i), (t.next_up(), i)] {
            let decoded = candidate.original_sample(edge, fraction).unwrap();
            assert_eq!(decoded.0, index);
            if fraction == t { assert_eq!(decoded.1, 0.0); }
            assert_eq!(full.original_sample(fraction), Some(decoded));
            let actual_xy = std::array::from_fn(|axis| from[axis] + fraction * (to[axis] - from[axis]));
            let borrowed = candidate.walk_state_at(edge, fraction, actual_xy).unwrap();
            let owned = candidate.continuation_at(edge, fraction, actual_xy).unwrap();
            let sample = candidate.sample(edge, index).unwrap();
            assert_eq!(borrowed.reference_sample_xy(), sample.point.xy);
            assert_eq!(borrowed.planned_ground_z(), sample.point.ground_z);
            assert_eq!(borrowed.semantic_key(), owned.semantic_key());
        }
    }
    // Independent evaluator-grid qualification on the same original full
    // straight XY input. This is a real source walk, not a Scan winner edge.
    let window=route.reference.full_window().unwrap();
    let original_walk=window.from().walk_state().walk_xy(window.to().walk_state().xy(),&[],None).unwrap();
    let original_grid=route.reference.freeze_candidate(vec![original_walk]).unwrap();
    let original_steps=original_grid.connection(0).unwrap().2;
    eprintln!("independent original straight steps={original_steps}");
    assert_eq!(original_steps,28);
    for index in 1..original_steps {
        let t=index as f64/original_steps as f64;
        assert_eq!(original_grid.original_sample(0,t).unwrap(),(index,0.0));
        assert_eq!(original_grid.original_sample(0,t.next_down()).unwrap().0,index-1);
        assert_eq!(original_grid.original_sample(0,t.next_up()).unwrap().0,index);
    }
    let t = (steps / 2) as f64 / steps as f64;
    let index = candidate.original_sample(edge, t).unwrap().0;
    let original = candidate.sample(edge, index).unwrap();
    let on_line = std::array::from_fn(|axis| from[axis] + t * (to[axis] - from[axis]));
    let actual_xy = [on_line[0], on_line[1] + 0.001];
    let q = f64::from(source.source_voxel_size_m());
    assert_ne!((original.point.xy[1] / q).floor() as i32, (actual_xy[1] / q).floor() as i32,
        "fixed lateral offset must really cross an adjacent fine column");
    assert_eq!(((original.point.xy[1] / q).floor() - (actual_xy[1] / q).floor()).abs(), 1.0);
    let seed = candidate.continuation_at(edge, t, actual_xy).unwrap();
    let nominal = candidate.continuation_at(edge, t, on_line).unwrap();
    assert_eq!(seed.walk_state().reference_sample_xy(), original.point.xy);
    assert_eq!(seed.walk_state().xy(), actual_xy);
    assert_ne!(seed.walk_state().reference_sample_xy(), seed.walk_state().xy());
    assert_eq!(seed.walk_state().planned_ground_z(), original.point.ground_z);
    assert_ne!(seed.semantic_key(), nominal.semantic_key(), "actual input belongs in the motion key");
    let slice = mapped.binding.slice(edge, 0.5 / steps as f64, 1.0).unwrap();
    assert_eq!(slice.original_sample(0.0), Some((0, 0.5)));
    assert_eq!(slice.original_sample(1.0), Some((steps, 0.0)));
    assert_eq!(slice.binding().candidate().connection(edge).unwrap().2, steps);
    let actual = ([actual_xy[0], actual_xy[1], c3_start().0[2]], c3_start().1);
    let query = seed.bind_body_query(&source).unwrap();
    let ground = query.start_ground_state(actual).unwrap();
    let end = [actual_xy[0] + 0.05, actual_xy[1]];
    c3_project(&query, seed.walk_state(), actual, &ground, end);
    let next = seed.walk_state().walk_xy(end, &[], None).unwrap();
    assert_eq!(next.end().xy(), end);
    assert_eq!(next.end().reference_sample_xy(), end,
        "new edge endpoint is its evaluator endpoint, not the old candidate sample");
    assert_eq!(candidate.original_sample(edge, t), Some((index, 0.0)));
}

fn c3_request(source: Arc<SurfaceSnapshot>, route: &PlannedGroundRoute, generation: u64) -> PlanRequest {
    let start = c3_start();
    let window = route.reference.full_window().unwrap();
    PlanRequest {
        ground: Some(GroundPlanInput {
            incoming: window.from().continuation([start.0[0], start.0[1]]).unwrap(),
            intent: GroundIntent::new(window),
            xy_reference: Arc::new(c3_xy_reference()), arrival_tolerance_m: Some(0.1),
        }),
        start_ground: source.body_query().start_ground_state(start).map_err(|error| error.to_string()),
        source, change_basis: None, changes: Arc::new(SurfaceGeometryChanges::Unknown),
        pose: c3_stamped(start), boundary: Arc::new(vec![]), reference: vec![], reference_source: None,
        final_yaw: None, arrival: None, end: 1.1, progress: 0.0, generation,
        straight: false, detour: false, scan: ScanConfig::default(),
    }
}
fn c3_recv<T>(rx: &mpsc::Receiver<T>) -> T {
    rx.recv_timeout(Duration::from_secs(30)).expect("real owner gate timed out")
}
fn c3_until(mut ready: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(30);
    while !ready() {
        assert!(Instant::now() < deadline, "real owner publication/retirement timed out");
        std::thread::yield_now();
    }
}
// Declared after PlanningOwner so panic cleanup unblocks both gates BEFORE its
// original Drop cancels and joins the only worker. No detached future remains.
struct C3GateCleanup { compute: mpsc::Sender<()>, publish: mpsc::Sender<()> }
impl Drop for C3GateCleanup {
    fn drop(&mut self) {
        for _ in 0..4 { let _ = self.compute.send(()); let _ = self.publish.send(()); }
    }
}

#[test]
fn c1c3_03_real_planning_owner_carries_ground_binding() {
    let (source, _) = c3_direct_source();
    let source = Arc::new(source);
    let route = c3_route(&PctPlanner::default(), &source, c3_start(), None, [0.1, 0.2]);
    let mut actual_owner = SurfaceFollower::new(c3_config()).unwrap();
    actual_owner.update_execution_map(ExecutionMap::Surface(source.clone())).unwrap();
    actual_owner.update_odometry(c3_stamped(c3_start()));
    assert!(actual_owner.ground_cursor().is_none());
    let mut kernel = PlanKernel::new(c3_config());
    let (started_tx, started_rx) = mpsc::channel();
    let (compute_tx, compute_rx) = mpsc::channel();
    let (computed_tx, computed_rx) = mpsc::channel();
    let (publish_tx, publish_rx) = mpsc::channel();
    let owner = PlanningOwner::new(move |request: PlanRequest, cancel: &Cancellation| {
        let generation = request.generation;
        started_tx.send(generation).unwrap();
        c3_recv(&compute_rx);
        let result = kernel.compute(request, cancel); // real Scan, no outcome hook
        let first = result.outcome.as_ref().ok().map(|candidate| body_control::body(&candidate.path.poses[0]));
        computed_tx.send((generation, result.outcome.is_ok(), first)).unwrap();
        c3_recv(&publish_rx);
        result
    }).unwrap();
    let cleanup = C3GateCleanup { compute: compute_tx.clone(), publish: publish_tx.clone() };
    assert!(owner.submit(1, c3_request(source.clone(), &route, 1)));
    assert_eq!(c3_recv(&started_rx), 1);
    compute_tx.send(()).unwrap();
    assert_eq!(c3_recv(&computed_rx), (1, true, Some(c3_start())));
    assert!(owner.take().is_none(), "computed but gated result is not yet a completion");
    assert!(actual_owner.ground_cursor().is_none());
    publish_tx.send(()).unwrap();
    c3_until(|| owner.completion_ready_for_test());
    let completion = owner.take().unwrap();
    assert_eq!(completion.id, 1);
    assert_eq!(completion.epoch, owner.epoch());
    let candidate = completion.value.outcome.expect("real finite-window kernel candidate");
    let binding = candidate.ground_binding.expect("real kernel must carry its actual Scan winner binding");
    assert_eq!(binding.family(), CandidateFamily::Direct);
    assert_eq!(candidate.path.poses.len(), binding.edges().len() + 1);
    for (edge, pair) in candidate.path.poses.windows(2).enumerate() {
        assert_eq!(binding.edges()[edge].motion(), [body_control::body(&pair[0]), body_control::body(&pair[1])]);
    }
    let goal = candidate.path.poses.last().unwrap().clone();
    actual_owner.admit_ground_candidate(candidate.path, goal, binding.clone(), None, None).unwrap();
    let cursor = actual_owner.ground_cursor();
    let progress = actual_owner.progress_m();
    assert_eq!(cursor, Some(ground_cursor::GroundCursor { edge: 0, fraction: 0.0 }));
    assert_eq!(progress, 0.0);
    // Separate real ReferenceOnly paired-entry qualification on this same
    // finite window. Does not claim kernel partial -> ReferenceOnly was reached.
    let mut request = c3_request(source.clone(), &route, 2);
    let input = request.ground.as_ref().unwrap();
    let fork = input.incoming.planning_fork(&source).unwrap();
    let tomo = Tomogram::from_surface(fork.snapshot()).unwrap();
    let outcome = nuwax_nav_scan::probe_ground_reference_candidates_on_fork(&fork, &tomo,
        c3_start(), request.start_ground.as_ref().unwrap(), input.incoming.walk_state(),
        &input.xy_reference, &c3_motion(), &request.scan, None, Some(([0.1, 0.2], 0.1)),
        &[], false, false, None, input.intent.window()).unwrap();
    match outcome {
        GroundReferenceCandidateOutcome::Complete(mapped) => {
            assert_eq!(mapped.binding.family(), CandidateFamily::Direct);
            assert_eq!(mapped.binding.edges().len() + 1, mapped.trajectory.poses.len());
        }
        GroundReferenceCandidateOutcome::Miss => panic!("fixed Direct reference candidate unexpectedly missed"),
    }
    assert_eq!(actual_owner.ground_cursor(), cursor);
    // A real successful result completed before cancellation, but gated before
    // publication, must be retired by the original owner rather than installed.
    let edge = binding.edges().iter().position(|edge| edge.motion()[0].0 != edge.motion()[1].0).unwrap();
    let motion = binding.edges()[edge].motion();
    let resumed: BodyPose = (std::array::from_fn(|axis|
        motion[0].0[axis] + 0.5 * (motion[1].0[axis] - motion[0].0[axis])), motion[0].1);
    let incoming = binding.candidate().continuation_at(edge, 0.5, [resumed.0[0], resumed.0[1]]).unwrap();
    let old_intent = request.ground.as_ref().unwrap().intent.clone();
    request.pose = c3_stamped(resumed);
    request.start_ground = source.body_query().start_ground_state(resumed).map_err(|error| error.to_string());
    let input = request.ground.as_mut().unwrap();
    input.incoming = incoming;
    input.xy_reference = Arc::new(vec![[resumed.0[0], resumed.0[1]], [0.1, 0.2]]);
    assert!(input.intent.same_intent(&old_intent));
    assert_ne!(input.incoming.walk_state().xy(), input.intent.window().from().walk_state().xy());
    assert!(owner.submit(2, request));
    assert_eq!(c3_recv(&started_rx), 2);
    compute_tx.send(()).unwrap();
    assert_eq!(c3_recv(&computed_rx), (2, true, Some(resumed)));
    let old_epoch = owner.epoch();
    owner.cancel();
    assert!(owner.epoch() > old_epoch);
    publish_tx.send(()).unwrap();
    c3_until(|| owner.is_idle_for_test());
    assert!(owner.take().is_none());
    assert_eq!(actual_owner.ground_cursor(), cursor);
    assert_eq!(actual_owner.progress_m(), progress);
    // Cancellation before real kernel.compute must report no usable candidate;
    // its returned error future is also fully retired and drained.
    assert!(owner.submit(3, c3_request(source, &route, 3)));
    assert_eq!(c3_recv(&started_rx), 3);
    owner.cancel();
    compute_tx.send(()).unwrap();
    assert_eq!(c3_recv(&computed_rx), (3, false, None));
    publish_tx.send(()).unwrap();
    c3_until(|| owner.is_idle_for_test());
    assert!(owner.take().is_none());
    assert!(!owner.faulted());
    assert_eq!(actual_owner.ground_cursor(), cursor);
    assert_eq!(actual_owner.progress_m(), progress);
    drop(cleanup);
    drop(owner); // actual original worker join
}

#[test]
fn c1c3_04_navigation_memo_preserves_real_distinct_successors() {
    let map = c3_bin_shift_source();
    let q = f64::from(0.05_f32);
    let xy = [0.025, 0.025];
    let to = [0.125, 0.025];
    let actual = ([xy[0], xy[1], 0.705], [0.0, 0.0, 0.0, 1.0]);
    let observations = map.observations().unwrap();
    let records: Vec<_> = observations.ground_records([0, 0]).collect();
    let endpoints: Vec<_> = observations.ground_records([2, 0]).collect();
    assert_eq!(records.len(), 2);
    assert_eq!(endpoints.len(), 2);
    assert_eq!(records[0].members().collect::<Vec<_>>(), vec![0]);
    assert_eq!(records[1].members().collect::<Vec<_>>(), vec![11]);
    assert_eq!(endpoints[1].members().collect::<Vec<_>>(), vec![11]);
    let low = records[0].height_at(xy).unwrap();
    let high = records[1].height_at(xy).unwrap();
    assert_eq!(low, 7.5 * q / 8.0);
    assert_eq!(high, (11.0 + 0.5 / 8.0) * q);
    assert_eq!(endpoints[0].height_at(to).unwrap(), low);
    assert_eq!(endpoints[1].height_at(to).unwrap(), (11.0 + 7.5 / 8.0) * q);
    let model = map.body_envelope().unwrap();
    assert!(11.0 * q - low > model.body_to_support_z_m + model.z_bounds_m[1]);
    let support = actual.0[2] - model.body_to_support_z_m;
    assert!((support - low).abs() < model.max_step_up_m + q);
    assert!((support - high).abs() < model.max_step_down_m + q);
    assert!(actual.0[2] + model.z_bounds_m[0] > 12.0 * q);
    let tomo = Tomogram::from_surface(&map).unwrap();
    let low_id = tomo.cell_at([xy[0], xy[1], low]).unwrap();
    let high_id = tomo.cell_at([xy[0], xy[1], high]).unwrap();
    assert_ne!(low_id, high_id);
    assert_eq!(tomo.cells[low_id].source_ground, Some(records[0].key()));
    assert_eq!(tomo.cells[high_id].source_ground, Some(records[1].key()));
    let planner = PctPlanner::default();
    let lower = c3_route(&planner, &map, actual, Some(low), to);
    let upper = c3_route(&planner, &map, actual, Some(high), to);
    let a = lower.reference.full_window().unwrap().from().walk_state();
    let b = upper.reference.full_window().unwrap().from().walk_state();
    assert_ne!(a.semantic_key(), b.semantic_key());
    lower.reference.freeze_candidate(vec![b.walk_xy(xy, &[], None).unwrap()]).unwrap();
    let query = a.bind_body_query(&map).unwrap();
    let physical = query.start_ground_state(actual).unwrap();
    // Independent direct physical qualifier before testing the real memo.
    let plain_a = c3_project(&query, a, actual, &physical, to);
    let plain_b = c3_project(&query, b, actual, &physical, to);
    assert_eq!(plain_a.pose().0[2], actual.0[2]);
    assert!((plain_b.pose().0[2] - actual.0[2] - 7.0 * q / 8.0).abs() < 1e-12);
    assert_ne!(plain_a.pose(), plain_b.pose());
    let cache = RefCell::new(HashMap::new());
    let proof = ObservationProofAccumulator::new(None);
    let evaluations = Cell::new(0);
    let target = PlanarPose { x: to[0], y: to[1], yaw: 0.0 };
    let run = |navigation| body_control::memoized_control_motion(&cache, &proof, actual, target,
        false, &physical, Some(navigation), || {
            evaluations.set(evaluations.get() + 1);
            body_control::control_project_edge(&query, actual, &physical, Some(navigation),
                to, 0.0, &[], body_control::ControlEdgeProfile::Preview)
        }).unwrap();
    let cold_a = run(a);
    let cold_b = run(b);
    assert_eq!(evaluations.get(), 2, "distinct real navigation inputs need two cold evaluations");
    let hit_b = run(b);
    let hit_a = run(a);
    assert_eq!(evaluations.get(), 2, "both repeated keys must reuse their complete successes");
    assert_eq!(cache.borrow().len(), 2);
    assert_eq!(cold_a.body.pose, plain_a.pose());
    assert_eq!(cold_b.body.pose, plain_b.pose());
    assert_ne!(cold_a.body.pose, cold_b.body.pose);
    for (cold, hit, incoming) in [(&cold_a, &hit_a, a), (&cold_b, &hit_b, b)] {
        assert_eq!(cold.body, hit.body);
        let cold_navigation = cold.navigation.expect("Ground cold result lost navigation endpoint");
        let hit_navigation = hit.navigation.expect("Ground memo hit lost navigation endpoint");
        assert_eq!(cold_navigation.semantic_key(), hit_navigation.semantic_key());
        assert_ne!(hit_navigation.semantic_key(), incoming.semantic_key());
        assert_eq!(hit_navigation.xy(), to);
        assert_eq!(hit_navigation.reference_sample_xy(), to);
        // Continue from the returned memo endpoint, not the old input association.
        let successor = [0.175, 0.025];
        let from_cold = body_control::control_project_edge(&query, cold.body.pose,
            cold.body.ground.as_ref().unwrap(), Some(cold_navigation), successor, 0.0,
            &[], body_control::ControlEdgeProfile::Preview).unwrap();
        let from_hit = body_control::control_project_edge(&query, hit.body.pose,
            hit.body.ground.as_ref().unwrap(), Some(hit_navigation), successor, 0.0,
            &[], body_control::ControlEdgeProfile::Preview).unwrap();
        assert_eq!(from_cold.body, from_hit.body);
        assert_eq!(from_cold.navigation.unwrap().semantic_key(), from_hit.navigation.unwrap().semantic_key());
    }
    assert_ne!(hit_a.navigation.unwrap().semantic_key(), hit_b.navigation.unwrap().semantic_key());
    assert_eq!(proof.bound().unwrap(), None);
}

// C1-c3 exacts 05–10: original finite control geometry and lifecycle inputs.
// Existing Body fixture/model; no source/model thresholds are widened.
fn c3_body_source() -> SurfaceSnapshot {
    super::test_snapshot(0.0, false).with_body_envelope(SurfaceBodyEnvelope {
        fixed_collision: None,
        footprint_xy_m: vec![[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]],
        z_bounds_m: [-0.05, 0.2], body_to_support_z_m: 0.3,
        max_step_up_m: 0.2, max_step_down_m: 0.2, max_cross_slope_deg: 20.0,
        support_uncertainty_m: 0.0,
    }).unwrap()
}
fn c3_pose(x: f64, y: f64, yaw: f64) -> PoseStamped {
    let mut pose = c3_stamped(([x, y, 0.4], [0.0, 0.0, 0.0, 1.0]));
    pose.pose.orientation = Quaternion::from_yaw(yaw);
    pose
}
pub(crate) fn c3_limits() -> MotionLimits {
    // Original runtime_ground_owner_* complete-stop calibration.
    MotionLimits { model: MotionModel::Differential, max_forward_mps: 0.5,
        max_reverse_mps: 0.0, max_lateral_mps: 0.0, max_angular_rps: 0.8,
        linear_acceleration_mps2: 1.0, guaranteed_braking_mps2: 1.0,
        angular_acceleration_rps2: 2.0, guaranteed_angular_braking_rps2: 2.0,
        reaction_time_sec: 0.05, minimum_turning_radius_m: 0.0, can_rotate_in_place: true }
}
fn c3_install(source: Arc<SurfaceSnapshot>, actual: BodyPose, to: [f64; 2],
    height: Option<f64>, mut config: SurfaceFollowerConfig)
    -> (SurfaceFollower, Arc<GroundCandidateBinding>, Path3D, PlannedGroundRoute) {
    let route = c3_route(&config.pct, &source, actual, height, to);
    let incoming = route.reference.full_window().unwrap().from()
        .continuation([actual.0[0], actual.0[1]]).unwrap();
    let fork = incoming.planning_fork(&source).unwrap();
    let tomo = Tomogram::from_surface(fork.snapshot()).unwrap();
    let physical = source.body_query().start_ground_state(actual).unwrap();
    let mapped = nuwax_nav_scan::plan_driven_ground_on_fork(&fork, &tomo, actual, &physical,
        incoming.walk_state(), &[[actual.0[0], actual.0[1]], to], &c3_motion(), &config.scan,
        None, None, &[], false, false, None, route.reference.full_window().unwrap()).unwrap();
    let binding = Arc::new(mapped.binding);
    let path = Path3D { poses: mapped.trajectory.poses.into_iter().map(c3_stamped).collect() };
    config.controller.simulation = true;
    config.basic_kinematics = Some(c3_motion());
    let mut owner = SurfaceFollower::new(config).unwrap();
    owner.update_execution_map(ExecutionMap::Surface(source)).unwrap();
    owner.update_odometry(c3_stamped(actual));
    assert_eq!(path.poses[0].pose, c3_stamped(actual).pose, "real Scan keeps the original initial full pose");
    if actual.1[0] != 0.0 {
        let interpolated = super::interpolate_pose(&path.poses[0], &path.poses[1], 0.0);
        eprintln!("tilted initial: actual={actual:?}; ratio_zero={:?}", body_control::body(&interpolated));
        assert_ne!(interpolated.pose, path.poses[0].pose, "fixed tilted input exposes ratio-zero normalization");
    }
    owner.admit_ground_candidate(path.clone(), path.poses.last().unwrap().clone(), binding.clone(), None, None).unwrap();
    (owner, binding, path, route)
}
fn c3_installed_body() -> (SurfaceFollower, Arc<GroundCandidateBinding>, Path3D, PlannedGroundRoute) {
    let mut config = SurfaceFollowerConfig::new(0.3);
    config.motion_limits = Some(c3_limits());
    c3_install(Arc::new(c3_body_source()), body_control::body(&c3_pose(1.05, 1.05, 0.0)),
        [1.55, 1.05], None, config)
}
fn c3_narrow_door() -> SurfaceSnapshot {
    // Exact original narrow_door_connection_requires_real_alignment_before_entry.
    let fine = f64::from(0.01_f32);
    let width = 32.0 * fine;
    let mut keys: Vec<[i32; 3]> = (-100..100).flat_map(|x| (-50..100).map(move |y| [x, y, -1])).collect();
    keys.extend((0..4).flat_map(|x| (-50..100).filter(|&y| y < 0 || y >= 33)
        .flat_map(move |y| (0..80).map(move |z| [x, y, z]))));
    let points: Vec<_> = keys.iter().map(|k| (((f64::from(k[0]) + 0.5) * fine) as f32,
        ((f64::from(k[1]) + 0.5) * fine) as f32, ((f64::from(k[2]) + 0.5) * fine) as f32)).collect();
    let mut blocks = ObservationBlocks::new();
    for key in keys { blocks.entry(key.map(|v| v.div_euclid(32))).or_default()
        .insert(key, SurfaceObservation { observed_ns: 0, hits: 0 }); }
    let mut observations = SurfaceObservations::new(0.01, 32).unwrap();
    for (key, block) in blocks { observations.replace_block(key, block).unwrap(); }
    observations.mark_complete_geometry_source();
    let mut geometry = super::test_snapshot(0.0, false).config().clone();
    geometry.voxel_size = 0.1; geometry.robot_height = 0.1; geometry.surface_closing_radius = 0.0;
    geometry.node_spacing_m = 0.3; geometry.wall_clearance_m = 0.0; geometry.step_threshold_m = 0.0;
    SurfaceSnapshot::from_points(&points, geometry, 0.01).unwrap().with_body_envelope(SurfaceBodyEnvelope {
        fixed_collision: None, footprint_xy_m: vec![[-0.3, -width / 2.0], [0.3, -width / 2.0],
            [0.3, width / 2.0], [-0.3, width / 2.0]], z_bounds_m: [-0.15, 0.15],
        body_to_support_z_m: 0.4, max_step_up_m: 0.0, max_step_down_m: 0.0,
        max_cross_slope_deg: 18.0, support_uncertainty_m: 0.0,
    }).unwrap().with_observations(Some(Arc::new(observations)))
}

#[test]
fn c1c3_05_connector_and_forward_arc_keep_branch_navigation() {
    let start = c3_pose(1.05, 1.05, 0.0);
    let target = c3_pose(1.05 + 2.0 * 1.0_f64.sin(), 1.05 + 2.0 * (1.0 - 1.0_f64.cos()), 1.0);
    // These are the exact three previously qualified pillar arrangements.
    for (block_chord, block_arc) in [(true, false), (true, true), (false, true)] {
        let mut points: Vec<_> = (0..40).flat_map(|x| (0..40).map(move |y|
            ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.05))).collect();
        for (x, y) in block_chord.then_some((1.85, 1.55)).into_iter()
            .chain(block_arc.then_some((2.05, 1.25))) {
            for z in 1..8 { points.push((x, y, (z as f32 + 0.5) * 0.1)); }
        }
        let base = c3_body_source();
        let source = SurfaceSnapshot::from_points(&points, base.config().clone(), 0.1).unwrap()
            .with_body_envelope(base.body_envelope().unwrap().clone()).unwrap();
        let actual = body_control::body(&start);
        source.validate_body_pose(actual.0, actual.1).unwrap();
        // A real short initial route establishes the source-owned incoming only.
        // No costly global route around either pillar is prepared.
        let route = c3_route(&PctPlanner::default(), &source, actual, None, [1.10, 1.05]);
        let incoming = route.reference.full_window().expect("real short source window").from().walk_state();
        let key = incoming.semantic_key();
        let query = incoming.bind_body_query(&source).unwrap();
        let physical = query.start_ground_state(actual).unwrap();
        let saved_physical = physical.clone();
        let (chord, chord_walks) = body_control::capture_navigation_walks(||
            body_control::connection_endpoint_with_navigation(&source, &query, actual, &physical, Some(incoming), &target));
        assert_eq!(chord.is_some(), !block_chord);
        assert!(chord_walks > 0);
        let (arc, arc_walks) = body_control::capture_navigation_walks(||
            body_control::forward_arc_connects_with_navigation(&source, &query, actual, &physical, Some(incoming), &target));
        assert_eq!(arc, !block_arc);
        assert!(arc_walks > 0);
        assert_eq!(incoming.semantic_key(), key, "failed chord cannot overwrite the independent arc incoming");
        assert_eq!(physical, saved_physical);
    }
    let source = c3_narrow_door();
    let start = c3_pose(-0.31471258334529195, 0.16213553950286064, 0.05);
    let target = c3_pose(-0.24500000044046072, 0.1649999963119626, 0.0);
    let actual = body_control::body(&start);
    // Backward 1 mm is a source-only finite seed and does not replace the actual
    // unchanged target/alignment control geometry or its sampling thresholds.
    let route = c3_route(&PctPlanner::default(), &source, actual, None,
        [start.pose.position.x - 0.001, start.pose.position.y]);
    let incoming = route.reference.full_window().expect("real door seed window").from().walk_state();
    let key = incoming.semantic_key();
    let query = incoming.bind_body_query(&source).unwrap();
    let physical = query.start_ground_state(actual).unwrap();
    assert!(body_control::connection_endpoint_with_navigation(&source, &query, actual,
        &physical, Some(incoming), &target).is_none());
    let (aligned, walks) = body_control::capture_navigation_walks(||
        body_control::translation_connects_with_navigation(&source, &query, actual, &physical, Some(incoming), &target));
    assert!(aligned, "original door requires the real first-turn-then-translate fallback");
    assert!(walks > 1);
    assert_eq!(incoming.semantic_key(), key);
}

#[test]
fn c1c3_06_delayed_bridge_and_complete_stop_do_not_consume_actual() {
    let (mut owner, _, path, _) = c3_installed_body();
    let committed = owner.committed_ground_basis().unwrap();
    let cursor = owner.ground_cursor();
    let progress = owner.progress_m();
    let moving = Twist2 { linear: Point2::new(0.5, 0.0), angular_z: 0.0 };
    // Real preview goes through normal, delayed and endpoint bridge even where
    // the fixed straight case makes their XY equal. Counts are structural only.
    let horizon = 0.5;
    let count = (moving.linear.x * horizon / (f64::from(owner.source_map.as_ref().unwrap().source_voxel_size_m()) * 0.5))
        .ceil().max(2.0) as usize;
    let (clear, walks) = body_control::capture_navigation_walks(||
        owner.predicted_body_motion_with_basis(moving, horizon, Some(&committed)));
    assert!(clear);
    assert_eq!(walks, 3 * count, "normal, delayed and bridge each use the shared navigation edge");
    assert_eq!(owner.committed_ground_basis(), Some(committed.clone()));
    assert_eq!(owner.ground_cursor(), cursor);
    // Exact original runtime recent-wall point: current actual remains clear,
    // while the future preview and complete stopping tail must both reject it.
    let source = owner.source_map.as_ref().unwrap().clone();
    let blocked = Arc::new(source.with_recent_obstacle_points(2, 2_000_000_000, 0, &[],
        [(0, [1.3, 1.05, 0.4])]).unwrap());
    owner.update_execution_map(ExecutionMap::Surface(blocked)).unwrap();
    owner.observe_applied_command(moving);
    assert!(owner.current_body_error().is_none());
    assert!(!owner.predicted_body_motion_with_basis(moving, horizon, Some(&committed)));
    assert!(!owner.check_selected_candidate_for_test(moving));
    assert_eq!(owner.ground_cursor(), cursor);
    assert_eq!(owner.progress_m(), progress);
    owner.update_execution_map(ExecutionMap::Surface(source)).unwrap();
    owner.observe_applied_command(Twist2::ZERO);
    // Original control-crate two-period/final-discrete-brake fixture, translated
    // to this existing actual origin; real runtime callbacks enforce its boundary.
    let start = PlanarPose { x: 1.05, y: 1.05, yaw: 0.0 };
    let right = start.x + 0.0105;
    owner.boundary = vec![[0.0, 0.0], [right, 0.0], [right, 3.0], [0.0, 3.0]];
    let mut limits = MotionLimits { model: MotionModel::Differential, max_forward_mps: 2.0,
        max_reverse_mps: 1.0, max_lateral_mps: 0.0, max_angular_rps: 2.0,
        linear_acceleration_mps2: 1.0, guaranteed_braking_mps2: 1.0,
        angular_acceleration_rps2: 2.0, guaranteed_angular_braking_rps2: 2.0,
        reaction_time_sec: 0.0, minimum_turning_radius_m: 0.0, can_rotate_in_place: true };
    limits.reaction_time_sec = 0.0;
    let command = Twist2 { linear: Point2::new(0.075, 0.0), angular_z: 0.0 };
    let radius = owner.source_map.as_ref().unwrap().body_envelope().unwrap().radius();
    let check = |periods, clear: &mut dyn FnMut(PlanarPose, PlanarPose, f64) -> bool,
        period: &mut dyn FnMut(PlanarPose, PlanarPose, f64) -> bool| {
        let trace = RefCell::new(Vec::new());
        let result = nuwax_nav_control::check_selected_stopping_trajectory_with_period_check(start,
            command, command, &limits, 0.05, periods, 0.001, radius,
            |a, b, t| { let ok = clear(a, b, t); trace.borrow_mut().push((0, a, b, t, ok)); ok },
            |a, b, t| { let ok = period(a, b, t); trace.borrow_mut().push((1, a, b, t, ok)); ok });
        (result.map(|value| (value.command, value.stopped_pose, value.samples)), trace.into_inner())
    };
    let (cold, hit, cold_failure, hit_failure) = owner.with_body_motion_checks_from_basis(Some(&committed),
        |clear, period, failure, _| {
            assert!(check(1, clear, period).0.is_some(), "one-period positive control");
            let cold = check(2, clear, period);
            let cold_failure = failure.borrow().clone();
            let hit = check(2, clear, period);
            let hit_failure = failure.borrow().clone();
            (cold, hit, cold_failure, hit_failure)
        }).unwrap();
    let (fresh, fresh_failure) = owner.with_body_motion_checks_from_basis(Some(&committed),
        |clear, period, failure, _| { let result = check(2, clear, period); (result, failure.borrow().clone()) }).unwrap();
    assert!(cold.0.is_none(), "full discrete tail reaches 10.625 mm and crosses 10.5 mm boundary");
    assert_eq!(cold, hit);
    assert_eq!(cold, fresh);
    assert!(cold.1.iter().any(|(_, _, to, _, ok)| !ok && to.x > right));
    assert!(cold.1.iter().any(|(kind, _, _, _, _)| *kind == 0));
    assert!(cold.1.iter().any(|(kind, _, _, _, _)| *kind == 1));
    assert!(cold_failure.is_some());
    assert_eq!(cold_failure, hit_failure);
    assert_eq!(cold_failure, fresh_failure);
    assert_eq!(owner.ground_cursor(), cursor);
    owner.boundary.clear();
    // Real selected-command capture, then the same path's new actual observation
    // arrives before consume. Current replay must use a temporary adjacent basis.
    let command = match owner.step_ground_internal(0.2, None).unwrap() {
        ground_cursor::GroundControlStep::Control(PlannerStep::Command(command)) => command,
        _ => panic!("fixed open straight did not select its real control command"),
    };
    assert!(owner.selected_body_command.is_some());
    let before = owner.committed_ground_basis();
    let before_cursor = owner.ground_cursor();
    let before_progress = owner.progress_m();
    let actual = super::interpolate_pose(&path.poses[0], &path.poses[1], 0.25);
    owner.update_odometry(actual.clone());
    let proposed = owner.proposed_ground_basis().unwrap().unwrap();
    assert_ne!(Some(proposed.clone()), before);
    assert_eq!(proposed.walk_state().unwrap().xy(), [actual.pose.position.x, actual.pose.position.y]);
    let (rechecked, walks) = body_control::capture_navigation_walks(||
        owner.recheck_selected_body_command_at_current_state(command));
    assert!(matches!(rechecked, Some(PlannerStep::Command(value)) if value == command));
    assert!(walks > 0);
    assert_eq!(owner.committed_ground_basis(), before);
    assert_eq!(owner.ground_cursor(), before_cursor);
    assert_eq!(owner.progress_m(), before_progress);
}

#[test]
fn c1c3_07_actual_resume_failure_and_cancel_commit_atomically() {
    use std::sync::atomic::AtomicBool;
    let (mut owner, binding, path, _) = c3_installed_body();
    let before = (owner.ground_cursor(), owner.progress_m(), owner.path_index, owner.committed_ground_basis());
    assert!(path.poses[0].pose.position.distance(path.poses[1].pose.position) > 0.0);
    assert_eq!(before.1, 0.0, "the real first connector is still unconsumed");
    owner.update_odometry(super::interpolate_pose(&path.poses[0], &path.poses[1], 0.5));
    let cancel = AtomicBool::new(true);
    assert!(owner.consume_ground_actual(Some(&cancel)).is_err());
    assert_eq!((owner.ground_cursor(), owner.progress_m(), owner.path_index, owner.committed_ground_basis()), before);
    let clear = owner.source_map.as_ref().unwrap().clone();
    let actual = owner.odometry.as_ref().unwrap().clone();
    let blocked = Arc::new(clear.with_recent_obstacle_points(1, 2_000_000_000, 0, &[],
        [(0, [actual.pose.position.x, actual.pose.position.y, actual.pose.position.z])]).unwrap());
    owner.update_execution_map(ExecutionMap::Surface(blocked)).unwrap();
    assert!(owner.consume_ground_actual(None).is_err());
    assert_eq!((owner.ground_cursor(), owner.progress_m(), owner.path_index, owner.committed_ground_basis()), before);
    owner.update_execution_map(ExecutionMap::Surface(clear)).unwrap();
    assert!(matches!(owner.consume_ground_actual(None).unwrap(), ground_cursor::GroundConsumption::Advanced));
    let advanced = (owner.ground_cursor(), owner.progress_m(), owner.path_index, owner.committed_ground_basis());
    assert_ne!(advanced.0, before.0);
    assert!(advanced.1 > 0.0);
    let resume = owner.ground_resume().unwrap();
    let goal = path.poses.last().unwrap().clone();
    owner.admit_ground_candidate(path.clone(), goal.clone(), binding.clone(), Some(resume.clone()), None).unwrap();
    assert_eq!((owner.ground_cursor(), owner.progress_m(), owner.path_index, owner.committed_ground_basis()), advanced);
    assert!(owner.admit_ground_candidate(path.clone(), goal, Arc::new(binding.as_ref().clone()), Some(resume), None).is_err());
    assert_eq!((owner.ground_cursor(), owner.progress_m(), owner.path_index, owner.committed_ground_basis()), advanced);
    assert_eq!(owner.consume_ground_actual(None).unwrap(), ground_cursor::GroundConsumption::Unchanged);
    assert_eq!((owner.ground_cursor(), owner.progress_m(), owner.path_index, owner.committed_ground_basis()), advanced);
}

#[test]
fn c1c3_08_navigation_context_and_selected_basis_retire_together() {
    use crate::replan_feedback::{ActualExecutionContext, GroundActualBasis, SuccessfulReplanMemo, SurfaceReplanInput, same_actual};
    use nuwax_nav_types_messages::GoalOptions;
    let (mut owner, binding, path, route) = c3_installed_body();
    let source = owner.source_map.as_ref().unwrap().clone();
    let pose = owner.odometry.as_ref().unwrap().clone();
    let actual = Arc::new(ActualExecutionContext { source: source.clone(), odometry: pose.clone(),
        ground: source.body_query().start_ground_state(body_control::body(&pose)).unwrap(),
        ground_evidence: None, navigation: owner.committed_ground_basis(), applied: Twist2::ZERO });
    let window = route.reference.full_window().unwrap();
    let goal = GroundRouteGoal { xy: [1.55, 1.05], requested_ground_z: None };
    let input = SurfaceReplanInput { source: source.clone(), start: pose.clone(), goal: crate::ExecutionGoal::Ground(crate::GroundExecutionGoal {frame_id:pose.frame_id,xy:goal.xy,
            yaw:0.0,requested_ground_z:goal.requested_ground_z}),
        options: GoalOptions::default(), boundary: Arc::new(vec![]), actual: Some(actual.clone()),
        ground_intent: Some(GroundIntent::new(window)), limits: SearchLimits::default(),
        offset: 0.3, local_planning_enabled: true, penalties: Arc::new(vec![]) };
    let mut memo = SuccessfulReplanMemo::default();
    memo.published(7, input.clone());
    assert!(memo.should_reuse(Some((&7, &actual)), &input));
    let mut shifted = actual.as_ref().clone();
    shifted.navigation = Some(GroundActualBasis::new(binding.clone(), 0, 0.25,
        [pose.pose.position.x, pose.pose.position.y]));
    assert!(!same_actual(&actual, &shifted), "same pose/applied/physical cannot hide changed original address");
    let mut changed = input.clone();
    changed.actual = Some(Arc::new(shifted.clone()));
    assert!(!input.same_complete_input(&changed));
    assert!(!memo.should_reuse(Some((&7, &shifted)), &changed));
    let mut changed_height = input.clone();
    if let crate::ExecutionGoal::Ground(goal)=&mut changed_height.goal { goal.requested_ground_z=Some(0.0); }
    assert!(!input.same_complete_input(&changed_height));
    assert!(!memo.should_reuse(Some((&7, &actual)), &changed_height));
    // Resolved height is now derived from the immutable original owner, not a
    // second mutable intent field that could contradict its source recording.
    assert_eq!(input.ground_intent.as_ref().unwrap().resolved_ground_z(),
        route.reference.target_sample().and_then(|sample|sample.point.ground_z));
    let connector = window.from().window_to(window.from()).unwrap();
    let mut changed = input.clone();
    changed.ground_intent = Some(GroundIntent::new(connector));
    assert!(!input.same_complete_input(&changed));
    assert!(!memo.should_reuse(Some((&7, &actual)), &changed));
    // Same predicates used by FailedLocalPlan: no private cache state is exposed
    // and this assertion is not labelled an executed failure-cache hit/miss.
    let a = GroundPlanInput { incoming: window.from().continuation([1.05, 1.05]).unwrap(),
        intent: input.ground_intent.clone().unwrap(), xy_reference: Arc::new(vec![[1.05, 1.05], [1.55, 1.05]]),
        arrival_tolerance_m: Some(0.1) };
    let mut b = a.clone(); b.intent = changed.ground_intent.unwrap();
    assert!(!a.same_input(&b));
    b = a.clone(); b.incoming = binding.candidate().continuation_at(0, 0.25, [1.05, 1.05]).unwrap();
    assert_eq!(a.incoming.semantic_key(),b.incoming.semantic_key(),"quarter-edge still uses the original first sample");
    assert!(a.same_input(&b),"same effective incoming is a valid memo equivalence");
    b.incoming = binding.candidate().continuation_at(0, 1.0, [1.05, 1.05]).unwrap();
    assert_ne!(a.incoming.semantic_key(),b.incoming.semantic_key(),"real endpoint is a different evaluator sample");
    assert!(!a.same_input(&b));
    let selected = match owner.step_ground_internal(0.2, None).unwrap() {
        ground_cursor::GroundControlStep::Control(PlannerStep::Command(command)) => command,
        _ => panic!("fixed lifecycle source did not select a real command"),
    };
    assert!(owner.selected_body_command.is_some());
    let old_basis = owner.committed_ground_basis().unwrap();
    let recent = Arc::new(source.with_recent_obstacle_points(3, 3_000_000_000, 0, &[], [(0, [1.3, 1.05, 0.4])]).unwrap());
    owner.update_execution_map(ExecutionMap::Surface(recent)).unwrap();
    assert!(owner.owns_ground_basis(&old_basis), "recent-only retains navigation owner");
    // It must recheck through the current view, never revive an old cached answer.
    let _ = owner.recheck_selected_body_command_at_current_state(selected);
    assert_eq!(owner.committed_ground_basis(), Some(old_basis.clone()));
    owner.stop();
    assert!(owner.ground_cursor().is_none());
    assert!(owner.selected_body_command.is_none());
    assert!(!owner.owns_ground_basis(&old_basis));
    assert!(owner.recheck_selected_body_command(selected).is_none());
    // Real metadata-only publication must preserve the admitted navigation
    // owner even though its current physical query view is replaced.
    let (metadata_source, mut metadata_blocks) = c3_direct_source();
    let (mut metadata_owner, _, _, _) = c3_install(Arc::new(metadata_source), c3_start(),
        [0.1, 0.2], None, c3_config());
    let before_metadata = metadata_owner.committed_ground_basis().unwrap();
    let old_source = metadata_owner.source_map.as_ref().unwrap().clone();
    let mut observations = old_source.observations().unwrap().as_ref().clone();
    for (&key, block) in &mut metadata_blocks {
        for value in block.values_mut() { value.observed_ns = 900; value.hits = 17; }
        assert!(observations.replace_block_with_geometry_changes(key, block.clone()).unwrap().is_empty());
    }
    observations.mark_complete_geometry_source();
    let timed = Arc::new(old_source.with_observations(Some(Arc::new(observations))));
    assert!(nuwax_nav_pct::same_source_geometry(&old_source, &timed));
    metadata_owner.update_execution_map(ExecutionMap::Surface(timed)).unwrap();
    assert!(metadata_owner.owns_ground_basis(&before_metadata));
    assert_eq!(metadata_owner.committed_ground_basis(), Some(before_metadata));
    let (mut owner, _, _, _) = c3_installed_body();
    let old_basis = owner.committed_ground_basis().unwrap();
    let _ = owner.step_ground_internal(0.2, None).unwrap();
    owner.configure_goal(&c3_pose(1.8, 1.05, 0.0), GoalOptions::default()).unwrap();
    assert!(owner.selected_body_command.is_none());
    assert!(!owner.owns_ground_basis(&old_basis));
    let (mut owner, _, _, _) = c3_installed_body();
    let old_basis = owner.committed_ground_basis().unwrap();
    let mut model = owner.source_map.as_ref().unwrap().body_envelope().unwrap().clone();
    model.footprint_xy_m = vec![[-0.11, -0.1], [0.11, -0.1], [0.11, 0.1], [-0.11, 0.1]];
    let changed = Arc::new(owner.source_map.as_ref().unwrap().with_body_envelope(model).unwrap());
    owner.update_execution_map(ExecutionMap::Surface(changed)).unwrap();
    assert!(owner.ground_cursor().is_none());
    assert!(owner.selected_body_command.is_none());
    assert!(!owner.owns_ground_basis(&old_basis));
}

fn c3_known_gap_source() -> (SurfaceSnapshot, f64) {
    // Exact C1-a known_source: two q=.05 native thin columns at [0,0]/[8,0].
    let q = 0.05_f32;
    let mut writer = Writer::new(Repository::temporary().unwrap(), q, 4 * 1024 * 1024).unwrap();
    let mut points = Vec::new();
    let mut height = None;
    for xy in [[0, 0], [8, 0]] {
        let column = HeightfieldColumn::from_counts(xy, q,
            BTreeMap::from([(0, [0, 0, 0, 1, 0, 0, 0, 0])])).unwrap().unwrap();
        height = Some(column.height());
        writer.observe([[xy[0], xy[1], 0]], 20).unwrap();
        writer.replace_ground_column(xy, &ColumnGroundRecords { classified_empty: false, legacy: vec![],
            heightfields: vec![HeightfieldSurface { column, samples: vec![] }] }).unwrap();
        points.push((((f64::from(xy[0]) + 0.5) * f64::from(q)) as f32,
            ((f64::from(xy[1]) + 0.5) * f64::from(q)) as f32, (0.5 * f64::from(q)) as f32));
    }
    let (observations, _) = received_observations(writer, q);
    let config = Config { world_frame: "map".into(), voxel_size: 0.1, robot_height: 0.3,
        surface_closing_radius: 0.0, node_spacing_m: 0.3, wall_clearance_m: 0.0,
        wall_buffer_m: 0.0, wall_buffer_weight: 0.0, max_slope_deg: 20.0,
        step_threshold_m: 0.15, min_step_tread_depth_m: 0.2, step_penalty_weight: 1.0,
        goal_tolerance: 0.1, viz_publish_hz: 0.0 };
    let model = SurfaceBodyEnvelope { fixed_collision: None,
        footprint_xy_m: vec![[-0.01, -0.01], [0.01, -0.01], [0.01, 0.01], [-0.01, 0.01]],
        z_bounds_m: [-0.1, 0.1], body_to_support_z_m: 0.4,
        max_step_up_m: 0.15, max_step_down_m: 0.15, max_cross_slope_deg: 20.0, support_uncertainty_m: 0.0 };
    (SurfaceSnapshot::from_points(&points, config, q).unwrap().with_body_envelope(model).unwrap()
        .with_observations(Some(Arc::new(observations))), height.unwrap())
}

#[test]
fn c1c3_09_unknown_bias_and_reference_end_never_claim_arrival() {
    use nuwax_nav_pct::ground_route::PlannedGroundAssociation;
    let (source, height) = c3_known_gap_source();
    let q = f64::from(0.05_f32);
    let xy = [0.5 * q, 0.5 * q];
    let to = [8.5 * q, 0.5 * q];
    let attitude = [(0.1_f64 / 2.0).sin(), 0.0, 0.0, (0.1_f64 / 2.0).cos()];
    let actual = ([xy[0], xy[1], height + 0.46], attitude);
    let (mut owner, binding, path, _) = c3_install(Arc::new(source), actual, to, Some(height), c3_config());
    let mut kinds = Vec::new();
    for edge in 0..binding.candidate().connection_count() {
        let steps = binding.candidate().connection(edge).unwrap().2;
        for i in 0..=steps {
            let sample = binding.candidate().sample(edge, i).unwrap();
            assert_eq!(sample.point.ground_z, Some(height));
            kinds.push(match sample.association { PlannedGroundAssociation::Known { .. } => 0,
                PlannedGroundAssociation::Carried => 1, PlannedGroundAssociation::Unanchored => 2 });
        }
    }
    assert_eq!(kinds.first(), Some(&0));
    assert_eq!(kinds.last(), Some(&0));
    assert!(kinds.contains(&1));
    assert!(!kinds.contains(&2));
    assert!(path.poses.iter().all(|p| (p.pose.position.z - actual.0[2]).abs() < 1e-12));
    for pose in path.poses.iter().skip(1) { owner.update_odometry(pose.clone()); owner.consume_ground_actual(None).unwrap(); }
    assert!(matches!(owner.step_ground_internal(0.02, None).unwrap(), ground_cursor::GroundControlStep::LocalReferenceEnd { .. }));
    let (source, _) = c3_direct_source();
    let actual = ([100.0, 100.0, 0.55], [0.0, 0.0, 0.0, 1.0]);
    let (mut owner, binding, path, route) = c3_install(Arc::new(source), actual, [100.1, 100.0], None, c3_config());
    assert!(path.poses.iter().all(|pose| pose.pose.position.z == actual.0[2]));
    for edge in 0..binding.candidate().connection_count() {
        let steps = binding.candidate().connection(edge).unwrap().2;
        for i in 0..=steps { assert_eq!(binding.candidate().sample(edge, i).unwrap().point.ground_z, None); }
    }
    for pose in path.poses.iter().skip(1) { owner.update_odometry(pose.clone()); owner.consume_ground_actual(None).unwrap(); }
    assert!(matches!(owner.step_ground_internal(0.2, None).unwrap(), ground_cursor::GroundControlStep::LocalReferenceEnd { .. }));
    // Real zero-motion Scan result remains neutral; no fake edge consumption or
    // physical body-Z backfill may convert it into the public Succeeded variant.
    let source = owner.source_map.as_ref().unwrap().clone();
    let physical = source.body_query().start_ground_state(actual).unwrap();
    let zero = nuwax_nav_scan::plan_driven_ground(&source, &Tomogram::from_surface(&source).unwrap(), actual,
        &physical, &[[100.0, 100.0]], &c3_motion(), &ScanConfig::default(), None,
        Some(([100.0, 100.0], 0.0)), &[], false, false, None, route.reference.full_window().unwrap()).unwrap();
    assert_eq!(zero.trajectory.poses, vec![actual]);
    assert!(zero.binding.edges().is_empty());
    assert_eq!(zero.binding.candidate().connection_count(), 0);
    assert_eq!(zero.binding.candidate().walk_state_at(0, 0.0, [100.0, 100.0]).unwrap().planned_ground_z(), None);
    let binding = Arc::new(zero.binding);
    let path = Path3D { poses: zero.trajectory.poses.into_iter().map(c3_stamped).collect() };
    owner.stop(); owner.update_odometry(c3_stamped(actual));
    assert_eq!(binding.stationary_pose(),Some(actual));
    // Same source and clear geometry cannot substitute another winner's pose.
    for shifted in [([actual.0[0]+0.01,actual.0[1],actual.0[2]],actual.1),
        ([actual.0[0],actual.0[1],actual.0[2]+0.01],actual.1)] {
        let other=c3_stamped(shifted);owner.update_odometry(other.clone());
        assert!(owner.admit_ground_candidate(Path3D {poses:vec![other.clone()]},other,
            binding.clone(),None,None).is_err());
        assert!(owner.ground_cursor().is_none());
    }
    owner.update_odometry(c3_stamped(actual));
    owner.admit_ground_candidate(path.clone(), path.poses.last().unwrap().clone(), binding, None, None).unwrap();
    let before = owner.ground_cursor();
    assert!(matches!(owner.step_ground_internal(0.2, None).unwrap(), ground_cursor::GroundControlStep::LocalReferenceEnd { .. }));
    assert_eq!(owner.ground_cursor(), before, "zero motion must not fabricate consumed progress");
}

#[test]
fn c1c3_10_body_control_keeps_original_queries_and_admission() {
    let source = c3_body_source();
    let actual = body_control::body(&c3_pose(1.05, 1.05, 0.0));
    let to = [1.15, 1.05];
    let blocked = source.with_recent_obstacle_points(4, 4_000_000_000, 0, &[], [(0, [1.15, 1.05, 0.4])]).unwrap();
    for map in [&source, &blocked] {
        let query = map.body_query();
        let ground = query.start_ground_state(actual).unwrap();
        let mut witness = None;
        let original = query.project_motion(actual, &ground, to, 0.0, &mut witness).and_then(|projection| {
            query.validate_projected_motion(actual, &projection, &[], &mut witness)?;
            Ok(projection)
        });
        let (current, walks) = body_control::capture_navigation_walks(|| body_control::control_project_edge(
            &query, actual, &ground, None, to, 0.0, &[], body_control::ControlEdgeProfile::Preview));
        assert_eq!(walks, 0);
        match (original, current) {
            (Ok(original), Ok(current)) => {
                assert_eq!(current.body.pose, original.pose());
                assert_eq!(current.body.ground.as_ref(), Some(original.ground()));
                assert!(current.navigation.is_none());
            }
            (Err(error), Err(body_control::BodyMotionFailure::Body(current))) => {
                assert_eq!(Some(current), witness);
                assert_eq!(error, witness.unwrap().rejection.error());
            }
            (Err(error), Err(body_control::BodyMotionFailure::Legacy(current))) => {
                assert!(witness.is_none()); assert_eq!(error, current);
            }
            _ => panic!("Body None changed original projection/sweep result"),
        }
    }
    // The normal Body path still admits a result that begins behind moving
    // actual, via the original bounded-nearest connector rather than strict start.
    let (mut body_owner, _, path, _) = c3_installed_body();
    body_owner.stop();
    let actual = super::interpolate_pose(&path.poses[0], &path.poses[1], 0.5);
    assert_ne!(actual.pose, path.poses[0].pose);
    body_owner.update_odometry(actual);
    let (admitted, walks) = body_control::capture_navigation_walks(|| body_owner.replace_execution_path(
        ExecutionPath::Surface(path.clone()), None, path.poses.last().unwrap().clone()));
    admitted.unwrap();
    assert_eq!(walks, 0);
    assert!(body_owner.ground_execution.is_none());
    assert!(body_owner.ground_cursor().is_none());
    let (decision, walks) = body_control::capture_navigation_walks(|| body_owner.follow_body_step(0.2));
    assert!(matches!(decision, PlannerStep::Command(_)));
    assert_eq!(walks, 0);
}

// C1-c4 runtime draft. Append to the EXISTING ground_owner_tests.rs module.
// All original c3_* fixtures, Writer/Receiver cleanup and owner gates are reused.
// Fixed C1-c4 qualification inputs; first execution is recorded in the stage evidence.
// Uses the exact rolling facade and scoped cancellation hooks supplied by the implementation.
// The six #[test] names below are exactly R3--R8 from the frozen selector list.

fn c4_first_motion(path: &Path3D) -> usize {
    path.poses.windows(2).position(|p| {
        p[0].pose.position.x != p[1].pose.position.x
            || p[0].pose.position.y != p[1].pose.position.y
    }).expect("fixed original candidate must contain a nonzero XY edge")
}

fn c4_fractional_actual(path: &Path3D, edge: usize, fraction: f64, lateral: f64, dz: f64) -> PoseStamped {
    let mut actual = super::interpolate_pose(&path.poses[edge], &path.poses[edge + 1], fraction);
    actual.pose.position.y += lateral;
    actual.pose.position.z += dz;
    actual
}

// Call the actual private selection kernel; do not re-infer its address from s.
fn c4_selected(owner: &SurfaceFollower, path: &Path3D, actual: &PoseStamped)
    -> (usize, f64, PoseStamped) {
    let mut cumulative = vec![0.0];
    for pair in path.poses.windows(2) {
        cumulative.push(cumulative.last().unwrap() +
            (pair[1].pose.position.x - pair[0].pose.position.x)
                .hypot(pair[1].pose.position.y - pair[0].pose.position.y));
    }
    let selected = super::closest_in_window_with_address::<true>(path, &cumulative,
        actual.pose.position, 0.0, owner.config.max_path_deviation_m);
    (selected.index - 1, selected.fraction, selected.pose)
}
fn c4_assert_half_cut(expected_edge: usize, edge: usize, fraction: f64) {
    assert_eq!(edge, expected_edge);
    assert!(fraction > 0.0 && fraction < 1.0, "a real fractional original edge must be selected");
    assert!((fraction - 0.5).abs() < 1e-12, "fixed midpoint selects {fraction}, not a different input");
}

fn c4_binding(owner: &SurfaceFollower) -> Arc<GroundCandidateBinding> {
    owner.ground_binding_for_test().expect("installed execution retains its one binding")
}

fn c4_retained_start(binding: &GroundCandidateBinding) -> usize {
    let prefix = binding.edges().iter().take_while(|edge| {
        edge.origin() == nuwax_nav_scan::CandidateEdgeOrigin::AdmissionConnector
    }).count();
    assert!(prefix > 0, "rolling install must contain its actual connector recording");
    assert!(prefix < binding.edges().len(), "rolling install must retain the candidate suffix");
    assert!(binding.candidate().is_splice_boundary(prefix));
    prefix
}

// Compare the exact original address and original sample semantics, while
// giving BOTH sides the same actual XY. Actual XY is not a navigation sample.
fn c4_assert_original_suffix(
    rolling: &GroundCandidateBinding, first: usize,
    original: &GroundCandidateBinding, edge: usize, fraction: f64,
) {
    assert_eq!(rolling.family(), original.family());
    assert_eq!(rolling.planned_join(), original.planned_join());
    assert_eq!(rolling.covers_original_goal(), original.covers_original_goal());
    let (leaf, range) = original.candidate().original_range(edge).unwrap();
    let (index, remainder) = original.candidate().original_sample(edge, fraction).unwrap();
    let steps = original.candidate().connection(edge).unwrap().2;
    let original_cut = if remainder == 0.0 { index as f64 / steps as f64 }
        else { range[0] + fraction * (range[1] - range[0]) };
    assert_eq!(rolling.candidate().original_range(first), Some((leaf, [original_cut, range[1]])));
    assert_eq!(rolling.edges().len() - first, original.edges().len() - edge);
    for (offset, retained) in (first..rolling.edges().len()).enumerate() {
        let old = edge + offset;
        assert_eq!(rolling.edges()[retained].origin(), original.edges()[old].origin());
        assert_eq!(rolling.candidate().connection(retained), original.candidate().connection(old));
        for local in [0.0, 0.25, 0.5, 1.0] {
            // Compare the shared original-slice decoder, never affine-map an
            // event back to t and risk moving it to the preceding original i.
            let start = if offset == 0 { fraction } else { 0.0 };
            let decoded = original.candidate().original_slice_sample(old, start, 1.0, local).unwrap();
            assert_eq!(rolling.candidate().original_sample(retained, local), Some(decoded));
            let xy = [0.125, 0.225]; // same actual coordinates, no physical evaluation
            let kept = rolling.candidate().walk_state_at(retained, local, xy).unwrap();
            let prior = original.candidate().sample(old, decoded.0).unwrap();
            let sample = rolling.candidate().sample(retained, decoded.0).unwrap();
            assert!(kept.same_reference_owner(original.original_window().owner()));
            assert_eq!(kept.reference_sample_xy(), prior.point.xy);
            assert_eq!(kept.planned_ground_z(), prior.point.ground_z);
            assert_eq!(sample.point.xy, prior.point.xy);
            assert_eq!(sample.point.ground_z, prior.point.ground_z);
            use nuwax_nav_pct::ground_route::PlannedGroundAssociation as A;
            match (sample.association, prior.association) {
                (A::Known { source: Some(a) }, A::Known { source: Some(b) }) => assert_eq!(a.key(), b.key()),
                (A::Known { source: None }, A::Known { source: None }) |
                (A::Carried, A::Carried) | (A::Unanchored, A::Unanchored) => (),
                _ => panic!("retained original association changed"),
            }
        }
    }
}

#[derive(Debug, PartialEq)]
struct C4ExecutionSnapshot {
    path: Option<Path3D>, goal: Option<PoseStamped>, cursor: Option<ground_cursor::GroundCursor>,
    progress: f64, target: usize, basis: Option<crate::replan_feedback::GroundActualBasis>,
    tracked: Option<PoseStamped>, physical: Option<(Pose3, Result<BodyGroundState, String>)>,
    alignment: Option<usize>, selected: bool,
}
fn c4_execution(owner: &SurfaceFollower) -> C4ExecutionSnapshot {
    C4ExecutionSnapshot { path: owner.path.clone(), goal: owner.goal.clone(),
        cursor: owner.ground_cursor(), progress: owner.progress_m(), target: owner.path_index,
        basis: owner.committed_ground_basis(), tracked: owner.last_tracked_pose.clone(),
        physical: owner.actual_ground.clone(), alignment: owner.alignment_target_index,
        selected: owner.selected_body_command.is_some() }
}
fn c4_dispatch_proof(owner: &mut SurfaceFollower) {
    owner.last_command_dispatch_bound = Some(crate::CommandDispatchBound::from_anchor(
        crate::command_dispatch::ObservationClockAnchor { monotonic: Instant::now(), unix_ns: 100 }, 200));
    assert!(owner.last_command_dispatch_bound.is_some());
}

#[test]
fn c1c4_03_real_planning_owner_admits_moving_replacement() {
    let (source, _) = c3_direct_source();
    let source = Arc::new(source);
    let (mut actual_owner, old_binding, old_path, route) = c3_install(
        source.clone(), c3_start(), [0.1, 0.2], None, c3_config());
    let old_edge = c4_first_motion(&old_path);
    let mut kernel = PlanKernel::new(c3_config());
    let (started_tx, started_rx) = mpsc::channel();
    let (compute_tx, compute_rx) = mpsc::channel();
    let (computed_tx, computed_rx) = mpsc::channel();
    let (publish_tx, publish_rx) = mpsc::channel();
    let owner = PlanningOwner::new(move |request: PlanRequest, cancel: &Cancellation| {
        started_tx.send(request.generation).unwrap();
        c3_recv(&compute_rx);
        let result = kernel.compute(request, cancel);
        computed_tx.send(result.outcome.as_ref().map(|candidate| candidate.path.clone())
            .map_err(|error| format!("{error:?}"))).unwrap();
        c3_recv(&publish_rx);
        result
    }).unwrap();
    // Same original cleanup owns both gates, including every panic exit.
    let cleanup = C3GateCleanup { compute: compute_tx.clone(), publish: publish_tx.clone() };
    assert!(owner.submit(1, c3_request(source, &route, 1)));
    assert_eq!(c3_recv(&started_rx), 1);
    let quarter = c4_fractional_actual(&old_path, old_edge, 0.25, 0.0, 0.0);
    actual_owner.update_odometry(quarter);
    assert_eq!(actual_owner.consume_ground_actual(None).unwrap(), ground_cursor::GroundConsumption::Advanced);
    let consumed_old = c4_execution(&actual_owner);
    assert!(consumed_old.progress > 0.0);
    assert!(actual_owner.owns_ground_basis(consumed_old.basis.as_ref().unwrap()));
    compute_tx.send(()).unwrap();
    let planned = c3_recv(&computed_rx).expect("unchanged finite Scan256 real kernel prerequisite");
    assert!(owner.take().is_none(), "publication gate is still closed");
    assert_eq!(planned.poses[0].pose, c3_stamped(c3_start()).pose);
    let edge = c4_first_motion(&planned);
    let actual = c4_fractional_actual(&planned, edge, 0.5, 0.01, 0.0);
    assert_ne!(actual.pose, planned.poses[0].pose);
    actual_owner.update_odometry(actual.clone());
    assert_eq!(actual_owner.ground_cursor(), consumed_old.cursor);
    publish_tx.send(()).unwrap();
    c3_until(|| owner.completion_ready_for_test());
    let completion = owner.take().expect("real ungated completion");
    assert_eq!(completion.id, 1);
    assert_eq!(completion.epoch, owner.epoch());
    let candidate = completion.value.outcome.expect("real kernel outcome, no candidate substitute");
    let original = candidate.ground_binding.expect("Ground worker must carry the real Scan winner");
    assert_eq!(original.family(), CandidateFamily::Direct);
    let (cut_edge, cut_fraction, join) = c4_selected(&actual_owner, &candidate.path, &actual);
    c4_assert_half_cut(edge, cut_edge, cut_fraction);
    let old_basis = consumed_old.basis.unwrap();
    actual_owner.admit_ground_candidate(candidate.path.clone(), candidate.path.poses.last().unwrap().clone(),
        original.clone(), None, None).expect("fixed moving Ground admission prerequisite");
    let rolling = c4_binding(&actual_owner);
    let installed = actual_owner.path.as_ref().unwrap().clone();
    let first = c4_retained_start(&rolling);
    assert_eq!(installed.poses[0].pose, actual.pose);
    assert!(installed.poses[..=first].windows(2).any(|p|
        p[0].pose.position.distance(p[1].pose.position) > 0.0), "real connector is nonzero");
    assert!((installed.poses[first].pose.position.x - join.pose.position.x).abs() < 1e-12);
    assert!((installed.poses[first].pose.position.y - join.pose.position.y).abs() < 1e-12);
    c4_assert_original_suffix(&rolling, first, &original, cut_edge, cut_fraction);
    assert!(!Arc::ptr_eq(&rolling, &old_binding));
    assert!(!actual_owner.owns_ground_basis(&old_basis));
    assert_eq!(actual_owner.ground_cursor(), Some(ground_cursor::GroundCursor { edge: 0, fraction: 0.0 }));
    assert_eq!(actual_owner.progress_m(), 0.0, "discarded candidate prefix is not consumed travel");
    let installed_state = c4_execution(&actual_owner);
    assert_eq!(actual_owner.consume_ground_actual(None).unwrap(), ground_cursor::GroundConsumption::Unchanged);
    assert_eq!(c4_execution(&actual_owner), installed_state);
    let observed = c4_fractional_actual(&installed, 0, 0.5, 0.0, 0.0);
    actual_owner.update_odometry(observed);
    assert_eq!(actual_owner.consume_ground_actual(None).unwrap(), ground_cursor::GroundConsumption::Advanced);
    assert!(actual_owner.progress_m() > 0.0);
    drop(cleanup);
    drop(owner);
}

#[test]
fn c1c4_04_connector_keeps_actual_bias_and_retained_suffix() {
    use nuwax_nav_pct::ground_route::PlannedGroundAssociation;
    let (source, height) = c3_known_gap_source();
    let source = Arc::new(source);
    let q = f64::from(0.05_f32);
    let attitude = [(0.1_f64 / 2.0).sin(), 0.0, 0.0, (0.1_f64 / 2.0).cos()];
    let start = ([0.5 * q, 0.5 * q, height + 0.46], attitude);
    let (mut owner, original, path, _) = c3_install(source.clone(), start,
        [8.5 * q, 0.5 * q], Some(height), c3_config());
    let edge = c4_first_motion(&path);
    let mut actual = c4_fractional_actual(&path, edge, 0.5, 0.0, 0.01);
    // Interpolation is an XY/yaw hint. The observation really retains roll=.1.
    actual.pose.orientation = c3_stamped(start).pose.orientation;
    let planned_cut = super::interpolate_pose(&path.poses[edge], &path.poses[edge + 1], 0.5);
    assert!((actual.pose.position.z - planned_cut.pose.position.z - 0.01).abs() < 1e-12);
    let (cut_edge, cut_fraction, selected) = c4_selected(&owner, &path, &actual);
    c4_assert_half_cut(edge, cut_edge, cut_fraction);
    owner.update_odometry(actual.clone());
    owner.refresh_current_body_view();
    let incoming_basis = owner.proposed_ground_basis().unwrap().unwrap();
    let incoming = incoming_basis.walk_state().unwrap();
    let physical = owner.current_ground_query_owned().unwrap().1;
    let query = incoming.bind_body_query(&source).unwrap();
    // Independent physical oracle on the same fixed input. This is not the
    // shipping admission implementation recording its connector a second time.
    let (oracle_connector, oracle_ground) = body_control::record_ground_connector(&source, &query,
        body_control::body(&actual), &physical, incoming, &selected, &[], None, || true)
        .expect("original physical connector prerequisite");
    let oracle_end = *oracle_connector.poses.last().unwrap();
    owner.admit_ground_candidate(path.clone(), path.poses.last().unwrap().clone(), original.clone(), None, None)
        .expect("fixed q=.05 known-gap biased midpoint prerequisite; do not alter source/model");
    let rolling = c4_binding(&owner);
    let installed = owner.path.as_ref().unwrap();
    let first = c4_retained_start(&rolling);
    c4_assert_original_suffix(&rolling, first, &original, cut_edge, cut_fraction);
    let p = body_control::body(&installed.poses[first]);
    assert_eq!(p, oracle_end, "installed connector uses the original complete physical response");
    assert!((p.0[2] - actual.pose.position.z).abs() < 1e-12);
    assert_ne!(p, body_control::body(&planned_cut), "connector must retain observed full-pose bias");
    assert!(p.1[0].abs() > 0.01, "actual roll was not reset to the planned yaw-only hint");
    let physical = rolling.candidate().with_body_navigation_seam(first, [p.0[0], p.0[1]], |seam|
        query.validate_motion_with_navigation(p, p, &oracle_ground, seam, &[], &mut None)).unwrap().unwrap();
    let next = &path.poses[cut_edge + 1];
    let xy = [next.pose.position.x, next.pose.position.y];
    let (first_projection, walks) = body_control::capture_navigation_walks(||
        rolling.candidate().with_body_navigation_slice(first, 0.0, 1.0, [p.0[0], p.0[1]], xy, |slice| {
            let projected = query.project_motion_with_navigation(p, &physical, xy,
                next.pose.orientation.yaw(), slice, &mut None).unwrap();
            query.validate_projected_motion(p, &projected, &[], &mut None).unwrap();
            projected
        }).unwrap());
    assert_eq!(walks, 0, "retained slice uses recorded samples, not control_project_edge re-walk");
    assert_ne!(first_projection.pose(), body_control::body(next));
    assert!((first_projection.pose().0[2] - next.pose.position.z - 0.01).abs() < 1e-12);
    let kinds: Vec<_> = (0..original.candidate().connection_count()).flat_map(|e| {
        (0..=original.candidate().connection(e).unwrap().2).map(move |i| (e, i))
    }).map(|(e, i)| match original.candidate().sample(e, i).unwrap().association {
        PlannedGroundAssociation::Known { .. } => 0,
        PlannedGroundAssociation::Carried => 1,
        PlannedGroundAssociation::Unanchored => 2,
    }).collect();
    assert_eq!(kinds.first(), Some(&0));
    assert_eq!(kinds.last(), Some(&0));
    assert!(kinds.contains(&1));
    assert!(!kinds.contains(&2));
    assert_eq!(owner.progress_m(), 0.0);
    // Original all-Unknown input, with no body-Z backfill of navigation height.
    let (unknown_source, _) = c3_direct_source();
    let initial = ([100.0, 100.0, 0.55], [0.0, 0.0, 0.0, 1.0]);
    let (mut unknown, original, path, _) = c3_install(Arc::new(unknown_source), initial,
        [100.1, 100.0], None, c3_config());
    let edge = c4_first_motion(&path);
    unknown.update_odometry(c4_fractional_actual(&path, edge, 0.5, 0.0, 0.0));
    unknown.admit_ground_candidate(path.clone(), path.poses.last().unwrap().clone(), original, None, None)
        .expect("original all-Unknown rolling continuation must use the existing carried physical kernel");
    let rolling = c4_binding(&unknown);
    for e in 0..rolling.candidate().connection_count() {
        for i in 0..=rolling.candidate().connection(e).unwrap().2 {
            assert_eq!(rolling.candidate().sample(e, i).unwrap().point.ground_z, None);
        }
    }
    assert_eq!(unknown.progress_m(), 0.0);
}

fn c4_navigation_sample<'a>(candidate: &'a nuwax_nav_pct::ground_route::GroundCandidateReference,
    edge: usize, t: f64) -> nuwax_nav_mls_planner::surface::NavigationGroundSample<'a> {
    use nuwax_nav_mls_planner::surface::NavigationGroundSample as N;
    use nuwax_nav_pct::ground_route::PlannedGroundAssociation as A;
    let index = candidate.original_sample(edge, t).unwrap().0;
    let sample = candidate.sample(edge, index).unwrap();
    match sample.association {
        A::Known { source: Some(record) } => N::KnownNative { record, reference_xy: sample.point.xy },
        A::Known { source: None } => N::KnownLegacy { reference_xy: sample.point.xy,
            height: sample.point.ground_z.expect("original Known sample retains height") },
        A::Carried => N::Carried { height: sample.point.ground_z.expect("original carried height") },
        A::Unanchored => N::Unanchored,
    }
}
fn c4_compare_seam(candidate: &nuwax_nav_pct::ground_route::GroundCandidateReference,
    right: usize, source: &SurfaceSnapshot, actual: BodyPose, require_clear: bool) {
    use nuwax_nav_mls_planner::surface::BodyNavigationSegment;
    let xy = [actual.0[0], actual.0[1]];
    let query = candidate.walk_state_at(right - 1, 1.0, xy).unwrap().bind_body_query(source).unwrap();
    let physical = query.start_ground_state(actual).unwrap();
    let before = physical.clone();
    let decode = |t| c4_navigation_sample(candidate, if t < 1.0 { right - 1 } else { right },
        if t < 1.0 { 1.0 } else { 0.0 });
    let changes = [1.0];
    let explicit = BodyNavigationSegment::from_recorded(source, xy, xy, &changes, &decode);
    let mut direct_witness = None;
    let direct = query.validate_motion_with_navigation(actual, actual, &physical, &explicit, &[], &mut direct_witness);
    let mut adapter_witness = None;
    let adapter = candidate.with_body_navigation_seam(right, xy, |segment|
        query.validate_motion_with_navigation(actual, actual, &physical, segment, &[], &mut adapter_witness)).unwrap();
    assert_eq!(adapter, direct, "adapter must preserve the original complete H3 result, including exact error");
    assert_eq!(adapter_witness, direct_witness, "no any-Err substitute for the real collision witness");
    assert_eq!(physical, before, "join preparation never mutates supplied physical state");
    if require_clear { direct.expect("fixed physical seam prerequisite"); }
}

#[test]
fn c1c4_05_join_uses_original_h3_without_unknown_gate() {
    let (source, _) = c3_direct_source();
    let route = c3_route(&PctPlanner::default(), &source, c3_start(), None, [0.1, 0.2]);
    let mapped = c3_direct(&source, &route);
    let candidate = mapped.binding.candidate();
    let edge = mapped.binding.edges().iter().position(|e| e.motion()[0].0 != e.motion()[1].0).unwrap();
    let motion = mapped.binding.edges()[edge].motion();
    let cut = std::array::from_fn(|axis| motion[0].0[axis] + 0.25 * (motion[1].0[axis] - motion[0].0[axis]));
    let xy = [cut[0], cut[1]];
    let incoming = route.reference.full_window().unwrap().from().walk_state();
    let prefix = route.reference.freeze_candidate_from(incoming, vec![incoming.walk_xy(xy, &[], None).unwrap()]).unwrap();
    let joined = candidate.prepend_to_suffix(&prefix, edge, 0.25).unwrap();
    let right = prefix.connection_count();
    let left_state = joined.walk_state_at(right - 1, 1.0, xy).unwrap();
    let right_state = joined.walk_state_at(right, 0.0, xy).unwrap();
    assert_ne!(left_state.reference_sample_xy(), right_state.reference_sample_xy());
    assert_ne!(left_state.semantic_key(), right_state.semantic_key());
    assert_eq!(left_state.planned_ground_z(), right_state.planned_ground_z());
    c4_compare_seam(&joined, right, &source, (cut, motion[0].1), true);

    // Existing real dual-association source; no new two-layer/tilted fixture.
    let source = c3_bin_shift_source();
    let xy = [0.025, 0.025];
    let to = [0.125, 0.025];
    let actual = ([xy[0], xy[1], 0.705], [0.0, 0.0, 0.0, 1.0]);
    let records: Vec<_> = source.observations().unwrap().ground_records([0, 0]).collect();
    assert_eq!(records.len(), 2);
    let planner = PctPlanner::default();
    let lower = c3_route(&planner, &source, actual, records[0].height_at(xy), to);
    let upper = c3_route(&planner, &source, actual, records[1].height_at(xy), to);
    let a = lower.reference.full_window().unwrap().from().walk_state();
    let b = upper.reference.full_window().unwrap().from().walk_state();
    assert_ne!(a.semantic_key(), b.semantic_key());
    let query = a.bind_body_query(&source).unwrap();
    let ground = query.start_ground_state(actual).unwrap();
    let low_response = c3_project(&query, a, actual, &ground, to);
    let high_response = c3_project(&query, b, actual, &ground, to);
    assert_ne!(low_response.pose(), high_response.pose(), "the original dual source must retain distinct physical responses");
    let left = lower.reference.freeze_candidate_from(a, vec![a.walk_xy(xy, &[], None).unwrap()]).unwrap();
    let right = lower.reference.freeze_candidate_from(b, vec![b.walk_xy(to, &[], None).unwrap()]).unwrap();
    let joined = right.prepend_to_suffix(&left, 0, 0.0).unwrap();
    c4_compare_seam(&joined, left.connection_count(), &source, actual, false);
    let foreign = c3_bin_shift_source();
    assert!(a.bind_body_query(&foreign).is_err(), "equal-looking geometry does not replace the source owner");

    // Original unknown-gap recording: select an actually recorded Carried edge,
    // not an invented Reference id or a physical-height reconstruction.
    let (source, height) = c3_known_gap_source();
    let q = f64::from(0.05_f32);
    let actual = ([0.5 * q, 0.5 * q, height + 0.46],
        [(0.1_f64 / 2.0).sin(), 0.0, 0.0, (0.1_f64 / 2.0).cos()]);
    let (_, binding, path, route) = c3_install(Arc::new(source.clone()), actual,
        [8.5 * q, 0.5 * q], Some(height), c3_config());
    let edge = (0..binding.candidate().connection_count()).find(|&e|
        matches!(binding.candidate().sample(e, 0).unwrap().association,
            nuwax_nav_pct::ground_route::PlannedGroundAssociation::Carried))
        .expect("original known-gap recording contains a carried start sample");
    let actual = body_control::body(&path.poses[edge]);
    let xy = [actual.0[0], actual.0[1]];
    let incoming = binding.candidate().walk_state_at(edge, 0.0, xy).unwrap();
    let prefix = route.reference.freeze_candidate_from(incoming,
        vec![incoming.walk_xy(xy, &[], None).unwrap()]).unwrap();
    let joined = binding.candidate().prepend_to_suffix(&prefix, edge, 0.0).unwrap();
    c4_compare_seam(&joined, prefix.connection_count(), &source, actual, true);
}

#[test]
fn c1c4_06_failed_cancelled_rolling_admission_preserves_execution() {
    use std::sync::atomic::{AtomicBool, Ordering};
    // Three deterministic checkpoints, all through the original atomic flag.
    // With the fixed .01m lateral connector, count==1. Its connector checkpoint
    // is exercised, but this does NOT claim cancellation after a completed step.
    for stage in ["entry", "connector", "commit"] {
        let (source, _) = c3_direct_source();
        let (mut owner, binding, path, _) = c3_install(Arc::new(source), c3_start(), [0.1, 0.2], None, c3_config());
        let edge = c4_first_motion(&path);
        let actual = c4_fractional_actual(&path, edge, 0.5, 0.01, 0.0);
        let (cut_edge, cut_fraction, join) = c4_selected(&owner, &path, &actual);
        c4_assert_half_cut(edge, cut_edge, cut_fraction);
        let map = owner.source_map.as_ref().unwrap();
        let distance = (actual.pose.position.x - join.pose.position.x)
            .hypot(actual.pose.position.y - join.pose.position.y);
        let yaw = angle_diff(join.pose.orientation.yaw(), actual.pose.orientation.yaw()).abs();
        let count = ((distance + map.body_envelope().unwrap().radius() * yaw)
            / (f64::from(map.source_voxel_size_m()) * 0.5)).max(yaw / 0.05).ceil().max(1.0) as usize;
        assert_eq!(count, 1, "fixed connector is one substep; do not silently enlarge its lateral offset");
        owner.update_odometry(actual);
        owner.refresh_current_body_view();
        let before = c4_execution(&owner);
        let old_binding = c4_binding(&owner);
        c4_dispatch_proof(&mut owner);
        let flag = Arc::new(AtomicBool::new(stage == "entry"));
        let result = if stage == "entry" {
            owner.admit_ground_candidate(path.clone(), path.poses.last().unwrap().clone(),
                binding.clone(), None, Some(flag.as_ref()))
        } else {
            ground_cursor::cancel_ground_admission_at_for_test(stage, 1, flag.clone(), ||
                owner.admit_ground_candidate(path.clone(), path.poses.last().unwrap().clone(),
                    binding.clone(), None, Some(flag.as_ref())))
        };
        assert!(flag.load(Ordering::Relaxed), "the real {stage} checkpoint must actually be reached");
        let error = result.expect_err("cancelled admission must not install its scratch result");
        assert!(error.contains("cancel"), "checkpoint returned an unrelated error: {error}");
        assert_eq!(c4_execution(&owner), before, "{stage} cancellation changed committed execution");
        assert!(Arc::ptr_eq(&c4_binding(&owner), &old_binding));
        assert_eq!(owner.take_command_dispatch_bound(), None, "failed admission invalidates its old dispatch proof");
    }
    // Original recent point, unchanged. It lies in the tiny connector's current
    // body footprint too; this is current-view rejection + atomicity coverage,
    // not an isolated connector-interior obstacle claim.
    let (source, _) = c3_direct_source();
    let (mut owner, binding, path, _) = c3_install(Arc::new(source), c3_start(), [0.1, 0.2], None, c3_config());
    let edge = c4_first_motion(&path);
    owner.update_odometry(c4_fractional_actual(&path, edge, 0.5, 0.01, 0.0));
    let clear = owner.source_map.as_ref().unwrap().clone();
    let recent = Arc::new(clear.with_recent_obstacle_points(1, 1_000_000_000, 0, &[],
        [(0, [-0.95, 0.2, 0.4])]).unwrap());
    owner.update_execution_map(ExecutionMap::Surface(recent)).unwrap();
    owner.refresh_current_body_view();
    let before = c4_execution(&owner);
    c4_dispatch_proof(&mut owner);
    assert!(owner.admit_ground_candidate(path.clone(), path.poses.last().unwrap().clone(), binding, None, None).is_err());
    assert_eq!(c4_execution(&owner), before);
    assert_eq!(owner.take_command_dispatch_bound(), None);
    // Existing complete-stop boundary, unchanged, on its original Body fixture.
    let (mut owner, binding, path, _) = c3_installed_body();
    let edge = c4_first_motion(&path);
    owner.update_odometry(c4_fractional_actual(&path, edge, 0.5, 0.01, 0.0));
    owner.boundary = vec![[0.0, 0.0], [1.05 + 0.0105, 0.0], [1.05 + 0.0105, 3.0], [0.0, 3.0]];
    owner.refresh_current_body_view();
    let before = c4_execution(&owner);
    c4_dispatch_proof(&mut owner);
    assert!(owner.admit_ground_candidate(path.clone(), path.poses.last().unwrap().clone(), binding, None, None).is_err());
    assert_eq!(c4_execution(&owner), before);
    assert_eq!(owner.take_command_dispatch_bound(), None);

    // Worker lifecycle is tested separately from the admission AtomicBool.
    // No sleep, synthetic completion, fake cancellation result or scheduler swap.
    let (source, _) = c3_direct_source();
    let source = Arc::new(source);
    let (actual_owner, _, _, route) = c3_install(source.clone(), c3_start(), [0.1, 0.2], None, c3_config());
    let retained = c4_execution(&actual_owner);
    let mut kernel = PlanKernel::new(c3_config());
    let (started_tx, started_rx) = mpsc::channel();
    let (compute_tx, compute_rx) = mpsc::channel();
    let (computed_tx, computed_rx) = mpsc::channel();
    let (publish_tx, publish_rx) = mpsc::channel();
    let owner = PlanningOwner::new(move |request: PlanRequest, cancel: &Cancellation| {
        let generation = request.generation;
        started_tx.send(generation).unwrap();
        c3_recv(&compute_rx);
        let result = kernel.compute(request, cancel);
        computed_tx.send((generation, result.outcome.is_ok())).unwrap();
        c3_recv(&publish_rx);
        result
    }).unwrap();
    let cleanup = C3GateCleanup { compute: compute_tx.clone(), publish: publish_tx.clone() };
    assert!(owner.submit(1, c3_request(source.clone(), &route, 1)));
    assert_eq!(c3_recv(&started_rx), 1);
    compute_tx.send(()).unwrap();
    assert_eq!(c3_recv(&computed_rx), (1, true));
    let epoch = owner.epoch();
    owner.cancel();
    assert!(owner.epoch() > epoch);
    publish_tx.send(()).unwrap();
    c3_until(|| owner.is_idle_for_test());
    assert!(owner.take().is_none(), "late real success must be retired by its original worker epoch");
    assert_eq!(c4_execution(&actual_owner), retained);
    assert!(owner.submit(2, c3_request(source, &route, 2)));
    assert_eq!(c3_recv(&started_rx), 2);
    owner.cancel();
    compute_tx.send(()).unwrap();
    assert_eq!(c3_recv(&computed_rx), (2, false));
    publish_tx.send(()).unwrap();
    c3_until(|| owner.is_idle_for_test());
    assert!(owner.take().is_none());
    assert!(!owner.faulted());
    assert_eq!(c4_execution(&actual_owner), retained);
    drop(cleanup);
    drop(owner);
}


#[test]
fn c1c4_07_actual_crosses_connector_then_original_suffix() {
    use std::sync::atomic::AtomicBool;
    let (mut owner, _, _, route) = c3_installed_body();
    let source = owner.source_map.as_ref().unwrap().clone();
    let actual = body_control::body(owner.odometry.as_ref().unwrap());
    let incoming = route.reference.full_window().unwrap().from().continuation([actual.0[0], actual.0[1]]).unwrap();
    let fork = incoming.planning_fork(&source).unwrap();
    let tomo = Tomogram::from_surface(fork.snapshot()).unwrap();
    let physical = source.body_query().start_ground_state(actual).unwrap();
    // The original c1b_08 final-yaw=.2 case, on the existing c3 Body fixture.
    // This is a real Scan invocation, not appended hand-authored yaw poses.
    let mapped = nuwax_nav_scan::plan_driven_ground_on_fork(&fork, &tomo, actual, &physical,
        incoming.walk_state(), &[[1.05, 1.05], [1.55, 1.05]], &c3_motion(), &ScanConfig::default(),
        Some(0.2), None, &[], false, false, None, route.reference.full_window().unwrap())
        .expect("original finite final-yaw Scan prerequisite");
    let original = Arc::new(mapped.binding);
    let path = Path3D { poses: mapped.trajectory.poses.into_iter().map(c3_stamped).collect() };
    assert!(path.poses.windows(2).any(|p| p[0].pose.position == p[1].pose.position
        && p[0].pose.orientation != p[1].pose.orientation), "real same-XY yaw edge prerequisite");
    let edge = c4_first_motion(&path);
    let actual = c4_fractional_actual(&path, edge, 0.5, 0.01, 0.0);
    let (cut_edge, cut_fraction, _) = c4_selected(&owner, &path, &actual);
    c4_assert_half_cut(edge, cut_edge, cut_fraction);
    owner.update_odometry(actual);
    owner.admit_ground_candidate(path.clone(), path.poses.last().unwrap().clone(), original.clone(), None, None).unwrap();
    let rolling = c4_binding(&owner);
    let installed = owner.path.as_ref().unwrap().clone();
    let first = c4_retained_start(&rolling);
    c4_assert_original_suffix(&rolling, first, &original, cut_edge, cut_fraction);
    let unchanged = (owner.ground_cursor(), owner.progress_m(), owner.path_index, owner.committed_ground_basis());
    let basis = unchanged.3.clone().unwrap();
    for _ in 0..3 {
        assert!(owner.predicted_body_motion_with_basis(
            Twist2 { linear: Point2::new(0.05, 0.0), angular_z: 0.0 }, 0.05, Some(&basis)));
        assert_eq!(owner.consume_ground_actual(None).unwrap(), ground_cursor::GroundConsumption::Unchanged);
        assert_eq!((owner.ground_cursor(), owner.progress_m(), owner.path_index, owner.committed_ground_basis()), unchanged);
    }
    // Actual reaches the connector end. A cancelled crossing cannot commit the
    // proposed semantic transition, target index, last_actual or progress.
    owner.update_odometry(installed.poses[first].clone());
    owner.refresh_current_body_view();
    let before_crossing = c4_execution(&owner);
    let cancelled = AtomicBool::new(true);
    assert!(owner.consume_ground_actual(Some(&cancelled)).is_err());
    assert_eq!(c4_execution(&owner), before_crossing);
    owner.consume_ground_actual(None).expect("actual connector/suffix seam must be checked again");
    assert!(owner.progress_m() > 0.0);
    let midway = c4_fractional_actual(&installed, first, 0.5, 0.0, 0.0);
    owner.update_odometry(midway.clone());
    owner.consume_ground_actual(None).unwrap();
    let cursor = owner.ground_cursor().unwrap();
    c4_assert_half_cut(first, cursor.edge, cursor.fraction);
    let consumed = owner.committed_ground_basis().unwrap();
    let (sample, _) = rolling.candidate().original_sample(first, cursor.fraction).unwrap();
    let steps = original.candidate().connection(cut_edge).unwrap().2;
    let original_nav = original.candidate().walk_state_at(cut_edge, sample as f64 / steps as f64,
        [midway.pose.position.x, midway.pose.position.y]).unwrap();
    assert_eq!(consumed.walk_state().unwrap().semantic_key(), original_nav.semantic_key());
    let resume = owner.ground_resume().unwrap();
    let resumed_state = c4_execution(&owner);
    owner.admit_ground_candidate(installed.clone(), installed.poses.last().unwrap().clone(),
        rolling.clone(), Some(resume.clone()), None).unwrap();
    assert_eq!(c4_execution(&owner), resumed_state, "legal resume keeps its target index and partial original address");
    assert!(owner.admit_ground_candidate(installed.clone(), installed.poses.last().unwrap().clone(),
        Arc::new(rolling.as_ref().clone()), Some(resume), None).is_err());
    assert_eq!(c4_execution(&owner), resumed_state);
    let command = match owner.step_ground_internal(0.2, None).unwrap() {
        ground_cursor::GroundControlStep::Control(PlannerStep::Command(command)) => command,
        _ => panic!("fixed retained straight must select a real command before replacement"),
    };
    assert!(owner.selected_body_command.is_some());
    let previous_basis = owner.committed_ground_basis().unwrap();
    // Replace an already composed candidate at its real retained half-edge.
    // PCT R2 inspects flat leaf Arc identity; this runtime case verifies that the
    // repeatedly composed execution still resolves to the original grid.
    let (repeat_edge, repeat_fraction, _) = c4_selected(&owner, &installed, &midway);
    c4_assert_half_cut(first, repeat_edge, repeat_fraction);
    owner.admit_ground_candidate(installed.clone(), installed.poses.last().unwrap().clone(),
        rolling.clone(), None, None).unwrap();
    let twice = c4_binding(&owner);
    let twice_path = owner.path.as_ref().unwrap().clone();
    let twice_first = c4_retained_start(&twice);
    c4_assert_original_suffix(&twice, twice_first, &rolling, repeat_edge, repeat_fraction);
    let (original_edge, original_range) = twice.candidate().original_range(twice_first).unwrap();
    assert_eq!(original_edge, cut_edge);
    assert!((original_range[0] - 0.75).abs() < 1e-12);
    assert_eq!(original_range[1], 1.0);
    assert!(!owner.owns_ground_basis(&previous_basis));
    assert!(owner.selected_body_command.is_none());
    assert!(owner.recheck_selected_body_command(command).is_none());
    assert_eq!(owner.progress_m(), 0.0);
    let mut outcome = ground_cursor::GroundConsumption::Unchanged;
    for actual in twice_path.poses.iter().skip(1) {
        owner.update_odometry(actual.clone());
        outcome = owner.consume_ground_actual(None).expect("directed actual traversal including same-XY yaw");
    }
    assert_eq!(outcome, ground_cursor::GroundConsumption::LocalReferenceEnd {
        covers_original_goal: twice.covers_original_goal(),
    });
    assert!(matches!(owner.step_ground_internal(0.2, None).unwrap(),
        ground_cursor::GroundControlStep::LocalReferenceEnd { .. }));
    assert_eq!(owner.consume_ground_actual(None).unwrap(), ground_cursor::GroundConsumption::Unchanged);
}

#[test]
fn c1c4_08_rolling_lifecycle_and_source_keep_body_compatibility() {
    use crate::replan_feedback::{ActualExecutionContext, GroundActualBasis, same_actual};
    use nuwax_nav_types_messages::GoalOptions;
    let (source, mut blocks) = c3_direct_source();
    let source = Arc::new(source);
    let (mut owner, original, path, _) = c3_install(source.clone(), c3_start(), [0.1, 0.2], None, c3_config());
    // An existing actual owner still requires the explicit seam even when the
    // incoming candidate's FIRST FULL POSE equals the current full observation.
    let initial_basis = owner.committed_ground_basis().unwrap();
    let initial_actual = owner.odometry.as_ref().unwrap().clone();
    assert_eq!(initial_actual.pose, path.poses[0].pose);
    owner.admit_ground_candidate(path.clone(), path.poses.last().unwrap().clone(), original.clone(), None, None)
        .expect("same-pose nonempty replacement still uses the original actual-basis seam");
    let same_pose = c4_binding(&owner);
    let same_pose_suffix = c4_retained_start(&same_pose);
    c4_assert_original_suffix(&same_pose, same_pose_suffix, &original, 0, 0.0);
    assert_eq!(owner.path.as_ref().unwrap().poses[0].pose, initial_actual.pose);
    assert!(!owner.owns_ground_basis(&initial_basis));
    assert_eq!(owner.ground_cursor(), Some(ground_cursor::GroundCursor { edge: 0, fraction: 0.0 }));
    assert_eq!(owner.progress_m(), 0.0);
    assert_eq!(owner.consume_ground_actual(None).unwrap(), ground_cursor::GroundConsumption::Advanced);
    assert!(owner.ground_cursor().unwrap().edge >= same_pose_suffix,
        "next actual-consumption call validates and retires the recorded zero connector");
    assert_eq!(owner.odometry.as_ref().unwrap().pose, initial_actual.pose);
    assert_eq!(owner.progress_m(), 0.0, "retiring a zero-motion seam fabricates no travelled length");
    assert_eq!(owner.consume_ground_actual(None).unwrap(), ground_cursor::GroundConsumption::Unchanged);
    let edge = c4_first_motion(&path);
    let actual = c4_fractional_actual(&path, edge, 0.5, 0.01, 0.0);
    let (cut_edge, cut_fraction, _) = c4_selected(&owner, &path, &actual);
    c4_assert_half_cut(edge, cut_edge, cut_fraction);
    owner.update_odometry(actual);
    let (admitted, walks) = body_control::capture_navigation_walks(||
        owner.admit_ground_candidate(path.clone(), path.poses.last().unwrap().clone(), original.clone(), None, None));
    admitted.expect("real rolling lifecycle prerequisite");
    assert!(walks > 0, "moving Ground must physically evaluate its real connector");
    let rolling = c4_binding(&owner);
    let first = c4_retained_start(&rolling);
    c4_assert_original_suffix(&rolling, first, &original, cut_edge, cut_fraction);
    let committed = owner.committed_ground_basis().unwrap();
    let pose = owner.odometry.as_ref().unwrap().clone();
    let context = ActualExecutionContext { source: source.clone(), odometry: pose.clone(),
        ground: owner.current_ground_query_owned().unwrap().1, ground_evidence:None, navigation: Some(committed.clone()), applied: Twist2::ZERO };
    let xy = [pose.pose.position.x, pose.pose.position.y];
    let mut changed = context.clone();
    changed.navigation = Some(GroundActualBasis::new(rolling.clone(), 0, 0.25, xy));
    assert!(!same_actual(&context, &changed), "same pose retains distinct original addresses");
    changed.navigation = Some(GroundActualBasis::new(Arc::new(rolling.as_ref().clone()), 0, 0.0, xy));
    assert!(!same_actual(&context, &changed), "same trace geometry with a different execution binding is not the same owner");
    // Retained decoder/adapter preparation performs zero control navigation
    // walks. This is structural counter coverage, never a timing assertion.
    let ((), retained_walks) = body_control::capture_navigation_walks(|| {
        for e in first..rolling.edges().len() {
            let motion = rolling.edges()[e].motion();
            rolling.candidate().with_body_navigation_slice(e, 0.0, 1.0,
                [motion[0].0[0], motion[0].0[1]], [motion[1].0[0], motion[1].0[1]], |_| ()).unwrap();
        }
    });
    assert_eq!(retained_walks, 0);
    let mut observations = source.observations().unwrap().as_ref().clone();
    for (&key, block) in &mut blocks {
        for value in block.values_mut() { value.observed_ns = 900; value.hits = 17; }
        assert!(observations.replace_block_with_geometry_changes(key, block.clone()).unwrap().is_empty());
    }
    observations.mark_complete_geometry_source();
    let metadata = Arc::new(source.with_observations(Some(Arc::new(observations.clone()))));
    assert!(nuwax_nav_pct::same_source_geometry(&source, &metadata));
    owner.update_execution_map(ExecutionMap::Surface(metadata.clone())).unwrap();
    assert!(owner.owns_ground_basis(&committed));
    assert_eq!(owner.committed_ground_basis(), Some(committed.clone()));
    assert!(Arc::ptr_eq(owner.source_map.as_ref().unwrap(), &metadata));
    let recent = Arc::new(metadata.with_recent_obstacle_points(1, 1_000_000_000, 0, &[],
        [(0, [-0.95, 0.2, 0.4])]).unwrap());
    assert!(nuwax_nav_pct::same_source_geometry(&source, &recent));
    let cursor = owner.ground_cursor();
    let progress = owner.progress_m();
    owner.update_execution_map(ExecutionMap::Surface(recent)).unwrap();
    assert!(owner.owns_ground_basis(&committed));
    assert!(owner.proposed_ground_basis().is_err(), "original nearby recent point must reach the current physical query");
    assert_eq!(owner.ground_cursor(), cursor);
    assert_eq!(owner.progress_m(), progress);
    assert_eq!(owner.committed_ground_basis(), Some(committed.clone()));
    // Actual fine geometry change from the original C3 metadata fixture.
    let key = [0_i32, 2, 4];
    let block_key = key.map(|v| v.div_euclid(32));
    let block = blocks.entry(block_key).or_default();
    block.insert(key, SurfaceObservation { observed_ns: 901, hits: 2 });
    assert!(!observations.replace_block_with_geometry_changes(block_key, block.clone()).unwrap().is_empty());
    observations.mark_complete_geometry_source();
    let geometry = Arc::new(source.with_observations(Some(Arc::new(observations))));
    assert!(!nuwax_nav_pct::same_source_geometry(&source, &geometry));
    owner.update_execution_map(ExecutionMap::Surface(geometry)).unwrap();
    assert!(owner.ground_cursor().is_none());
    assert!(owner.selected_body_command.is_none());
    assert!(!owner.owns_ground_basis(&committed));
    for new_goal in [false, true] {
        let (source, _) = c3_direct_source();
        let (mut owner, binding, path, _) = c3_install(Arc::new(source), c3_start(), [0.1, 0.2], None, c3_config());
        let edge = c4_first_motion(&path);
        owner.update_odometry(c4_fractional_actual(&path, edge, 0.5, 0.01, 0.0));
        owner.admit_ground_candidate(path.clone(), path.poses.last().unwrap().clone(), binding, None, None).unwrap();
        let old = owner.committed_ground_basis().unwrap();
        if new_goal { owner.configure_goal(&c3_pose(0.2, 0.2, 0.0), GoalOptions::default()).unwrap(); }
        else { owner.stop(); }
        assert!(!owner.owns_ground_basis(&old));
        assert!(owner.ground_cursor().is_none());
        assert!(owner.selected_body_command.is_none());
    }
    // Original Body-none physical projection/sweep and moving admission.
    let body = c3_body_source();
    let actual = body_control::body(&c3_pose(1.05, 1.05, 0.0));
    let query = body.body_query();
    let ground = query.start_ground_state(actual).unwrap();
    let projected = query.project_motion(actual, &ground, [1.15, 1.05], 0.0, &mut None).unwrap();
    query.validate_projected_motion(actual, &projected, &[], &mut None).unwrap();
    let (checked, walks) = body_control::capture_navigation_walks(||
        body_control::control_project_edge(&query, actual, &ground, None, [1.15, 1.05], 0.0,
            &[], body_control::ControlEdgeProfile::Preview));
    let checked = checked.unwrap();
    assert_eq!(walks, 0);
    assert_eq!(checked.body.pose, projected.pose());
    assert_eq!(checked.body.ground.as_ref(), Some(projected.ground()));
    assert!(checked.navigation.is_none());
    // Same physical unit orientation with the antipodal representation. A
    // motionless connector keeps the actual quaternion rather than creating a
    // fictitious edge to the projection kernel's canonical positive w.
    {
        let (mut owner,binding,path,_)=c3_installed_body();
        let mut actual=owner.odometry.as_ref().unwrap().clone();
        actual.pose.orientation=Quaternion {x:0.0,y:0.0,z:0.0,w:-1.0};
        owner.update_odometry(actual.clone());
        owner.admit_ground_candidate(path.clone(),path.poses.last().unwrap().clone(),binding,None,None).unwrap();
        let installed=owner.path.as_ref().unwrap();
        assert_eq!(installed.poses[0].pose,actual.pose);
        assert_eq!(installed.poses[1].pose,actual.pose);
        assert_eq!(owner.ground_cursor(),Some(ground_cursor::GroundCursor {edge:0,fraction:0.0}));
        assert_eq!(owner.consume_ground_actual(None).unwrap(),ground_cursor::GroundConsumption::Advanced);
        assert_eq!(owner.progress_m(),0.0);
        assert!(owner.ground_cursor().unwrap().edge>=1);
        assert_eq!(owner.odometry.as_ref().unwrap().pose,actual.pose);
    }
    let (mut owner, binding, path, _) = c3_installed_body();
    let stationary_actual = owner.odometry.as_ref().unwrap().clone();
    owner.admit_ground_candidate(path.clone(), path.poses.last().unwrap().clone(), binding, None, None).unwrap();
    assert_eq!(owner.ground_cursor(), Some(ground_cursor::GroundCursor { edge: 0, fraction: 0.0 }));
    assert_eq!(owner.consume_ground_actual(None).unwrap(), ground_cursor::GroundConsumption::Advanced);
    assert_eq!(owner.progress_m(), 0.0);
    assert_eq!(owner.odometry.as_ref().unwrap().pose, stationary_actual.pose);
    let command = match owner.step_ground_internal(0.2, None).unwrap() {
        ground_cursor::GroundControlStep::Control(PlannerStep::Command(command)) => command,
        _ => panic!("retired zero connector must not strand the original calibrated straight control"),
    };
    assert!(command.linear.x > 0.0, "real forward control must not remain at a zero connector");
    owner.stop();
    owner.update_odometry(c4_fractional_actual(&path, c4_first_motion(&path), 0.5, 0.0, 0.0));
    let (result, walks) = body_control::capture_navigation_walks(|| owner.replace_execution_path(
        ExecutionPath::Surface(path.clone()), None, path.poses.last().unwrap().clone()));
    result.unwrap();
    assert_eq!(walks, 0);
    assert!(owner.ground_cursor().is_none());
    let (command, walks) = body_control::capture_navigation_walks(|| owner.follow_body_step(0.2));
    assert!(matches!(command, PlannerStep::Command(_)));
    assert_eq!(walks, 0);
}

// C1-c5: explicit Initial lifecycle and real stationary Scan terminals.
// Public RouteFollower goal dispatch and stopped-arrival results remain separate.
fn c5_initial(source:Arc<SurfaceSnapshot>,actual:BodyPose,height:Option<f64>)
    ->(SurfaceFollower,nuwax_nav_pct::ground_route::PreparedGroundStart) {
    let config=c3_config();
    let initial=config.pct.prepare_ground_start(&source,GroundRouteStart {
        xy:[actual.0[0],actual.0[1]],ground_z:height},None).unwrap();
    let mut owner=SurfaceFollower::new(config).unwrap();
    owner.update_execution_map(ExecutionMap::Surface(source)).unwrap();
    owner.update_odometry(c3_stamped(actual));
    owner.initialize_ground_actual(initial.basis().clone(),None).unwrap();
    assert!(owner.path.is_none());assert!(owner.ground_binding_for_test().is_none());
    assert!(owner.ground_cursor().is_none());
    (owner,initial)
}
fn c5_pending(owner:&mut SurfaceFollower) {
    assert!(matches!(owner.step_ground_internal(0.2,None).unwrap(),
        ground_cursor::GroundControlStep::Control(PlannerStep::Pending(_))));
    assert!(owner.path.is_none());assert!(owner.ground_cursor().is_none());
}
fn c5_stationary(source:&SurfaceSnapshot,route:&PlannedGroundRoute,
    incoming:&nuwax_nav_pct::ground_route::OwnedGroundWalkState,pose:BodyPose)
    ->(Arc<GroundCandidateBinding>,Path3D) {
    let xy=[pose.0[0],pose.0[1]];
    assert_eq!(incoming.walk_state().xy(),xy);
    assert!(incoming.same_reference_owner(&route.reference));
    let physical=source.body_query().start_ground_state(pose).unwrap();
    let fork=incoming.planning_fork(source).unwrap();
    let tomo=Tomogram::from_surface(fork.snapshot()).unwrap();
    let mapped=nuwax_nav_scan::plan_driven_ground_on_fork(&fork,&tomo,pose,&physical,
        incoming.walk_state(),&[xy],&c3_motion(),&ScanConfig::default(),None,Some((xy,0.0)),
        &[],false,false,None,route.reference.full_window().unwrap()).unwrap();
    assert_eq!(mapped.trajectory.poses,vec![pose]);assert!(mapped.binding.edges().is_empty());
    assert_eq!(mapped.binding.candidate().connection_count(),0);
    assert_eq!(mapped.binding.stationary_pose(),Some(pose));
    assert_eq!(mapped.binding.candidate().walk_state_at(0,0.0,xy).unwrap().semantic_key(),incoming.semantic_key());
    (Arc::new(mapped.binding),Path3D {poses:mapped.trajectory.poses.into_iter().map(c3_stamped).collect()})
}
fn c5_consume_path(owner:&mut SurfaceFollower,path:&Path3D) {
    for pose in path.poses.iter().skip(1) {owner.update_odometry(pose.clone());owner.consume_ground_actual(None).unwrap();}
}
fn c5_terminal_matches(joined:&GroundCandidateBinding,original:&GroundCandidateBinding,xy:[f64;2]) {
    let last=joined.candidate().connection_count()-1;
    assert_eq!(joined.edges()[last].origin(),nuwax_nav_scan::CandidateEdgeOrigin::AdmissionTerminal);
    assert_eq!(joined.candidate().connection(last),None,"point event has no invented sample grid");
    assert_eq!(joined.candidate().original_sample(last,0.0),None);
    assert_eq!(joined.candidate().walk_state_at(last,1.0,xy).unwrap().semantic_key(),
        original.candidate().walk_state_at(0,0.0,xy).unwrap().semantic_key());
    assert_eq!(joined.family(),original.family());
    assert_eq!(joined.covers_original_goal(),original.covers_original_goal());
}

#[test]
fn i01_first_pending_movement_uses_initial_actual_basis() {
    let (source,_)=c3_direct_source();let source=Arc::new(source);
    let (mut actual_owner,seed)=c5_initial(source.clone(),c3_start(),None);
    let route=actual_owner.config.pct.ground_route_from_start(&source,c3_start(),&seed,
        GroundRouteGoal {xy:[0.1,0.2],requested_ground_z:None},0.0,&[],None,None,SearchLimits::default(),&[]).unwrap();
    let context=actual_owner.refresh_execution_context(Twist2::ZERO).unwrap();
    let mut request=c3_request(source.clone(),&route,1);
    let input=request.ground.take().unwrap();
    request.ground=GroundPlanInput::from_actual(&context,input.intent,input.xy_reference,input.arrival_tolerance_m);
    request.pose=context.odometry.clone();request.start_ground=Ok(context.ground.clone());
    assert_eq!(request.ground.as_ref().unwrap().incoming.semantic_key(),seed.basis().semantic_key());
    let mut kernel=PlanKernel::new(c3_config());
    let (started_tx,started_rx)=mpsc::channel();let (compute_tx,compute_rx)=mpsc::channel();
    let (computed_tx,computed_rx)=mpsc::channel();let (publish_tx,publish_rx)=mpsc::channel();
    let worker=PlanningOwner::new(move |request:PlanRequest,cancel:&Cancellation| {
        started_tx.send(request.generation).unwrap();c3_recv(&compute_rx);
        let result=kernel.compute(request,cancel);
        computed_tx.send(result.outcome.as_ref().map(|v|v.path.clone()).map_err(|e|format!("{e:?}"))).unwrap();
        c3_recv(&publish_rx);result
    }).unwrap();
    let cleanup=C3GateCleanup {compute:compute_tx.clone(),publish:publish_tx.clone()};
    assert!(worker.submit(1,request));assert_eq!(c3_recv(&started_rx),1);compute_tx.send(()).unwrap();
    let planned=c3_recv(&computed_rx).unwrap();assert!(worker.take().is_none());
    let edge=c4_first_motion(&planned);let observed=c4_fractional_actual(&planned,edge,0.5,0.01,0.0);
    actual_owner.update_odometry(observed.clone());
    let read=actual_owner.refresh_execution_context(Twist2::ZERO).unwrap();assert!(read.navigation.is_none());
    assert_eq!(actual_owner.initial_ground_for_test().unwrap().0,*seed.basis());
    c5_pending(&mut actual_owner);
    let initial=actual_owner.initial_ground_for_test().unwrap();assert_eq!(initial.1.pose,observed.pose);
    assert_eq!(initial.0.walk_state().xy(),[observed.pose.position.x,observed.pose.position.y]);
    c5_pending(&mut actual_owner);assert_eq!(actual_owner.initial_ground_for_test().unwrap(),initial);
    publish_tx.send(()).unwrap();c3_until(||worker.completion_ready_for_test());
    let completion=worker.take().unwrap();assert_eq!((completion.id,completion.epoch),(1,worker.epoch()));
    let candidate=completion.value.outcome.unwrap();let original=candidate.ground_binding.unwrap();
    assert_eq!(candidate.path.poses[0].pose,c3_stamped(c3_start()).pose);
    let (cut,fraction,_)=c4_selected(&actual_owner,&candidate.path,&observed);c4_assert_half_cut(edge,cut,fraction);
    actual_owner.admit_ground_candidate(candidate.path.clone(),candidate.path.poses.last().unwrap().clone(),original.clone(),None,None).unwrap();
    assert!(actual_owner.initial_ground_for_test().is_none());
    let joined=c4_binding(&actual_owner);let first=c4_retained_start(&joined);
    c4_assert_original_suffix(&joined,first,&original,cut,fraction);
    let installed=actual_owner.path.as_ref().unwrap().clone();assert_eq!(installed.poses[0].pose,observed.pose);
    assert!(installed.poses[..=first].windows(2).any(|p|p[0].pose.position.distance(p[1].pose.position)>0.0));
    assert_eq!(actual_owner.progress_m(),0.0);
    actual_owner.update_odometry(c4_fractional_actual(&installed,0,0.5,0.0,0.0));
    assert_eq!(actual_owner.consume_ground_actual(None).unwrap(),ground_cursor::GroundConsumption::Advanced);
    drop(cleanup);drop(worker);
}

#[test]
fn i02_initial_context_never_pairs_new_pose_with_old_navigation() {
    use crate::replan_feedback::SurfaceReplanInput;
    let (source,_)=c3_direct_source();let source=Arc::new(source);
    let (mut owner,seed)=c5_initial(source.clone(),c3_start(),Some(0.0));
    let original=owner.initial_ground_for_test().unwrap();
    let route=owner.config.pct.ground_route_from_start(&source,c3_start(),&seed,
        GroundRouteGoal {xy:[0.1,0.2],requested_ground_z:None},0.0,&[],None,None,SearchLimits::default(),&[]).unwrap();
    let intent=GroundIntent::new(route.reference.full_window().unwrap());
    let mut last=None;
    for x in [-0.95,-0.90] {
        let actual=c3_stamped(([x,0.2,0.4],c3_start().1));owner.update_odometry(actual.clone());
        let context=owner.refresh_execution_context(Twist2::ZERO).unwrap();
        assert_eq!(context.odometry,actual);assert!(context.navigation.is_none());
        assert_eq!(owner.initial_ground_for_test().unwrap(),original);last=Some(context);
    }
    let context=last.unwrap();
    assert!(GroundPlanInput::from_actual(&context,intent.clone(),Arc::new(c3_xy_reference()),Some(0.1)).is_none());
    let input=SurfaceReplanInput {source:source.clone(),start:context.odometry.clone(),goal:crate::ExecutionGoal::Body(c3_stamped(([0.1,0.2,0.4],c3_start().1))),
        options:nuwax_nav_types_messages::GoalOptions::default(),boundary:Arc::new(vec![]),actual:Some(context.clone()),ground_intent:Some(intent),
        limits:SearchLimits::default(),offset:0.4,local_planning_enabled:true,penalties:Arc::new(vec![])};
    assert!(!input.same_complete_input(&input));let mut body=input.clone();body.ground_intent=None;
    assert!(body.same_complete_input(&body),"old Body context equality remains valid");
    let walk=seed.basis().walk_state().walk_xy([-0.90,0.2],&[],None).unwrap();let query=seed.basis().bind_body_query(&source).unwrap();
    walk.with_body_navigation(|_,nav|query.validate_motion_with_navigation(c3_start(),body_control::body(&context.odometry),
        &original.2,nav,&[],&mut None)).unwrap().unwrap();
    let expected=seed.basis().recorded_successor(&walk).unwrap();
    c5_pending(&mut owner);let committed=owner.initial_ground_for_test().unwrap();
    assert_eq!(committed.0,expected);assert_eq!(committed.1,context.odometry);assert_eq!(committed.2,context.ground);
    let paired=owner.refresh_execution_context(Twist2::ZERO).unwrap();assert!(paired.navigation.is_some());
    let mut changed=context.odometry.clone();changed.pose.position.z+=0.001;owner.update_odometry(changed);
    let flag=std::sync::atomic::AtomicBool::new(true);
    assert!(owner.step_ground_internal(0.2,Some(&flag)).is_err());assert_eq!(owner.initial_ground_for_test().unwrap(),committed);
    c5_pending(&mut owner);assert_ne!(owner.initial_ground_for_test().unwrap().1.pose,committed.1.pose);
}

#[test]
fn i03_initial_unknown_bias_and_source_retirement() {
    let (source,mut blocks)=c3_direct_source();let source=Arc::new(source);
    let actual=([100.0,100.0,0.55],c3_start().1);
    for height in [None,Some(0.0)] {
        let (mut owner,seed)=c5_initial(source.clone(),actual,height);
        assert_eq!(seed.basis().walk_state().planned_ground_z(),height);
        owner.update_odometry(c3_stamped(([100.05,100.0,0.56],actual.1)));c5_pending(&mut owner);
        assert_eq!(owner.initial_ground_for_test().unwrap().0.walk_state().planned_ground_z(),height);
        assert_eq!(owner.odometry.as_ref().unwrap().pose.position.z,0.56);
        let old=owner.initial_ground_for_test().unwrap();
        let mut observations=source.observations().unwrap().as_ref().clone();
        for (&key,values) in &mut blocks {for value in values.values_mut(){value.observed_ns=900;value.hits=17;}
            assert!(observations.replace_block_with_geometry_changes(key,values.clone()).unwrap().is_empty());}
        observations.mark_complete_geometry_source();
        let timed=Arc::new(source.with_observations(Some(Arc::new(observations))));
        owner.update_execution_map(ExecutionMap::Surface(timed.clone())).unwrap();
        assert_eq!(owner.initial_ground_for_test().unwrap(),old);
        let recent=Arc::new(timed.with_recent_obstacle_points(1,1_000_000_000,0,&[],[(0,[100.05,100.0,0.56])]).unwrap());
        owner.update_execution_map(ExecutionMap::Surface(recent)).unwrap();
        assert!(owner.step_ground_internal(0.2,None).is_err());assert_eq!(owner.initial_ground_for_test().unwrap(),old);
        let mut model=source.body_envelope().unwrap().clone();model.footprint_xy_m[0][0]-=0.01;
        owner.update_execution_map(ExecutionMap::Surface(Arc::new(source.with_body_envelope(model).unwrap()))).unwrap();
        assert!(owner.committed_ground_basis().is_none());
    }
}

#[test]
fn i04_owned_stationary_terminal_is_admissible() {
    for antipode in [false,true] {
        let (mut owner,_,path,route)=c3_installed_body();c5_consume_path(&mut owner,&path);
        let terminal=body_control::body(path.poses.last().unwrap());let xy=[terminal.0[0],terminal.0[1]];
        let incoming=route.reference.full_window().unwrap().to().continuation(xy).unwrap();
        let (single,single_path)=c5_stationary(owner.source_map.as_ref().unwrap(),&route,&incoming,terminal);
        assert!(single.candidate().end_matches(route.reference.full_window().unwrap().to()));
        assert!(single.covers_original_goal());let old=c4_binding(&owner);
        if antipode {owner.update_odometry(c3_stamped((terminal.0,terminal.1.map(|v|-v))));}
        let actual=owner.odometry.as_ref().unwrap().clone();
        owner.admit_ground_candidate(single_path.clone(),single_path.poses[0].clone(),single.clone(),None,None).unwrap();
        let joined=c4_binding(&owner);assert!(!Arc::ptr_eq(&old,&joined));
        c5_terminal_matches(&joined,&single,xy);assert_eq!(owner.path.as_ref().unwrap().poses[0].pose,actual.pose);
        assert_eq!(owner.ground_cursor().unwrap().fraction,0.0);
        assert!(matches!(owner.step_ground_internal(0.2,None).unwrap(),ground_cursor::GroundControlStep::LocalReferenceEnd {covers_original_goal:true}));
        assert!(single.edges().is_empty());assert_eq!(single.stationary_pose(),Some(terminal));
    }
}

#[test]
fn i05_stationary_yaw_and_semantic_seam_are_consumed() {
    let (mut owner,_,path,route)=c3_installed_body();c5_consume_path(&mut owner,&path);
    let terminal=body_control::body(path.poses.last().unwrap());let xy=[terminal.0[0],terminal.0[1]];
    let yaw=Quaternion::from_yaw(0.2);let target=(terminal.0,[yaw.x,yaw.y,yaw.z,yaw.w]);
    let incoming=route.reference.full_window().unwrap().to().continuation(xy).unwrap();
    let (single,single_path)=c5_stationary(owner.source_map.as_ref().unwrap(),&route,&incoming,target);
    owner.admit_ground_candidate(single_path.clone(),single_path.poses[0].clone(),single.clone(),None,None).unwrap();
    let joined=c4_binding(&owner);let installed=owner.path.as_ref().unwrap().clone();
    assert!(installed.poses.windows(2).any(|p|p[0].pose.orientation!=p[1].pose.orientation));
    assert!(installed.poses.windows(2).any(|p|
        angle_diff(p[1].pose.orientation.yaw(),p[0].pose.orientation.yaw()).abs()>0.0));
    let target_yaw=yaw.yaw();let installed_yaw=installed.poses.last().unwrap().pose.orientation.yaw();
    assert!(angle_diff(installed_yaw,target_yaw).abs()<1e-12);
    assert!(installed.poses.iter().all(|p|p.pose.position==installed.poses[0].pose.position));
    c5_terminal_matches(&joined,&single,xy);
    assert_eq!(owner.consume_ground_actual(None).unwrap(),ground_cursor::GroundConsumption::Unchanged);
    let saved=c4_execution(&owner);let flag=std::sync::atomic::AtomicBool::new(true);
    owner.update_odometry(installed.poses[1].clone());assert!(owner.consume_ground_actual(Some(&flag)).is_err());
    assert_eq!(owner.ground_cursor(),saved.cursor);assert_eq!(owner.committed_ground_basis(),saved.basis);
    c5_consume_path(&mut owner,&installed);
    let actual_yaw=owner.odometry.as_ref().unwrap().pose.orientation.yaw();
    assert!(angle_diff(actual_yaw,target_yaw).abs()<1e-12);
    eprintln!("I05 real yaw start={:?} target={:?} installed={:?} consumed_actual={:?}",
        installed.poses[0].pose.orientation.yaw(),target_yaw,installed_yaw,actual_yaw);
    assert!(matches!(owner.step_ground_internal(0.2,None).unwrap(),ground_cursor::GroundControlStep::LocalReferenceEnd {..}));
    // A different legitimate original sample at the same actual pose survives
    // the zero connector as its own terminal event, not as the connector end.
    let (mut owner,_,path,route)=c3_installed_body();let edge=c4_first_motion(&path);
    let actual=c4_fractional_actual(&path,edge,0.5,0.0,0.0);owner.update_odometry(actual.clone());owner.consume_ground_actual(None).unwrap();
    let xy=[actual.pose.position.x,actual.pose.position.y];
    let old=owner.committed_ground_basis().unwrap().continuation().unwrap();
    // The earlier original sample remains in the current/adjacent column;
    // a far terminal Unanchored id is not a legal zero-walk incoming here.
    let incoming=route.reference.full_window().unwrap().from().continuation(xy).unwrap();
    assert_ne!(old.walk_state().reference_sample_xy(),incoming.walk_state().reference_sample_xy());
    let (single,p)=c5_stationary(owner.source_map.as_ref().unwrap(),&route,&incoming,body_control::body(&actual));
    assert!(!single.covers_original_goal());
    owner.admit_ground_candidate(p.clone(),p.poses[0].clone(),single.clone(),None,None).unwrap();
    let joined=c4_binding(&owner);c5_terminal_matches(&joined,&single,xy);
    let end=joined.edges().len()-1;assert!(joined.candidate().is_splice_boundary(end));
    assert_ne!(joined.candidate().walk_state_at(end-1,1.0,xy).unwrap().reference_sample_xy(),
        joined.candidate().walk_state_at(end,0.0,xy).unwrap().reference_sample_xy());
    assert!(matches!(owner.step_ground_internal(0.2,None).unwrap(),ground_cursor::GroundControlStep::LocalReferenceEnd {covers_original_goal:false}));
    assert_eq!(owner.committed_ground_basis().unwrap().walk_state().unwrap().semantic_key(),incoming.semantic_key());
}

#[test]
fn i06_moving_stationary_terminal_uses_real_connector() {
    for cancel_stage in [None,Some("entry"),Some("connector"),Some("commit")] {
        let (mut owner,_,path,route)=c3_installed_body();let end=path.poses.last().unwrap();
        let previous=path.poses.iter().rposition(|p|p.pose.position!=end.pose.position).unwrap();
        for p in path.poses.iter().take(previous+1).skip(1) {owner.update_odometry(p.clone());owner.consume_ground_actual(None).unwrap();}
        let actual=owner.odometry.as_ref().unwrap().clone();let distance=actual.pose.position.distance(end.pose.position);
        assert!(distance>0.0 && distance<=owner.config.max_path_deviation_m);
        let terminal=body_control::body(end);let xy=[terminal.0[0],terminal.0[1]];
        let incoming=route.reference.full_window().unwrap().to().continuation(xy).unwrap();
        let (single,p)=c5_stationary(owner.source_map.as_ref().unwrap(),&route,&incoming,terminal);
        let saved=c4_execution(&owner);let flag=Arc::new(std::sync::atomic::AtomicBool::new(cancel_stage==Some("entry")));
        let result=if let Some(stage)=cancel_stage.filter(|s|*s!="entry") {
            ground_cursor::cancel_ground_admission_at_for_test(stage,1,flag.clone(),||
                owner.admit_ground_candidate(p.clone(),p.poses[0].clone(),single.clone(),None,Some(&flag)))
        }else {owner.admit_ground_candidate(p.clone(),p.poses[0].clone(),single.clone(),None,Some(&flag))};
        if cancel_stage.is_some() {
            assert!(flag.load(std::sync::atomic::Ordering::Relaxed));assert!(result.unwrap_err().contains("cancel"));
            assert_eq!(c4_execution(&owner),saved);continue;
        }
        result.unwrap();let joined=c4_binding(&owner);let installed=owner.path.as_ref().unwrap().clone();
        assert_eq!(installed.poses[0].pose,actual.pose);assert!(installed.poses.windows(2).any(|p|p[0].pose.position!=p[1].pose.position));
        c5_terminal_matches(&joined,&single,xy);assert_eq!(owner.progress_m(),0.0);
        assert_eq!(owner.consume_ground_actual(None).unwrap(),ground_cursor::GroundConsumption::Unchanged);
        c5_consume_path(&mut owner,&installed);
        assert!(matches!(owner.step_ground_internal(0.2,None).unwrap(),ground_cursor::GroundControlStep::LocalReferenceEnd {covers_original_goal:true}));
    }
    let (source,_)=c3_direct_source();let (mut owner,_,path,route)=c3_install(Arc::new(source),c3_start(),[0.1,0.2],None,c3_config());
    let terminal=body_control::body(path.poses.last().unwrap());let xy=[terminal.0[0],terminal.0[1]];
    assert!(c3_stamped(c3_start()).pose.position.distance(path.poses.last().unwrap().pose.position)>owner.config.max_path_deviation_m);
    let incoming=route.reference.full_window().unwrap().to().continuation(xy).unwrap();
    let (single,p)=c5_stationary(owner.source_map.as_ref().unwrap(),&route,&incoming,terminal);
    let saved=c4_execution(&owner);
    assert!(owner.admit_ground_candidate(p.clone(),p.poses[0].clone(),single,None,None).unwrap_err().contains("window"));
    assert_eq!(c4_execution(&owner),saved);
}

#[test]
fn i07_prepared_start_and_original_target_keep_presence() {
    let (source,_)=c3_direct_source();let planner=PctPlanner::default();let actual=c3_start();
    for height in [None,Some(0.0)] {
        let start=GroundRouteStart {xy:[actual.0[0],actual.0[1]],ground_z:height};
        let initial=planner.prepare_ground_start(&source,start,None).unwrap();
        assert_eq!(initial.basis().walk_state().planned_ground_z().is_some(),height.is_some());
        for requested in [None,Some(0.0)] {
            let goal=GroundRouteGoal {xy:[0.1,0.2],requested_ground_z:requested};
            let old=planner.ground_route_with_limits_and_penalties(&source,actual,start,goal,0.0,&[],None,None,SearchLimits::default(),&[]).unwrap();
            let prepared=planner.ground_route_from_start(&source,actual,&initial,goal,0.0,&[],None,None,SearchLimits::default(),&[]).unwrap();
            assert_eq!(old.points,prepared.points);assert_eq!(old.guide_body_poses,prepared.guide_body_poses);
            assert_eq!(old.height,prepared.height);assert_eq!(old.planned_layer,prepared.planned_layer);
            assert_eq!(prepared.reference.original_goal().requested_ground_z,requested);
            assert_eq!(prepared.reference.original_goal().xy,goal.xy);
            assert_eq!(prepared.reference.target_sample().is_some(),requested.is_some());
            assert_eq!(prepared.reference.full_window().unwrap().from().walk_state().semantic_key(),initial.basis().semantic_key());
            let retained=prepared.reference.clone();assert_eq!(retained.original_goal().requested_ground_z,requested);
        }
    }
    let cancel=std::sync::atomic::AtomicBool::new(true);
    assert!(planner.prepare_ground_start(&source,GroundRouteStart {xy:[-1.0,0.2],ground_z:None},Some(&cancel)).is_err());
}

#[test]
fn i08_initial_candidate_stationary_lifecycle_keeps_body() {
    let (source,_)=c3_direct_source();let source=Arc::new(source);
    let (mut initial,_)=c5_initial(source.clone(),c3_start(),None);
    let old=initial.committed_ground_basis().unwrap();initial.stop();assert!(initial.committed_ground_basis().is_none());
    assert!(!initial.owns_ground_basis(&old));
    let (mut owner,_,path,route)=c3_installed_body();c5_consume_path(&mut owner,&path);
    let terminal=body_control::body(path.poses.last().unwrap());let xy=[terminal.0[0],terminal.0[1]];
    let incoming=route.reference.full_window().unwrap().to().continuation(xy).unwrap();
    let (single,p)=c5_stationary(owner.source_map.as_ref().unwrap(),&route,&incoming,terminal);
    owner.admit_ground_candidate(p.clone(),p.poses[0].clone(),single,None,None).unwrap();
    let _=owner.step_ground_internal(0.2,None).unwrap();
    let token=owner.ground_resume().unwrap();let binding=c4_binding(&owner);let installed=owner.path.as_ref().unwrap().clone();
    let saved=owner.ground_cursor();owner.admit_ground_candidate(installed.clone(),p.poses[0].clone(),binding,Some(token),None).unwrap();
    assert_eq!(owner.ground_cursor(),saved);
    owner.stop();assert!(owner.ground_execution.is_none());
    owner.replace_execution_path(ExecutionPath::Surface(installed.clone()),None,p.poses[0].clone()).unwrap();
    assert!(owner.ground_execution.is_none());
    let query=source.body_query();let physical=query.start_ground_state(c3_start()).unwrap();
    let to=[-0.95,0.2];let expected=query.project_motion(c3_start(),&physical,to,0.0,&mut None).unwrap();
    let (actual,walks)=body_control::capture_navigation_walks(||body_control::control_project_edge(&query,c3_start(),&physical,None,to,0.0,&[],body_control::ControlEdgeProfile::Preview));
    let actual=actual.unwrap();assert_eq!(actual.body.pose,expected.pose());assert!(actual.navigation.is_none());assert_eq!(walks,0);
}

// D1 appended qualification only. Append after the existing C1-c3/c4/c5 tests.
// NOT BUILT / NOT LISTED / NOT RUN. Existing helpers and fixtures remain intact.

fn d1_goal(xy: [f64; 2], requested_ground_z: Option<f64>) -> crate::ExecutionGoal {
    crate::ExecutionGoal::Ground(crate::GroundExecutionGoal {
        frame_id: NavFrameId::Map, xy, yaw: 0.0, requested_ground_z,
    })
}
fn d1_options() -> nuwax_nav_types_messages::GoalOptions {
    nuwax_nav_types_messages::GoalOptions {
        position_tolerance_m: 0.0, yaw_tolerance_rad: 0.0, allow_reverse: false,
    }
}
fn d1_initial_capture(source: Arc<SurfaceSnapshot>, actual: BodyPose,
    basis: nuwax_nav_pct::ground_route::OwnedGroundWalkState) -> crate::GroundInitialCapture {
    let mut observer = SurfaceFollower::new(c3_config()).unwrap();
    observer.update_execution_map(ExecutionMap::Surface(source)).unwrap();
    observer.update_odometry(c3_stamped(actual));
    let context = observer.refresh_execution_context(Twist2::ZERO).unwrap();
    assert!(context.navigation.is_none());
    crate::GroundInitialCapture::from_actual(basis, context).unwrap()
}
fn d1_typed_initial(source: Arc<SurfaceSnapshot>, actual: BodyPose,
    height: Option<f64>, goal: crate::ExecutionGoal) -> SurfaceFollower {
    let prepared = PctPlanner::default().prepare_ground_start(&source,
        GroundRouteStart { xy: [actual.0[0], actual.0[1]], ground_z: height }, None).unwrap();
    d1_captured_owner(source, actual, prepared.basis().clone(), goal)
}
fn d1_captured_owner(source: Arc<SurfaceSnapshot>, actual: BodyPose,
    basis: nuwax_nav_pct::ground_route::OwnedGroundWalkState, goal: crate::ExecutionGoal) -> SurfaceFollower {
    let capture = d1_initial_capture(source.clone(), actual, basis);
    let mut owner = SurfaceFollower::new(c3_config()).unwrap();
    owner.configure_execution_goal(&goal, d1_options(), Some(capture)).unwrap();
    owner.update_execution_map(ExecutionMap::Surface(source)).unwrap();
    owner.update_odometry(c3_stamped(actual));
    owner.consume_initial_actual_for_planning().unwrap();
    owner
}
fn d1_input(actual: Arc<crate::replan_feedback::ActualExecutionContext>,
    goal: crate::ExecutionGoal, penalties: Vec<nuwax_nav_pct::RoutePenalty>)
    -> crate::replan_feedback::SurfaceReplanInput {
    crate::replan_feedback::SurfaceReplanInput {
        source: actual.source.clone(), start: actual.odometry.clone(), goal,
        options: d1_options(), boundary: Arc::new(vec![]), actual: Some(actual),
        ground_intent: None, limits: SearchLimits::default(), offset: 0.4,
        local_planning_enabled: true, penalties: Arc::new(penalties),
    }
}
fn d1_path(route: &PlannedGroundRoute) -> crate::ExecutionPath {
    crate::ExecutionPath::Ground(crate::GroundExecutionPath {
        reference: route.reference.clone(),
        body_guidance: Path3D { poses: route.guide_body_poses.iter().copied().map(c3_stamped).collect() },
    })
}
fn d1_relation(owner: &mut SurfaceFollower) -> nuwax_nav_types_messages::GroundLayerResult {
    owner.refresh_execution_context(Twist2::ZERO).unwrap().ground_evidence.as_ref()
        .expect("typed owner publishes actual source evidence").relation
}
fn d1_consume_fraction(owner: &mut SurfaceFollower, path: &Path3D, edge: usize, fraction: f64) {
    for pose in path.poses.iter().take(edge + 1).skip(1) {
        owner.update_odometry(pose.clone()); owner.consume_ground_actual(None).unwrap();
    }
    let actual = c4_fractional_actual(path, edge, fraction, 0.001, 0.0);
    owner.update_odometry(actual); owner.consume_ground_actual(None).unwrap();
}

// Panic cleanup cancels the real scheduler task; normal teardown awaits its
// original shutdown/drop, which in turn joins PlannerWorker/PlanningOwner.
struct D1SpawnCleanup(Option<tokio::task::JoinHandle<Result<(), crate::RuntimeError>>>);
impl Drop for D1SpawnCleanup {
    fn drop(&mut self) { if let Some(task) = self.0.take() { task.abort(); } }
}
async fn d1_handle_round_trip(owner: &mut SurfaceFollower, goal: crate::ExecutionGoal,
    penalties: Vec<nuwax_nav_pct::RoutePenalty>, expect_search: bool) -> PlannedGroundRoute {
    use crate::tests::ground_arrival_qualification::observed_worker;
    let before = owner.refresh_execution_context(Twist2::ZERO).unwrap();
    let incoming = before.navigation.as_ref().unwrap().continuation().unwrap();
    let key = incoming.semantic_key();
    let cursor = owner.ground_cursor();
    let capture = crate::GroundInitialCapture::from_actual(incoming, before.clone()).unwrap();
    let (worker, audit, dropped) = observed_worker(c3_config());
    let motion = crate::tests::FakeMotion(Arc::new(std::sync::Mutex::new(
        crate::tests::FakeMotionState::default())));
    let (handle, task) = crate::spawn_scheduler(crate::NavScheduler::new(
        motion, worker, crate::SchedulerConfig::default()).unwrap());
    let mut cleanup = D1SpawnCleanup(Some(task));
    handle.update_costmap(crate::CostmapObservation { costmap: ExecutionMap::Surface(before.source.clone()),
        source_timestamp_ns: 1 }).await.unwrap();
    handle.update_odometry(before.odometry.clone()).await.unwrap();
    let mut session = crate::tests::session_for("d01-consumed");
    session.goal = goal.clone(); session.options = d1_options(); session.ground_initial = Some(capture);
    let identity = session.execution_identity.clone();
    handle.start_session(session).await.unwrap();
    let published = tokio::time::timeout(Duration::from_secs(3), async {
        loop {
            if let Some(actual) = handle.capture_planning_actual(identity.clone()).await.unwrap() {
                break actual;
            }
            tokio::task::yield_now().await;
        }
    }).await.expect("real handle must publish the consumed Initial without a fake path identity");
    assert!(crate::replan_feedback::same_pose(&published.odometry, &before.odometry));
    assert_eq!(published.navigation.as_ref().unwrap().continuation().unwrap().semantic_key(), key);
    assert!(audit.lock().unwrap().snapshots.is_empty(), "publication must precede any local winner");
    let input = d1_input(published, goal, penalties);
    let route = input.compute_ground_route(&PctPlanner::default(), None, None)
        .expect("fixed consumed global request must remain feasible");
    let root = route.reference.full_window().unwrap().from().walk_state();
    assert_eq!(root.semantic_key(), key, "continued root retains source, original sample and actual XY");
    if expect_search {
        assert!(route.stats.expanded > 0, "the fixed nonempty penalty must enter the same original search");
        assert!(route.reference.connections().next().is_some(), "recorded root prefix survives reduction");
    }
    for pair in route.guide_body_poses.windows(2) {
        before.source.body_query().validate_unanchored_motion(pair[0], pair[1], &[], &mut None).unwrap();
    }
    assert_eq!(owner.ground_cursor(), cursor);
    assert_eq!(owner.refresh_execution_context(Twist2::ZERO).unwrap().navigation.as_ref()
        .unwrap().continuation().unwrap().semantic_key(), key, "planning never commits the original actual owner");
    let mut versioned = crate::tests::path_for("d01-consumed", 1);
    versioned.path = d1_path(&route);
    handle.submit_path(versioned).await.unwrap();
    let snapshot_result = tokio::time::timeout(Duration::from_secs(3), async {
        loop {
            if let Some(snapshot) = audit.lock().unwrap().snapshots.last().cloned() { break snapshot; }
            assert!(handle.status().last_terminal.is_none(), "a terminal failure is not a real admission");
            tokio::task::yield_now().await;
        }
    }).await;
    if snapshot_result.is_err() {
        let observed = { audit.lock().unwrap().clone() };
        eprintln!("D01 fixed admission diagnostic actual={:?} target={:?} route={:?} requests={:?} admissions={} snapshots={} commands={:?} events={} status={:?}",
            before.odometry, input.goal, route.guide_body_poses, observed.requests, observed.admissions.len(),
            observed.snapshots.len(), observed.commands, observed.events.len(), handle.status());
    }
    let snapshot = snapshot_result.expect("typed VersionedPath must reach real RouteFollower/PlanningOwner/Scan");
    {
        let observed = { audit.lock().unwrap().clone() };
        assert!(!observed.requests.is_empty());
        assert!(observed.requests.iter().all(|(_, ticket, ground)| ticket.is_some() && *ground));
        assert!(!observed.overflow);
        assert!(observed.events.is_empty());
        assert_eq!(observed.body_successes, 0);
    }
    assert_eq!(snapshot.path.poses.len(), snapshot.binding.edges().len() + 1);
    for (edge, pair) in snapshot.path.poses.windows(2).enumerate() {
        assert_eq!(snapshot.binding.edges()[edge].motion(),
            [body_control::body(&pair[0]), body_control::body(&pair[1])]);
    }
    let (edge, fraction) = snapshot.cursor.expect("real admitted owner has a cursor");
    for consumed in snapshot.binding.edges().iter().take(edge) {
        assert_eq!(consumed.motion()[0], consumed.motion()[1],
            "without odometry movement only real zero-motion semantic edges may be consumed");
    }
    if let Some(current) = snapshot.binding.edges().get(edge) {
        assert!(fraction == 0.0 || current.motion()[0] == current.motion()[1],
            "admission and planned coverage cannot consume nonzero actual motion");
    }
    assert_eq!(snapshot.binding.original_window().owner().original_goal().requested_ground_z,
        route.reference.original_goal().requested_ground_z);
    handle.cancel("d01-consumed").await.unwrap();
    handle.shutdown().await.unwrap();
    cleanup.0.take().unwrap().await.unwrap().unwrap();
    assert!(dropped.load(std::sync::atomic::Ordering::Acquire));
    route
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn d01_typed_ground_route_reaches_real_planning_owner() {
    use nuwax_nav_pct::ground_route::PlannedGroundAssociation as A;
    // Factory origin retains the old root mode. This is an existing exact
    // numerical comparison, called inside D01 rather than adding a selector.
    i07_prepared_start_and_original_target_keep_presence();
    let (source, _) = c3_direct_source();
    let q = f64::from(source.source_voxel_size_m());
    let (mut known, _, path, _) = c3_install(Arc::new(source), c3_start(), [0.1, 0.2], Some(0.0), c3_config());
    let edge = c4_first_motion(&path);
    d1_consume_fraction(&mut known, &path, edge, 0.5);
    let original = known.refresh_execution_context(Twist2::ZERO).unwrap();
    let state = original.navigation.as_ref().unwrap().continuation().unwrap();
    let sample = state.walk_state().reference_sample_xy();
    let actual_xy = state.walk_state().xy();
    assert_ne!(sample.map(|v| (v / q).floor() as i32), actual_xy.map(|v| (v / q).floor() as i32),
        "fixed Known midpoint +0.001 must actually cross a source column; do not retune");
    assert!(state.walk_state().planned_ground_z().is_some());
    let exact_route=d1_handle_round_trip(&mut known, d1_goal([0.1, 0.2], None), vec![], false).await;
    let end=exact_route.reference.full_window().unwrap().to();
    let (connection,sample,_)=end.indices();
    assert_eq!(exact_route.points.last().unwrap().xy,[0.1,0.2]);
    assert_eq!(exact_route.guide_body_poses.last().unwrap().0[..2],[0.1,0.2]);
    assert_eq!(end.walk_state().xy(),[0.1,0.2]);
    assert_eq!(end.walk_state().reference_sample_xy(),
        exact_route.reference.sample(connection,sample).unwrap().point.xy);
    assert_ne!(end.walk_state().reference_sample_xy(),end.walk_state().xy(),
        "fixed negative-to-positive endpoint separates original sample arithmetic from actual endpoint");
    let mut body_z_only = original.odometry.clone(); body_z_only.pose.position.z += 0.001;
    known.update_odometry(body_z_only); known.consume_ground_actual(None).unwrap();
    let after = known.refresh_execution_context(Twist2::ZERO).unwrap();
    assert_eq!(after.navigation.as_ref().unwrap().continuation().unwrap().semantic_key(), state.semantic_key());
    d1_handle_round_trip(&mut known, d1_goal([0.1, 0.2], None), vec![], false).await;

    let (source, height) = c3_known_gap_source();
    let q = f64::from(0.05_f32);
    let actual = ([0.5 * q, 0.5 * q, height + 0.46],
        [(0.1_f64 / 2.0).sin(), 0.0, 0.0, (0.1_f64 / 2.0).cos()]);
    let goal = [8.5 * q, 0.5 * q];
    let (mut carried, binding, path, _) = c3_install(Arc::new(source), actual, goal, Some(height), c3_config());
    let edge = (0..binding.candidate().connection_count()).find(|&edge|
        matches!(binding.candidate().sample(edge, 0).unwrap().association, A::Carried)
            && path.poses[edge].pose.position != path.poses[edge + 1].pose.position)
        .expect("unchanged known-gap winner has a real carried motion edge");
    d1_consume_fraction(&mut carried, &path, edge, 0.25);
    let penalties = vec![nuwax_nav_pct::RoutePenalty {
        center: [4.5 * q, 0.5 * q, height + 0.46], radius: q, weight: 1.0,
    }];
    let replanned = d1_handle_round_trip(&mut carried, d1_goal(goal, None), penalties.clone(), true).await;
    let connections: Vec<_> = replanned.reference.connections().collect();
    eprintln!("D01 CARRIED expansions={} connections={} guide_points={}; reducer shortcut only if real output demonstrates it",
        replanned.stats.expanded, connections.len(), replanned.guide_body_poses.len());
    // Fixed same-column center subcase: consume that actual pose through the
    // existing owner before replanning. No synthetic Reference/witness insertion.
    let current = carried.refresh_execution_context(Twist2::ZERO).unwrap();
    let mut center = current.odometry.clone();
    center.pose.position.x = (center.pose.position.x / q).floor().mul_add(q, 0.5 * q);
    center.pose.position.y = (center.pose.position.y / q).floor().mul_add(q, 0.5 * q);
    carried.update_odometry(center); carried.consume_ground_actual(None).unwrap();
    let center_basis=carried.refresh_execution_context(Twist2::ZERO).unwrap()
        .navigation.as_ref().unwrap().continuation().unwrap();
    let center_route = d1_handle_round_trip(&mut carried, d1_goal(goal, None), penalties, true).await;
    let prefix = center_route.reference.connections().next().unwrap();
    // The old zero-length-winner premise failed in execution-17 after actual
    // endpoint bits were preserved. Keep this input and qualify its real first
    // parent record; no particular equally legal winner shape is required.
    eprintln!("D01 real root prefix span={:?} steps={}",prefix.point_span,prefix.steps);
    assert_eq!(prefix.point_span[0],0);assert!(prefix.steps>0);
    let root = center_route.reference.full_window().unwrap().from().walk_state();
    assert_eq!(root.semantic_key(),center_basis.semantic_key());
    assert_eq!(root.reference_sample_xy(),center_basis.walk_state().reference_sample_xy());
    let ordinary = center_route.reference.position(0, prefix.steps, 0.0).unwrap().walk_state();
    assert_ne!(root.semantic_key(), ordinary.semantic_key(), "transient root and ordinary center cannot alias");

    let (source, mut metadata_blocks) = c3_direct_source();
    let actual = ([100.0, 100.0, 0.55], [0.0, 0.0, 0.0, 1.0]);
    let (mut unknown, _, path, _) = c3_install(Arc::new(source), actual, [100.1, 100.0], None, c3_config());
    d1_consume_fraction(&mut unknown, &path, c4_first_motion(&path), 0.25);
    for z in [0.55, 0.56] {
        let mut actual = unknown.odometry.clone().unwrap(); actual.pose.position.z = z;
        unknown.update_odometry(actual); unknown.consume_ground_actual(None).unwrap();
        for requested in [None, Some(0.0)] {
            let route = d1_handle_round_trip(&mut unknown, d1_goal([100.1, 100.0], requested), vec![], false).await;
            assert_eq!(route.reference.original_goal().requested_ground_z, requested);
            assert!(route.points.iter().all(|point| point.ground_z.is_none()));
            for (edge, connection) in route.reference.connections().enumerate() {
                for index in 0..=connection.steps {
                    assert!(matches!(route.reference.sample(edge, index).unwrap().association, A::Unanchored));
                }
            }
            let context = unknown.refresh_execution_context(Twist2::ZERO).unwrap();
            let key = context.navigation.as_ref().unwrap().continuation().unwrap().semantic_key();
            let mut input = d1_input(context, d1_goal([100.1, 100.0], requested), vec![]);
            let cancelled = std::sync::atomic::AtomicBool::new(true);
            assert!(input.compute_ground_route(&PctPlanner::default(), Some(&cancelled), None).is_err());
            let original_source = input.source.clone();
            let mut observations = original_source.observations().unwrap().as_ref().clone();
            for (&block, values) in &mut metadata_blocks {
                for value in values.values_mut() { value.observed_ns = 900; value.hits = 17; }
                assert!(observations.replace_block_with_geometry_changes(block, values.clone()).unwrap().is_empty());
            }
            observations.mark_complete_geometry_source();
            input.source = Arc::new(original_source.with_observations(Some(Arc::new(observations.clone()))));
            assert!(nuwax_nav_pct::same_source_geometry(&original_source, &input.source));
            let compatible = input.compute_ground_route(&PctPlanner::default(), None, None).unwrap();
            assert_eq!(compatible.reference.full_window().unwrap().from().walk_state().semantic_key(), key);
            let changed_key = [0_i32, 2, 4];
            let block_key = changed_key.map(|v| v.div_euclid(32));
            let mut changed_block = metadata_blocks.get(&block_key).cloned().unwrap_or_default();
            changed_block.insert(changed_key, SurfaceObservation { observed_ns: 901, hits: 2 });
            assert!(!observations.replace_block_with_geometry_changes(block_key, changed_block).unwrap().is_empty());
            observations.mark_complete_geometry_source();
            input.source = Arc::new(original_source.with_observations(Some(Arc::new(observations))));
            assert!(input.compute_ground_route(&PctPlanner::default(), None, None).is_err(), "true source retirement rejects the retained basis");
            input.source = Arc::new(c3_direct_source().0);
            assert!(input.compute_ground_route(&PctPlanner::default(), None, None).is_err(), "foreign source cannot reopen a retained owner");
            assert_eq!(unknown.refresh_execution_context(Twist2::ZERO).unwrap().navigation.as_ref()
                .unwrap().continuation().unwrap().semantic_key(), key);
        }
    }
}

#[test]
fn d02_actual_source_distinguishes_two_feasible_navigation_states() {
    use nuwax_nav_types_messages::GroundLayerResult as R;
    // Exact approved third fixture: q=.05; lower member0/bin7; upper member11
    // bin0 -> bin7 at x>=1; physical z=.705; offset=.4; step=.21.
    let source = Arc::new(c3_bin_shift_source());
    let xy = [0.025, 0.025]; let to = [0.125, 0.025];
    let actual = ([xy[0], xy[1], 0.705], [0.0, 0.0, 0.0, 1.0]);
    let records: Vec<_> = source.observations().unwrap().ground_records([0, 0]).collect();
    assert_eq!(records.len(), 2);
    let heights = [records[0].height_at(xy).unwrap(), records[1].height_at(xy).unwrap()];
    assert!(heights[0] < heights[1]);
    let planner = PctPlanner::default();
    let lower = c3_route(&planner, &source, actual, Some(heights[0]), to);
    let upper = c3_route(&planner, &source, actual, Some(heights[1]), to);
    let low = lower.reference.full_window().unwrap().from().walk_state();
    let high = upper.reference.full_window().unwrap().from().walk_state();
    assert_ne!(low.semantic_key(), high.semantic_key());
    let query = low.bind_body_query(&source).unwrap();
    let (physical, independent) = query.start_ground_state_with_evidence(actual).unwrap();
    assert!(independent.is_direct(), "original fixture must independently select a real actual source");
    let low_response = c3_project(&query, low, actual, &physical, to);
    let high_response = c3_project(&query, high, actual, &physical, to);
    assert_ne!(low_response.pose(), high_response.pose());
    // The real owner compares independent actual-source selection only after
    // the original physical validation; neither planner layer is an authority.
    let mut low_owner = d1_typed_initial(source.clone(), actual, Some(heights[0]), d1_goal(to, None));
    let mut high_owner = d1_typed_initial(source.clone(), actual, Some(heights[1]), d1_goal(to, None));
    let relations = [d1_relation(&mut low_owner), d1_relation(&mut high_owner)];
    assert_eq!(relations.iter().filter(|&&r| r == R::Matched).count(), 1);
    assert_eq!(relations.iter().filter(|&&r| r == R::KnownConflict).count(), 1,
        "both sweeps are feasible but only the independent actual association may match");
    let actuals = [low_owner.refresh_execution_context(Twist2::ZERO).unwrap(),
        high_owner.refresh_execution_context(Twist2::ZERO).unwrap()];
    assert_eq!(actuals[0].ground, actuals[1].ground);
    assert_eq!(actuals[0].ground_evidence.as_ref().unwrap().observation,
        actuals[1].ground_evidence.as_ref().unwrap().observation);
    assert_eq!(actuals[0].ground_evidence.as_ref().unwrap().observation, independent);
    // On this same original third geometry, consume the unchanged association
    // through each actual known column before taking one adjacent step outside
    // its y=-8 boundary. This checks symmetric carried retention without a new
    // two-layer fixture or manufactured result evidence.
    for (owner, expected) in [(&mut low_owner, relations[0]), (&mut high_owner, relations[1])] {
        for column in (-8..=0).rev() {
            let pose = ([actual.0[0], (f64::from(column) + 0.5) * f64::from(0.05_f32), actual.0[2]], actual.1);
            owner.update_odometry(c3_stamped(pose)); owner.consume_ground_actual(None).unwrap();
            assert_eq!(d1_relation(owner), expected);
        }
        let outside = ([actual.0[0], -8.5 * f64::from(0.05_f32), actual.0[2]], actual.1);
        owner.update_odometry(c3_stamped(outside)); owner.consume_ground_actual(None).unwrap();
        let context = owner.refresh_execution_context(Twist2::ZERO).unwrap();
        assert_eq!(context.ground.query_state(), nuwax_nav_mls_planner::surface::GroundQueryState::Unknown);
        assert_eq!(context.ground_evidence.as_ref().unwrap().relation, expected,
            "pure carried omission preserves a proven KnownConflict as well as Same");
    }
}

// Exact original MLS actual-ground fixture, copied without numeric changes.
fn d1_original_actual_step_scene(levels:impl Fn(i32)->Vec<i32>,measured:bool)->SurfaceSnapshot {
    use nuwax_map_chunks::{LayerObservation,LayerObservations,LayerSample,block_key,local_key};
    const Q:f32=0.125;
    let mut points=Vec::new();
    let mut blocks=BTreeMap::<[i32;3],BTreeMap<[i32;3],SurfaceObservation>>::new();
    let mut layers=BTreeMap::<[i32;3],LayerObservations>::new();
    for x in 0..16 {for y in 0..8 {for z in levels(x) {
        let key=[x,y,z];let local=local_key(key);
        let origin=key.map(|v|(f64::from(v)+0.5)*f64::from(Q));
        points.push((origin[0] as f32,origin[1] as f32,origin[2] as f32));
        blocks.entry(block_key(key)).or_default().insert(key,SurfaceObservation{observed_ns:100,hits:3});
        if measured {
            layers.entry(block_key(key)).or_default().insert(local,LayerObservation {
                members:vec![local],normal:[0.0,0.0,1.0],plane_origin:origin,
                samples:(0..3).map(|i|LayerSample {source:Default::default(),point:origin,
                    source_sequence:i+1,source_timestamp_ns:100+i,point_index:u32::from(local)}).collect(),
            });
        }
    }}}
    let config=Config {world_frame:"map".into(),voxel_size:0.25,robot_height:0.2,
        surface_closing_radius:0.0,node_spacing_m:0.5,wall_clearance_m:0.0,
        wall_buffer_m:0.0,wall_buffer_weight:0.0,max_slope_deg:18.0,
        step_threshold_m:0.2,min_step_tread_depth_m:0.5,step_penalty_weight:1.0,
        goal_tolerance:0.1,viz_publish_hz:0.0};
    let model=SurfaceBodyEnvelope {fixed_collision:None,
        footprint_xy_m:vec![[-0.08,-0.06],[0.08,-0.06],[0.08,0.06],[-0.08,0.06]],
        z_bounds_m:[-0.1,0.1],body_to_support_z_m:0.4,max_step_up_m:0.2,max_step_down_m:0.2,
        max_cross_slope_deg:18.0,support_uncertainty_m:0.0};
    let mut observations=SurfaceObservations::new(Q,32).unwrap();
    for (block,values) in blocks {observations.replace_block(block,values).unwrap();}
    for (block,values) in layers {observations.replace_ground_layers(block,Arc::new(values)).unwrap();}
    observations.mark_complete_geometry_source();
    SurfaceSnapshot::from_points(&points,config,Q).unwrap().with_body_envelope(model).unwrap()
        .with_observations(Some(Arc::new(observations)))
}


// Exact original Scan scene_with_extra_cells; called only with (Some(0),0,&[]).
    fn d1_original_prefix_scene(door: Option<i32>, slope: f64, extra: &[[i32; 3]]) -> SurfaceSnapshot {
        let size = 0.01_f32;
        let max_step = if slope == 0.0 { 0.0 } else { 0.15 };
        let mut cells: Vec<[i32; 3]> = (-130..131)
            .flat_map(|x| {
                (-70..100).map(move |y| [x, y, ((f64::from(x) + 0.5) * slope - 0.5).floor() as i32])
            })
            .collect();
        if let Some(width) = door {
            cells.extend((0..4).flat_map(|x| {
                (-70..100)
                    .filter(move |&y| y < 0 || y >= width)
                    .flat_map(move |y| (0..65).map(move |z| [x, y, z]))
            }));
        }
        cells.extend_from_slice(extra);
        let points: Vec<_> = cells
            .iter()
            .map(|p| {
                (
                    (p[0] as f32 + 0.5) * size,
                    (p[1] as f32 + 0.5) * size,
                    (p[2] as f32 + 0.5) * size,
                )
            })
            .collect();
        let mut blocks = BTreeMap::<[i32; 3], BTreeMap<[i32; 3], SurfaceObservation>>::new();
        for key in cells {
            blocks
                .entry(key.map(|v| v.div_euclid(32)))
                .or_default()
                .insert(
                    key,
                    SurfaceObservation {
                        observed_ns: 1,
                        hits: 2,
                    },
                );
        }
        let mut observations = SurfaceObservations::new(size, 32).unwrap();
        for (key, values) in blocks {
            observations.replace_block(key, values).unwrap();
        }
        observations.mark_complete_geometry_source();
        let cfg = Config {
            world_frame: "map".into(),
            voxel_size: 0.1,
            robot_height: 0.55,
            surface_closing_radius: 0.0,
            node_spacing_m: 0.3,
            wall_clearance_m: 0.0,
            wall_buffer_m: 0.1,
            wall_buffer_weight: 2.0,
            max_slope_deg: 18.0,
            step_threshold_m: max_step,
            min_step_tread_depth_m: 0.2,
            step_penalty_weight: 1.0,
            goal_tolerance: 0.1,
            viz_publish_hz: 0.0,
        };
        SurfaceSnapshot::from_points(&points, cfg, size)
            .unwrap()
            .with_body_envelope(SurfaceBodyEnvelope {
                fixed_collision: None,
                footprint_xy_m: vec![[-0.3, -0.16], [0.3, -0.16], [0.3, 0.16], [-0.3, 0.16]],
                z_bounds_m: [-0.15, 0.15],
                body_to_support_z_m: 0.4,
                max_step_up_m: f64::from(max_step),
                max_step_down_m: f64::from(max_step),
                max_cross_slope_deg: 18.0,
                support_uncertainty_m: 0.0,
            })
            .unwrap()
            .with_observations(Some(Arc::new(observations)))
    }

#[test]
fn d04_unknown_continuity_and_height_presence_remain_distinct() {
    use nuwax_nav_mls_planner::surface::GroundQueryState;
    use nuwax_nav_types_messages::GroundLayerResult as R;
    use crate::replan_feedback::{same_actual, SuccessfulReplanMemo};
    let (source, height) = c3_known_gap_source();
    let source = Arc::new(source); let q = f64::from(0.05_f32);
    // The existing gap source needs actual support in its thin [0,0] column.
    // D01 retains its original roll=.1 control; this relation control uses
    // the existing identity attitude so the independent source is really Known.
    let actual = ([0.5 * q, 0.5 * q, height + 0.46], [0.0, 0.0, 0.0, 1.0]);
    let goal = d1_goal([8.5 * q, 0.5 * q], None);
    let mut carried = d1_typed_initial(source.clone(), actual, Some(height), goal.clone());
    assert_eq!(d1_relation(&mut carried), R::Matched);
    let mut unknown = actual; unknown.0[0] = 2.5 * q;
    carried.update_odometry(c3_stamped(unknown)); carried.consume_ground_actual(None).unwrap();
    let context = carried.refresh_execution_context(Twist2::ZERO).unwrap();
    assert_eq!(context.ground.query_state(), GroundQueryState::Unknown);
    assert_eq!(context.ground.height().to_bits(), height.to_bits());
    assert_eq!(context.ground_evidence.as_ref().unwrap().relation, R::Matched,
        "a real pure carry retains the earlier independently proven match");
    assert!(context.ground_evidence.as_ref().unwrap().carry_intact);
    let mut unanchored = d1_typed_initial(source, unknown, None, goal);
    assert_eq!(d1_relation(&mut unanchored), R::Unresolved,
        "same physical Unknown pose without an independent navigation anchor cannot mint a match");
    assert!(unanchored.refresh_execution_context(Twist2::ZERO).unwrap().navigation.as_ref()
        .unwrap().continuation().unwrap().walk_state().planned_ground_z().is_none());

    // Exact original actual_unknown_after_crossing_known_step input, including
    // legacy and measured versions. Intermediate actual source is preserved
    // even though endpoint query_state is Unknown.
    for measured in [false, true] {
        let map = Arc::new(d1_original_actual_step_scene(|x| vec![if x < 8 { 0 } else { 1 }], measured));
        let from = ([0.5625, 0.4375, 0.5], [0.0, 0.0, 0.0, 1.0]);
        let crossed_pose = ([1.5625, 0.4375, 0.65], from.1);
        let endpoint = ([2.4375, 0.4375, 0.75], from.1);
        let query = map.body_query();
        let (initial, _) = query.start_ground_state_with_evidence(from).unwrap();
        let crossed = query.start_ground_state(crossed_pose).unwrap();
        let fresh = query.start_ground_state(endpoint).unwrap();
        let (observed, summary) = query.observe_actual_ground_with_evidence(from, &initial, endpoint).unwrap();
        assert_eq!(fresh.query_state(), GroundQueryState::Unknown);
        assert_ne!(fresh.height().to_bits(), crossed.height().to_bits());
        assert_eq!(observed.query_state(), GroundQueryState::Unknown);
        assert_eq!(observed.height().to_bits(), crossed.height().to_bits());
        assert_ne!(observed.height().to_bits(), initial.height().to_bits());
        assert!(!summary.carry_intact(), "endpoint Unknown cannot hide the real crossed Known step");

        let goal = d1_goal([endpoint.0[0], endpoint.0[1]], None);
        let mut owner = d1_typed_initial(map, from, Some(initial.height()), goal.clone());
        // Only native/observed full-source identity can prove Same. Legacy
        // identity-free records deliberately remain unresolved.
        if measured { assert_eq!(d1_relation(&mut owner), R::Matched); }
        let before = owner.refresh_execution_context(Twist2::ZERO).unwrap();
        let before_input = d1_input(before.clone(), goal.clone(), vec![]);
        let mut memo = SuccessfulReplanMemo::default();
        memo.published(7_u64, before_input.clone());
        assert!(memo.should_reuse(Some((&7_u64, before.as_ref())), &before_input));
        owner.update_odometry(c3_stamped(crossed_pose));
        let at_b = owner.refresh_execution_context(Twist2::ZERO).unwrap();
        assert!(at_b.navigation.is_none(), "context refresh does not consume A to B");
        assert!(!at_b.ground_evidence.as_ref().unwrap().carry_intact);
        owner.update_odometry(c3_stamped(endpoint));
        let at_c = owner.refresh_execution_context(Twist2::ZERO).unwrap();
        assert_eq!(at_c.ground.height().to_bits(), crossed.height().to_bits());
        assert_eq!(at_c.ground.query_state(), GroundQueryState::Unknown);
        assert!(!at_c.ground_evidence.as_ref().unwrap().carry_intact,
            "A->B->C read-only refresh must AND the intermediate reassociation");
        assert!(at_c.navigation.is_none());
        // Return to the exact original actual pose without a nav commit. All
        // old memo inputs now agree except the genuinely observed interval
        // summary. No test fabricates Matched or edits evidence fields.
        owner.update_odometry(c3_stamped(from));
        let returned = owner.refresh_execution_context(Twist2::ZERO).unwrap();
        assert!(crate::replan_feedback::same_pose(&returned.odometry, &before.odometry));
        assert_eq!(returned.ground, before.ground);
        assert_eq!(returned.navigation, before.navigation);
        assert_eq!(returned.applied, before.applied);
        assert!(returned.source.same_body_query_view(&before.source));
        assert_ne!(returned.ground_evidence, before.ground_evidence);
        assert!(!same_actual(&returned, &before));
        let returned_input = d1_input(returned.clone(), goal.clone(), vec![]);
        assert!(!memo.should_reuse(Some((&7_u64, returned.as_ref())), &returned_input),
            "changed real evidence invalidates Ground memo even at the same physical/nav pose");
        // Repeat the same fixed B/C observations and finally use the sole
        // actual-consumption path. The old Matched evidence cannot survive.
        owner.update_odometry(c3_stamped(crossed_pose)); owner.refresh_execution_context(Twist2::ZERO).unwrap();
        owner.update_odometry(c3_stamped(endpoint)); owner.refresh_execution_context(Twist2::ZERO).unwrap();
        owner.consume_ground_actual(None).unwrap();
        assert_eq!(d1_relation(&mut owner), R::Unresolved);
    }

    // Requested None/0/positive/negative are distinct original inputs on the
    // existing Unanchored scene; none becomes body Z minus calibration.
    let (source, _) = c3_direct_source(); let source = Arc::new(source);
    let actual = ([100.0, 100.0, 0.55], [0.0, 0.0, 0.0, 1.0]);
    let mut owner = d1_typed_initial(source, actual, None, d1_goal([100.1, 100.0], None));
    let context = owner.refresh_execution_context(Twist2::ZERO).unwrap();
    let mut inputs = Vec::new();
    for requested in [None, Some(0.0), Some(0.3), Some(-0.3)] {
        let input = d1_input(context.clone(), d1_goal([100.1, 100.0], requested), vec![]);
        let route = input.compute_ground_route(&PctPlanner::default(), None, None).unwrap();
        assert_eq!(route.reference.original_goal().requested_ground_z, requested);
        assert!(route.points.iter().all(|point| point.ground_z.is_none()));
        assert_eq!(GroundIntent::new(route.reference.full_window().unwrap()).goal().requested_ground_z, requested);
        for earlier in &inputs { assert!(!input.same_complete_input(earlier)); }
        inputs.push(input);
    }
}

#[test]
fn d05_only_consumed_original_terminal_emits_arrival() {
    use nuwax_nav_scan::{GroundProgressPrefixPolicy, CandidateEdgeOrigin};
    // Original progress_prefix_fixture, its original long Scan reference,
    // boundary, arrival, default 256 budget and +.375m real PCT owner. Never
    // prepare the historical 53,280-expansion full global goal.
    let source = Arc::new(d1_original_prefix_scene(Some(0), 0.0, &[]));
    let tomo = Tomogram::from_surface(&source).unwrap();
    let actual = c3_start(); let reference = [[-1.0, 0.2], [1.0, 0.2]];
    let boundary = [[-1.5, -0.5], [1.5, -0.5], [1.5, 0.9], [-1.5, 0.9]];
    let route = c3_route(&PctPlanner::default(), &source, actual, None, [actual.0[0] + 0.375, actual.0[1]]);
    let intent = GroundIntent::new(route.reference.full_window().unwrap());
    let initial_key = intent.window().from().walk_state().semantic_key();
    let physical = source.body_query().start_ground_state(actual).unwrap();
    assert_eq!(ScanConfig::default().search_expansion_limit, 256);
    let mapped = nuwax_nav_scan::plan_driven_ground(&source, &tomo, actual, &physical,
        &reference, &c3_motion(), &ScanConfig::default(), None, Some(([1.0, 0.2], 0.1)),
        &boundary, false, false, Some(GroundProgressPrefixPolicy {
            reference: &reference, minimum_distance_m: 0.375,
        }), intent.window()).expect("fixed unchanged short-window partial prerequisite");
    assert!(mapped.trajectory.partial); assert_eq!(mapped.binding.family(), CandidateFamily::Search);
    assert!(!mapped.binding.planned_join()); assert!(!mapped.binding.covers_original_goal());
    assert!(mapped.binding.edges().iter().any(|edge| matches!(edge.origin(), CandidateEdgeOrigin::Parent { .. })));
    assert!(!mapped.binding.edges().iter().any(|edge| matches!(edge.origin(), CandidateEdgeOrigin::Connector { .. })));
    assert_eq!(intent.window().from().walk_state().semantic_key(), initial_key);
    let binding = Arc::new(mapped.binding);
    let path = Path3D { poses: mapped.trajectory.poses.into_iter().map(c3_stamped).collect() };
    let initial = intent.window().from().continuation([actual.0[0], actual.0[1]]).unwrap();
    let mut owner = d1_captured_owner(source, actual, initial,
        d1_goal(route.reference.original_goal().xy, None));
    owner.configure_boundary(&boundary);
    owner.admit_ground_candidate(path.clone(), path.poses.last().unwrap().clone(), binding.clone(), None, None).unwrap();
    assert!(owner.ground_arrival_step(&intent).unwrap().is_none(), "planned coverage is never an actual terminal");
    let installed = c4_binding(&owner);
    assert!(!installed.planned_join());
    c5_consume_path(&mut owner, &path);
    assert!(matches!(owner.step_ground_internal(0.2, None).unwrap(),
        ground_cursor::GroundControlStep::LocalReferenceEnd { covers_original_goal: false }));
    assert!(owner.ground_arrival_step(&intent).unwrap().is_none(), "real partial local end is still not original terminal");
    assert!(GroundIntent::new(installed.original_window()).same_intent(&intent));
    let end = owner.ground_cursor();
    for _ in 0..2 {
        owner.consume_ground_actual(None).unwrap();
        assert_eq!(owner.ground_cursor(), end);
        assert!(owner.ground_arrival_step(&intent).unwrap().is_none());
    }

    // Separate genuine RouteFollower window-advance control. This does not
    // claim partial -> full-goal/ReferenceOnly coverage. Freeze a 1m horizon
    // before running on the original 1.1m c3 straight; budget/model unchanged.
    let (source, _) = c3_direct_source(); let source = Arc::new(source);
    let mut config = c3_config(); config.scan.horizon_m = 1.0;
    // Window advancement executes the real controller; the earlier no-motion-
    // calibration failure is retained. Use the existing complete calibration.
    config.motion_limits=Some(c3_limits());
    assert_eq!(config.lookahead_m, 0.5);
    let prepared = config.pct.prepare_ground_start(&source,
        GroundRouteStart { xy: [-1.0, 0.2], ground_z: None }, None).unwrap();
    let capture = d1_initial_capture(source.clone(), actual, prepared.basis().clone());
    let route = config.pct.ground_route_from_start(&source, actual, &prepared,
        GroundRouteGoal { xy: [0.1, 0.2], requested_ground_z: None }, 0.0,
        &[], None, None, SearchLimits::default(), &[]).unwrap();
    let goal = d1_goal([0.1, 0.2], None);
    let mut route_owner = crate::RouteFollower::new(config).unwrap();
    route_owner.configure_execution_goal(&goal, d1_options(), Some(capture)).unwrap();
    route_owner.update_execution_map(ExecutionMap::Surface(source)).unwrap();
    route_owner.update_odometry(c3_stamped(actual));
    route_owner.consume_initial_actual_for_planning().unwrap();
    route_owner.replace_typed_execution_path(d1_path(&route), None, goal).unwrap();
    let first = route_owner.ground_window_for_test().unwrap();
    assert!(!first.window().to().is_terminal(), "fixed 1m horizon must cut the original 1.1m route");
    let initial_progress = route_owner.progress_m();
    c3_until(|| {
        let step = route_owner.step(0.2);
        assert!(!matches!(step, PlannerStep::GroundArrival(_) | PlannerStep::Succeeded));
        route_owner.ground_binding_for_test().is_some()
    });
    let binding = route_owner.ground_binding_for_test().unwrap();
    eprintln!("D05 fixed window diagnostic from={:?} to={:?} planned_join={} family={:?} partial_path={:?} wait={:?}",
        first.window().from().walk_state().xy(), first.window().to().walk_state().xy(),
        binding.planned_join(), binding.family(), route_owner.local_path(), route_owner.execution_wait_reason());
    let last=binding.edges().last().unwrap().motion()[1].0;
    let candidate_end=binding.candidate().walk_state_at(binding.candidate().connection_count()-1,1.0,[last[0],last[1]]).unwrap();
    eprintln!("D05 exact join diagnostic end_matches={} candidate={:?} original={:?} candidate_sample={:?} original_sample={:?}",
        binding.candidate().end_matches(first.window().to()),candidate_end.semantic_key(),
        first.window().to().walk_state().semantic_key(),candidate_end.reference_sample_xy(),
        first.window().to().walk_state().reference_sample_xy());
    assert!(binding.planned_join());
    assert!(GroundIntent::new(binding.original_window()).same_intent(&GroundIntent::new(first.window())));
    assert_eq!(route_owner.progress_m(), initial_progress, "planned join cannot advance an unfinished window");
    let path = route_owner.local_path().cloned().unwrap();
    assert_eq!(path.poses.len(), binding.edges().len() + 1);
    // A future original segment is nearby at the first endpoint but cannot
    // become arrival before actual adjacent consumption of its own window.
    for pose in path.poses.iter().skip(1) {
        route_owner.update_odometry(pose.clone());
        let step = route_owner.step(0.2);
        assert!(!matches!(step, PlannerStep::GroundArrival(_) | PlannerStep::Succeeded));
    }
    let second = route_owner.ground_window_for_test().unwrap();
    assert_eq!(second.window().from().indices(), first.window().to().indices());
    assert!(route_owner.progress_m() > initial_progress);
    let progress = route_owner.progress_m();
    for _ in 0..2 {
        let step = route_owner.step(0.2);
        assert!(!matches!(step, PlannerStep::GroundArrival(_) | PlannerStep::Succeeded));
        assert_eq!(route_owner.progress_m(), progress, "the completed actual binding advances exactly once");
        assert_eq!(route_owner.ground_window_for_test().unwrap().window().from().indices(),
            second.window().from().indices());
    }
    route_owner.stop(); // original worker cancellation/join remains in Drop
}
