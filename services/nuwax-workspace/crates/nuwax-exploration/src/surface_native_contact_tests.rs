// Fixed native counts and Writer/Receiver fixture ported from
// nuwax-nav-pct/src/heightfield_tests.rs:21-203 at the frozen A/B tree.
// No fake projection error, injected poll, scan hook, or new contact model.
mod native_contact_unavailable_tests {
use super::*;
use nuwax_map_chunks::heightfield_observations::{HEIGHT_SUB_BINS, HeightfieldColumn};
use nuwax_map_chunks::{ColumnGroundRecords, Domain, GroundRecordRef, HeightfieldSurface,
    Key, Receiver, Repository, Transfer, Writer, global_key};
use nuwax_nav_mls_planner::{SurfaceBodyEnvelope, SurfaceObservation, SurfaceObservations};
use std::collections::{BTreeMap, BTreeSet};

const Q: f32 = 0.05;
const BUDGET: usize = 4 * 1024 * 1024;

fn q() -> f64 {
    f64::from(Q)
}

fn center(xy: [i32; 2]) -> [f64; 2] {
    xy.map(|v| (f64::from(v) + 0.5) * q())
}

fn point(xy: [i32; 2], z: f64) -> [f64; 3] {
    let [x, y] = center(xy);
    [x, y, z]
}

fn column(
    xy: [i32; 2],
    counts: impl IntoIterator<Item = (i32, [u32; HEIGHT_SUB_BINS])>,
) -> HeightfieldColumn {
    HeightfieldColumn::from_counts(xy, Q, counts.into_iter().collect())
        .unwrap()
        .unwrap()
}

fn thin(xy: [i32; 2], z: i32) -> HeightfieldColumn {
    column(xy, [(z, [0, 0, 0, 1, 0, 0, 0, 0])])
}

fn native(columns: Vec<HeightfieldColumn>) -> ColumnGroundRecords {
    ColumnGroundRecords {
        classified_empty: false, legacy: vec![],
        heightfields: columns
            .into_iter()
            .map(|column| HeightfieldSurface {
                column,
                samples: vec![],
            })
            .collect(),
    }
}

fn config(body_height: f32) -> Config {
    Config {
        world_frame: "map".into(),
        voxel_size: 0.1,
        robot_height: body_height,
        surface_closing_radius: 0.0,
        node_spacing_m: 0.3,
        wall_clearance_m: 0.0,
        wall_buffer_m: 0.0,
        wall_buffer_weight: 0.0,
        max_slope_deg: 0.0,
        step_threshold_m: 0.0,
        min_step_tread_depth_m: 0.2,
        step_penalty_weight: 1.0,
        goal_tolerance: 0.1,
        viz_publish_hz: 0.0,
    }
}

struct Fixture {
    source: SurfaceSnapshot,
}

fn scene(
    columns: Vec<([i32; 2], ColumnGroundRecords)>,
    unclaimed: impl IntoIterator<Item = Key>,
    cfg: Config,
    body: Option<SurfaceBodyEnvelope>,
) -> Fixture {
    let mut keys: BTreeSet<_> = unclaimed.into_iter().collect();
    for (xy, records) in &columns {
        for surface in &records.heightfields {
            assert_eq!(surface.column.xy(), *xy);
            keys.extend(surface.column.members().map(|(z, _)| [xy[0], xy[1], z]));
        }
        for (block, layer) in &records.legacy {
            keys.extend(
                layer
                    .members
                    .iter()
                    .map(|&local| global_key(*block, local).unwrap()),
            );
        }
    }
    assert!(!keys.is_empty(), "夹具必须有真实原始占据");
    let mut writer = Writer::new(Repository::temporary().unwrap(), Q, BUDGET).unwrap();
    writer.observe(keys.iter().copied(), 20).unwrap();
    for (xy, records) in &columns {
        writer.replace_ground_column(*xy, records).unwrap();
    }
    let root = writer
        .snapshot(
            Domain {
                session: "pct-heightfield-contract".into(),
                generation: 1,
                frame_generation: 0,
                producer_generation: 1,
            },
            1,
            20,
        )
        .unwrap();
    let mut receiver = Receiver::new(Repository::temporary().unwrap());
    let admitted = receiver
        .receive(
            &Transfer {
                root: root.encode().unwrap(),
                included: BTreeMap::new(),
            },
            &root.domain,
            |_, reference| {
                writer
                    .repository()
                    .read_object(reference.object, nuwax_map_chunks::MAX_BLOCK_BYTES)
            },
        )
        .unwrap();

    let mut observations = SurfaceObservations::new(Q, 32).unwrap();
    let mut parts = Vec::new();
    for reference in &admitted.blocks {
        let (block, records) = receiver.ground_parts(reference).unwrap();
        observations
            .replace_block(
                reference.key,
                block
                    .iter()
                    .map(|(&local, evidence)| {
                        (
                            global_key(reference.key, local).unwrap(),
                            SurfaceObservation {
                                observed_ns: evidence.observed_ns,
                                hits: evidence.hits,
                            },
                        )
                    })
                    .collect(),
            )
            .unwrap();
        let legacy = records
            .iter()
            .filter_map(|(local, record)| match record {
                GroundRecordRef::LegacyPlane(layer) => Some((local, layer.clone())),
                GroundRecordRef::HeightfieldV1(_) | GroundRecordRef::ClassifiedEmptyColumn => None,
            })
            .collect();
        observations
            .replace_ground_layers(reference.key, Arc::new(legacy))
            .unwrap();
        parts.push((reference.key, records));
    }
    let block_xys: BTreeSet<_> = parts.iter().map(|(key, _)| [key[0], key[1]]).collect();
    for block_xy in block_xys {
        let fragments: Vec<_> = parts
            .iter()
            .filter(|(key, _)| key[..2] == block_xy)
            .cloned()
            .collect();
        observations.replace_heightfield_columns(block_xy, &fragments);
    }
    observations.mark_complete_geometry_source();
    let points: Vec<_> = keys
        .iter()
        .map(|key| {
            let p = key.map(|v| ((f64::from(v) + 0.5) * q()) as f32);
            (p[0], p[1], p[2])
        })
        .collect();
    let mut source = SurfaceSnapshot::from_points(&points, cfg, Q)
        .unwrap()
        .with_observations(Some(Arc::new(observations)));
    if let Some(body) = body {
        source = source.with_body_envelope(body).unwrap();
    }
    Fixture { source }
}

const NATIVE_REASON: &str = "NativeContactGeometryUnavailable: 原生地表成员与参考外形重叠，有限接触几何尚未提供；未证明碰撞或可通行";

#[test]
fn native_planar_frontiers_keep_unavailable_across_prepared_cursor_and_repeat() {
    // The same thick native member and calibrated reference as the existing
    // heightfield_member_overlap_requires_contact_geometry_independent_of_step_limit.
    let original = column([0, 0], [(0, [1; 8])]);
    let mut cfg = config(0.3);
    cfg.wall_clearance_m = 0.03;
    let snapshot = Arc::new(scene(vec![([0, 0], native(vec![original.clone()]))], [], cfg, None).source);
    let reference = point([0, 0], original.height());
    let map = nuwax_nav_pct::Tomogram::from_surface(&snapshot).unwrap();
    assert_eq!(map.project_reference_motion(reference, center([0, 0]), &[]).unwrap_err(), NATIVE_REASON);
    assert_eq!(map.project_reference_motion_typed(reference, center([0, 0]), &[]).unwrap_err(),
        nuwax_nav_pct::ReferenceMotionFailure::NativeContactGeometryUnavailable);
    let receiver = SurfaceInputReceiver::new();
    receiver.observe(snapshot.clone()).unwrap();
    let input = receiver.freeze().unwrap().fork_for_planning();
    let mut adapter = source(receiver, 1);
    let mut pose = state(reference[2] + 0.3, 1);
    pose.robot_pose.pose.position = Point3::new(reference[0], reference[1], reference[2] + 0.3);
    let candidates = frontier_geometry(&snapshot, reference, &[], None).unwrap().candidates.len();
    assert!(candidates > 4, "fixed fixture must require a real route-budget continuation");
    let before = committed(&adapter);
    let prepared = adapter.prepare_poll_with_planning_input(&pose, &input).unwrap();
    assert!(!matches!(prepared.outcome(), Ok(ExplorationPoll::NoFrontier | ExplorationPoll::Goal(_))));
    assert_eq!(committed(&adapter), before);
    adapter.discard_prepared_poll().unwrap();
    assert_eq!(committed(&adapter), before);
    let mut saw_continuation = false;
    let mut result = ExplorationPoll::Pending;
    // The fixed candidate count bounds cursor progress; no arbitrary timeout.
    for _ in 0..=candidates {
        adapter.prepare_poll_with_planning_input(&pose, &input).unwrap();
        result = adapter.commit_prepared_poll().unwrap();
        assert!(!adapter.last_no_frontier);
        assert!(adapter.selected.is_none());
        match &result {
            ExplorationPoll::Pending => {
                saw_continuation = true;
                assert_eq!(adapter.deferred_route.as_ref().and_then(|route| route.source_unavailable.as_ref()),
                    Some(&nuwax_nav_pct::ReferenceMotionFailure::NativeContactGeometryUnavailable));
            }
            ExplorationPoll::SourceUnavailable(reason) => { assert_eq!(reason, NATIVE_REASON); break; }
            other => panic!("native uncertainty must not become completion or obstacle: {other:?}"),
        }
    }
    assert!(saw_continuation);
    assert_eq!(result, ExplorationPoll::SourceUnavailable(NATIVE_REASON.into()));
    for _ in 0..2 {
        adapter.prepare_poll_with_planning_input(&pose, &input).unwrap();
        assert_eq!(adapter.commit_prepared_poll().unwrap(), result);
        assert!(!adapter.last_no_frontier);
        assert_eq!(adapter.diagnostics().last_error.as_deref(), Some(NATIVE_REASON));
    }

    // Existing successful full-body fixture: prove the separate body path still
    // selects an actual admissible goal without visiting the planar interface.
    let body_snapshot = unknown_layer_map();
    let body_receiver = SurfaceInputReceiver::new();
    body_receiver.observe(body_snapshot.clone()).unwrap();
    let mut body_adapter = source(body_receiver, 1);
    let body_goal = goal(body_adapter.poll_goal(&state(0.4, 1)).unwrap());
    let p = body_goal.pose.pose.position;
    let q = body_goal.pose.pose.orientation;
    body_snapshot.validate_body_motion(([1.05,1.05,0.4],[0.,0.,0.,1.]),
        ([p.x,p.y,p.z],[q.x,q.y,q.z,q.w]), &[]).unwrap();
    assert_eq!(body_adapter.diagnostics().last_error, None);

    // Existing enclosed observed-map control remains genuine NoFrontier.
    let clear_receiver = SurfaceInputReceiver::new();
    clear_receiver.observe(super::map()).unwrap();
    let mut clear = source(clear_receiver, 1);
    clear.configure_constraints(Some(ExplorationBoundary::from_vertices(vec![
        Point2::new(0.5,0.5), Point2::new(1.5,0.5),
        Point2::new(1.5,1.5), Point2::new(0.5,1.5),
    ]).unwrap()), 1, 0.3).unwrap();
    assert_eq!(clear.poll_goal(&state(0.4, 1)).unwrap(), ExplorationPoll::NoFrontier);
    assert_eq!(clear.poll_goal(&state(0.4, 2)).unwrap(), ExplorationPoll::NoFrontier);
    assert!(clear.last_no_frontier);
}

#[test]
fn native_planar_route_filter_retains_cause_and_real_obstacle_stays_ordinary() {
    // Exact adjacent native-column fixture from the existing PCT regression.
    let original = thin([0, 1], 0);
    let mut cfg = config(0.3);
    cfg.wall_clearance_m = 0.03;
    let native_fixture = scene(vec![(original.xy(), native(vec![original.clone()]))], [], cfg.clone(), None);
    let from = point([0, 0], 0.6 * q());
    let to = point([3, 0], from[2]);
    let source_map = nuwax_nav_pct::Tomogram::from_surface(&native_fixture.source).unwrap();
    assert!(source_map.cell_at(from).is_none());
    assert_eq!(source_map.project_reference_motion(from, center([0, 0]), &[]).unwrap_err(), NATIVE_REASON);
    let body = |reference: [f64;3]| ([reference[0],reference[1],reference[2]+0.3],[0.,0.,0.,1.]);
    let budget = FrontierRouteBudget::with_states(64);
    assert_eq!(frontier_route_cost(&source_map, &native_fixture.source, from, to, body(from), body(to),
        0.3, &[], 0.3, &budget, 64), None);
    assert_eq!(budget.source_unavailable.borrow().as_ref(),
        Some(&nuwax_nav_pct::ReferenceMotionFailure::NativeContactGeometryUnavailable));
    assert!(!budget.last_inconclusive.get(), "native missing geometry is not a larger-search retry");

    // Keep the original producer's obstacle precedence and text. No error
    // strings are manufactured by the fixture or matched by production code.
    let blocked = scene(vec![(original.xy(), native(vec![original]))], [[0,1,1]], cfg, None);
    let blocked_map = nuwax_nav_pct::Tomogram::from_surface(&blocked.source).unwrap();
    let ordinary = "平面运动起点存在已知障碍或不兼容地形";
    assert_eq!(blocked_map.project_reference_motion(from, center([0, 0]), &[]).unwrap_err(), ordinary);
    assert_eq!(blocked_map.project_reference_motion_typed(from, center([0, 0]), &[]).unwrap_err(),
        nuwax_nav_pct::ReferenceMotionFailure::Other(ordinary.into()));
    // Actual existing cancellation scope has priority over native geometry;
    // no error text or producer outcome is injected into the projection.
    let cancelled=nuwax_nav_pct::PlanningCancellation::from_flag(Arc::new(
        std::sync::atomic::AtomicBool::new(true)));
    nuwax_nav_pct::with_planning_cancellation(&cancelled,|| {
        let typed=source_map.project_reference_motion_typed(from,center([0,0]),&[]).unwrap_err();
        assert_eq!(typed,nuwax_nav_pct::ReferenceMotionFailure::Other("层析规划已取消".into()));
        assert_eq!(source_map.project_reference_motion(from,center([0,0]),&[]).unwrap_err(),typed.to_string());
    });
    assert!(!nuwax_nav_pct::planning_cancelled());
    let budget = FrontierRouteBudget::with_states(64);
    assert_eq!(frontier_route_cost(&blocked_map, &blocked.source, from, to, body(from), body(to),
        0.3, &[], 0.3, &budget, 64), None);
    assert_eq!(budget.source_unavailable.borrow().as_ref(), None);
}
}
