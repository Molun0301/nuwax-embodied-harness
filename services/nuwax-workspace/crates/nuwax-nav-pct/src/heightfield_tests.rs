//! 已关联的固定原生计数，经 Writer/Receiver 准入后交给生产 PCT。
//! 不重写统计核，不把全局参考通过当作真实机体接触或实机运动许可。
#![cfg(feature = "mls-adapter")]

use crate::{PlanningFailure, SearchLimits, Tomogram};
use nuwax_map_chunks::heightfield_observations::{HEIGHT_SUB_BINS, HeightfieldColumn};
use nuwax_map_chunks::{
    ColumnGroundRecords, Domain, GroundRecordRef, GroundRecords, HeightfieldSurface, Key,
    LayerObservation, LayerSample, Receiver, Repository, Transfer, Writer, block_key, global_key,
    local_key,
};
use nuwax_nav_mls_planner::surface::GroundKey;
use nuwax_nav_mls_planner::{
    SurfaceBodyEnvelope, SurfaceObservation, SurfaceObservations, SurfaceSnapshot,
    mls_planner::Config,
};
use nuwax_types::ground_layer::GroundApproximation;
use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

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
    parts: Vec<(Key, GroundRecords)>,
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
    Fixture { source, parts }
}

fn native_id(map: &Tomogram, original: &HeightfieldColumn) -> usize {
    let xy = original.xy();
    let anchor = [xy[0], xy[1], original.members().next().unwrap().0];
    map.cells
        .iter()
        .position(|cell| cell.source_ground == Some(GroundKey::HeightfieldV1(anchor)))
        .expect("原生高度列必须保留自己的来源身份")
}

fn assert_native(map: &Tomogram, source: &SurfaceSnapshot, original: &HeightfieldColumn) -> usize {
    let id = native_id(map, original);
    let cell = &map.cells[id];
    assert_eq!(cell.ground, original.height());
    assert_eq!(cell.ground_interval, original.occupied_bounds());
    assert_eq!(
        cell.ground_approximation,
        GroundApproximation::ObservedHeightfield
    );
    let record = source
        .observations()
        .unwrap()
        .ground_record(cell.source_ground.unwrap())
        .unwrap();
    assert!(record.is_heightfield());
    assert_eq!(record.direction(), None);
    assert_eq!(record.scatter(), None);
    assert_eq!(
        record.members().collect::<Vec<_>>(),
        original.members().map(|(z, _)| z).collect::<Vec<_>>()
    );
    assert_eq!(record.occupied_bounds(q()), original.occupied_bounds());
    for (z, _) in original.members() {
        assert!(record.contains_member([original.xy()[0], original.xy()[1], z]));
    }
    let xy = center(original.xy());
    assert_eq!(map.ground_height_at(id, xy), Some(original.height()));
    assert_eq!(
        map.ground_height_at(id, [xy[0] + q() * 0.2, xy[1] - q() * 0.2]),
        Some(original.height())
    );
    assert_eq!(
        map.ground_height_at(id, [xy[0] + q(), xy[1]]),
        None,
        "已选高度列不能外推为相邻列的地面"
    );
    id
}

fn assert_blocked(map: &Tomogram, start: [f64; 3], goal: [f64; 3], boundary: &[[f64; 2]]) {
    assert!(
        !map.segment_clear(start, goal, boundary),
        "固定直段必须被真实几何阻断"
    );
    let failure = map
        .plan_bounded_with_limits_typed(
            start,
            goal,
            0.0,
            boundary,
            None,
            None,
            &[],
            SearchLimits { max_states: 4096 },
        )
        .unwrap_err();
    assert!(
        matches!(failure, PlanningFailure::Other(_)),
        "不能把资源预算或物化范围失败当作几何反例：{failure:?}"
    );
}

#[test]
fn heightfield_thick_5_10_15cm_keeps_native_height_members_and_bounds() {
    for cells in [1, 2, 3] {
        let originals: Vec<_> = (0..8)
            .map(|x| column([x, 0], (0..cells).map(|z| (z, [1; 8]))))
            .collect();
        let fixture = scene(
            originals
                .iter()
                .map(|c| (c.xy(), native(vec![c.clone()])))
                .collect(),
            [],
            config(0.3),
            None,
        );
        let map = Tomogram::from_surface(&fixture.source).unwrap();
        assert_eq!(map.cells.len(), originals.len());
        for original in &originals {
            assert_eq!(original.occupied_bounds(), [0.0, f64::from(cells) * q()]);
            assert_native(&map, &fixture.source, original);
            assert_eq!(
                map.occupancy[&original.xy()],
                vec![[0.0, f64::from(cells) * q()]]
            );
        }
        let route = map
            .plan(
                point([0, 0], originals[0].height()),
                point([7, 0], originals[7].height()),
                0.0,
                &[],
                None,
            )
            .unwrap();
        assert!(route.points.len() > 1);
        assert!(route.points.iter().all(|p| p[2] == originals[0].height()));
        assert!(
            route
                .points
                .windows(2)
                .all(|p| map.segment_clear(p[0], p[1], &[]))
        );
    }
}

#[test]
fn heightfield_near_bimodal_mass_swap_keeps_native_height_without_plane() {
    let mut heights = Vec::new();
    for (low, high) in [(49, 51), (51, 49)] {
        let original = column(
            [0, 0],
            [
                (0, [0, 0, 0, 0, 0, 0, low, 0]),
                (1, [0, high, 0, 0, 0, 0, 0, 0]),
            ],
        );
        let fixture = scene(
            vec![([0, 0], native(vec![original.clone()]))],
            [],
            config(0.3),
            None,
        );
        let map = Tomogram::from_surface(&fixture.source).unwrap();
        assert_eq!(map.cells.len(), 1);
        assert_native(&map, &fixture.source, &original);
        heights.push(map.cells[0].ground);
    }
    assert!(
        (heights[0] - heights[1]).abs() <= q() / 8.0,
        "沿用原生近双峰的一个子bin变化界"
    );
}

#[test]
fn heightfield_near_distinct_surfaces_keep_separate_members_and_ceiling() {
    let lower = thin([0, 0], 0);
    let upper = thin([0, 0], 2);
    let fixture = scene(
        vec![([0, 0], native(vec![upper.clone(), lower.clone()]))],
        [],
        config(0.02),
        None,
    );
    let map = Tomogram::from_surface(&fixture.source).unwrap();
    assert_eq!(map.cells.len(), 2);
    let low = assert_native(&map, &fixture.source, &lower);
    let high = assert_native(&map, &fixture.source, &upper);
    assert_eq!(map.cells[low].ceiling, 2.0 * q());
    assert!(map.cells[high].ceiling.is_infinite());
    assert_eq!(map.cell_at(point([0, 0], lower.height())), Some(low));
    assert_eq!(map.cell_at(point([0, 0], upper.height())), Some(high));
    assert_eq!(
        map.occupancy[&[0, 0]],
        vec![[0.0, q()], [2.0 * q(), 3.0 * q()]]
    );
}

#[test]
fn heightfield_cross_z_block_source_keeps_whole_column_mass_and_members() {
    for (xy, low_z) in [([-1, 3], -1), ([32, -1], 31)] {
        let original = column(
            xy,
            [
                (low_z, [0, 0, 0, 0, 0, 0, 0, 90]),
                (low_z + 1, [10, 0, 0, 0, 0, 0, 0, 0]),
            ],
        );
        let fixture = scene(
            vec![(xy, native(vec![original.clone()]))],
            [],
            config(0.3),
            None,
        );
        assert_eq!(fixture.parts.len(), 2);
        assert_ne!(fixture.parts[0].0[2], fixture.parts[1].0[2]);
        let mut masses = BTreeMap::new();
        for (block, records) in &fixture.parts {
            for (_, record) in records.iter() {
                if let GroundRecordRef::HeightfieldV1(fragment) = record {
                    assert_eq!(fragment.height_m, original.height());
                    assert_eq!(fragment.occupied_bounds, original.occupied_bounds());
                    for (local, bins) in &fragment.members {
                        masses.insert(
                            global_key(*block, *local).unwrap()[2],
                            bins.iter().sum::<f64>(),
                        );
                    }
                }
            }
        }
        assert!((masses[&low_z] - 0.9).abs() < 1e-15);
        assert!((masses[&(low_z + 1)] - 0.1).abs() < 1e-15);
        let map = Tomogram::from_surface(&fixture.source).unwrap();
        assert_eq!(map.cells.len(), 1);
        assert_native(&map, &fixture.source, &original);
    }
}

#[test]
fn heightfield_step_boundary_preserves_directional_capability_limits() {
    let lower = thin([0, 0], 0);
    let upper = thin([1, 0], 2);
    let rise = upper.height() - lower.height();
    assert!((rise - 2.0 * q()).abs() < 1e-12);
    for (up, down, may_up, may_down) in [
        (rise, 0.0, true, false),
        (0.0, rise, false, true),
        (rise - 2e-6, rise - 2e-6, false, false),
        (rise - 0.5e-6, rise - 0.5e-6, true, true),
    ] {
        let body = SurfaceBodyEnvelope {
            fixed_collision: None,
            footprint_xy_m: vec![[-0.01, -0.01], [0.01, -0.01], [0.01, 0.01], [-0.01, 0.01]],
            z_bounds_m: [-0.1, 0.1],
            body_to_support_z_m: 0.3,
            max_step_up_m: up,
            max_step_down_m: down,
            max_cross_slope_deg: 0.0,
            support_uncertainty_m: 0.0,
        };
        let fixture = scene(
            vec![
                ([0, 0], native(vec![lower.clone()])),
                ([1, 0], native(vec![upper.clone()])),
            ],
            [],
            config(0.3),
            Some(body),
        );
        let map = Tomogram::from_surface(&fixture.source).unwrap();
        let low = assert_native(&map, &fixture.source, &lower);
        let high = assert_native(&map, &fixture.source, &upper);
        assert_eq!(
            map.cells[low].neighbors.contains(&high),
            may_up,
            "up={up} down={down}"
        );
        assert_eq!(
            map.cells[high].neighbors.contains(&low),
            may_down,
            "up={up} down={down}"
        );
        assert_eq!(
            map.segment_clear(map.position(low), map.position(high), &[]),
            may_up
        );
        assert_eq!(
            map.segment_clear(map.position(high), map.position(low), &[]),
            may_down
        );
    }
}

#[test]
fn heightfield_exact_members_do_not_exempt_occupancy_inside_bounds() {
    let sparse = column(
        [1, 0],
        [
            (0, [99, 0, 0, 0, 0, 0, 0, 0]),
            (3, [1, 0, 0, 0, 0, 0, 0, 0]),
        ],
    );
    let h = sparse.height();
    let ends = [0, 2].map(|x| column([x, 0], [(0, [1, 0, 0, 0, 0, 0, 0, 0])]));
    let originals = [ends[0].clone(), sparse.clone(), ends[1].clone()];
    let base = scene(
        originals
            .iter()
            .map(|c| (c.xy(), native(vec![c.clone()])))
            .collect(),
        [],
        config(0.3),
        None,
    );
    let base_map = Tomogram::from_surface(&base.source).unwrap();
    assert_native(&base_map, &base.source, &sparse);
    for z in [1, 2] {
        let obstacle = [1, 0, z];
        let fixture = scene(
            originals
                .iter()
                .map(|c| (c.xy(), native(vec![c.clone()])))
                .collect(),
            [obstacle],
            config(0.3),
            None,
        );
        let observations = fixture.source.observations().unwrap();
        let record = observations
            .ground_record(GroundKey::HeightfieldV1([1, 0, 0]))
            .unwrap();
        assert_eq!(record.occupied_bounds(q()), [0.0, 4.0 * q()]);
        assert_eq!(record.members().collect::<Vec<_>>(), vec![0, 3]);
        assert!(!record.contains_member(obstacle));
        assert!(observations.get(obstacle).is_some());
        let map = Tomogram::from_surface(&fixture.source).unwrap();
        assert!(
            map.occupancy[&[1, 0]]
                .iter()
                .any(|band| band[0] <= f64::from(z) * q() && band[1] >= f64::from(z + 1) * q())
        );
        assert!(
            !map.cells
                .iter()
                .any(|cell| cell.source_ground == Some(record.key())),
            "非成员实际净空不足不能被bounds豁免"
        );
        let boundary = [[0.0, 0.0], [3.0 * q(), 0.0], [3.0 * q(), q()], [0.0, q()]];
        assert_blocked(&map, point([0, 0], h), point([2, 0], h), &boundary);
    }
}

#[test]
fn heightfield_low_bar_rejects_carried_motion_after_valid_thin_start() {
    let originals: Vec<_> = (0..7).map(|x| thin([x, 0], 0)).collect();
    let build = |bar| {
        scene(
            originals
                .iter()
                .map(|c| (c.xy(), native(vec![c.clone()])))
                .collect(),
            if bar { vec![[3, 0, 1]] } else { vec![] },
            config(0.3),
            None,
        )
    };
    let from = point([0, 0], q());
    let goal = center([6, 0]);
    let base = build(false);
    let base_map = Tomogram::from_surface(&base.source).unwrap();
    let projected = base_map.project_reference_motion(from, goal, &[]).unwrap();
    assert!((projected[2] - from[2]).abs() < 1e-12);
    let blocked = build(true);
    let blocked_map = Tomogram::from_surface(&blocked.source).unwrap();
    assert_eq!(
        blocked_map
            .project_reference_motion(from, center([0, 0]), &[])
            .unwrap(),
        from
    );
    let error = blocked_map
        .project_reference_motion(from, goal, &[])
        .unwrap_err();
    assert!(
        error == "平面运动穿过已知障碍或不兼容地形"
            || error == "平面运动外形与实际占据或禁行区域相交",
        "须由真实横杆的几何检查拒绝：{error}"
    );
    assert!(
        blocked
            .source
            .observations()
            .unwrap()
            .get([3, 0, 1])
            .is_some()
    );
}

#[test]
fn heightfield_diagonal_cannot_cut_a_known_wall_corner() {
    let a = thin([0, 0], 0);
    let b = thin([1, 1], 0);
    let floors = vec![([0, 0], native(vec![a.clone()])), ([1, 1], native(vec![b]))];
    let clear = scene(floors.clone(), [], config(0.3), None);
    let clear_map = Tomogram::from_surface(&clear.source).unwrap();
    let boundary = [
        [0.0, 0.0],
        [2.0 * q(), 0.0],
        [2.0 * q(), 2.0 * q()],
        [0.0, 2.0 * q()],
    ];
    let start = point([0, 0], a.height());
    let goal = point([1, 1], a.height());
    assert!(clear_map.plan(start, goal, 0.0, &boundary, None).is_ok());
    let walls = [[1, 0], [0, 1]]
        .into_iter()
        .flat_map(|xy| (0..22).map(move |z| [xy[0], xy[1], z]));
    let blocked = scene(floors, walls, config(0.3), None);
    let map = Tomogram::from_surface(&blocked.source).unwrap();
    let from = native_id(&map, &a);
    let to = map.cell_at(goal).unwrap();
    assert!(!map.cells[from].neighbors.contains(&to));
    assert_blocked(&map, start, goal, &boundary);
}

#[test]
fn heightfield_unknown_gap_carries_reference_without_creating_ground() {
    let originals = [thin([0, 0], 0), thin([8, 0], 0)];
    let fixture = scene(
        originals
            .iter()
            .map(|c| (c.xy(), native(vec![c.clone()])))
            .collect(),
        [],
        config(0.3),
        None,
    );
    let map = Tomogram::from_surface(&fixture.source).unwrap();
    let before = format!("{:?}", map.cells);
    let occupancy = map.occupancy.clone();
    let h = originals[0].height();
    for (start, end) in [(0, 8), (3, 8), (0, 5)] {
        let route = map
            .plan(point([start, 0], h), point([end, 0], h), 0.0, &[], None)
            .unwrap();
        assert!(route.points.len() > 1);
        assert!(route.points.iter().all(|p| p[2] == h));
        assert!(
            route
                .points
                .windows(2)
                .all(|p| map.segment_clear(p[0], p[1], &[]))
        );
    }
    assert_eq!(format!("{:?}", map.cells), before);
    assert_eq!(map.occupancy, occupancy);
    assert!(map.cell_at(point([4, 0], h)).is_none());
    assert_eq!(
        fixture
            .source
            .observations()
            .unwrap()
            .ground_records([4, 0])
            .count(),
        0
    );
    let boundary = [[0.0, 0.0], [9.0 * q(), 0.0], [9.0 * q(), q()], [0.0, q()]];
    for obstacle in [vec![[4, 0, 2]], (0..22).map(|z| [4, 0, z]).collect()] {
        let blocked = scene(
            originals
                .iter()
                .map(|c| (c.xy(), native(vec![c.clone()])))
                .collect(),
            obstacle,
            config(0.3),
            None,
        );
        assert_blocked(
            &Tomogram::from_surface(&blocked.source).unwrap(),
            point([0, 0], h),
            point([8, 0], h),
            &boundary,
        );
    }
}

#[test]
fn heightfield_and_legacy_records_select_by_height_and_keep_compatibility() {
    let xy = [0, 0];
    let lower = thin(xy, 0);
    let key = [0, 0, 4];
    let origin = point(xy, 4.5 * q());
    let normal_length = 1.01_f64.sqrt();
    let plane = LayerObservation {
        members: vec![local_key(key)],
        plane_origin: origin,
        normal: [-0.1 / normal_length, 0.0, 1.0 / normal_length],
        samples: vec![LayerSample {
            source: Default::default(),
            point: origin,
            source_sequence: 1,
            source_timestamp_ns: 20,
            point_index: 0,
        }],
    };
    let mut records = native(vec![lower.clone()]);
    records.legacy.push((block_key(key), plane));
    let fixture = scene(vec![(xy, records)], [[1, 0, -1]], config(0.02), None);
    let observations = fixture.source.observations().unwrap();
    let order: Vec<_> = observations.ground_records(xy).map(|r| r.key()).collect();
    assert_eq!(
        order,
        vec![
            GroundKey::LegacyPlane(key),
            GroundKey::HeightfieldV1([0, 0, 0])
        ],
        "原生借用迭代顺序不是高度顺序"
    );
    let map = Tomogram::from_surface(&fixture.source).unwrap();
    let low = assert_native(&map, &fixture.source, &lower);
    assert_eq!(map.cell_at(point(xy, lower.height())), Some(low));
    let high = map
        .cells
        .iter()
        .position(|c| c.source_ground == Some(GroundKey::LegacyPlane(key)))
        .unwrap();
    assert_eq!(
        map.cells[high].ground_approximation,
        GroundApproximation::ObservedLayer
    );
    let sample_xy = [center(xy)[0] + 0.01, center(xy)[1]];
    let expected = origin[2] + 0.001;
    assert!((map.ground_height_at(high, sample_xy).unwrap() - expected).abs() < 1e-9);
    let legacy_band = map.cells.iter().find(|c| c.column == [1, 0]).unwrap();
    assert_eq!(legacy_band.source_ground, None);
    assert_eq!(
        legacy_band.ground_approximation,
        GroundApproximation::LegacyContactInterval
    );
    assert_eq!(legacy_band.ground, 0.0);
}

fn sloped_legacy_fixture() -> Fixture {
    let key = [0, 0, 0];
    let origin = point([0, 0], 0.5 * q());
    let norm = 1.0625_f64.sqrt();
    let layer = LayerObservation {
        members: vec![local_key(key)],
        plane_origin: origin,
        normal: [-0.25 / norm, 0.0, 1.0 / norm],
        samples: vec![LayerSample {
            source: Default::default(),
            point: origin,
            source_sequence: 1,
            source_timestamp_ns: 20,
            point_index: 0,
        }],
    };
    let mut cfg = config(0.3);
    cfg.max_slope_deg = 20.0;
    scene(
        vec![(
            [0, 0],
            ColumnGroundRecords {
                classified_empty: false, legacy: vec![(block_key(key), layer)],
                heightfields: vec![],
            },
        )],
        [],
        cfg,
        Some(SurfaceBodyEnvelope {
            fixed_collision: None,
            footprint_xy_m: vec![[-0.01, -0.01], [0.01, -0.01], [0.01, 0.01], [-0.01, 0.01]],
            z_bounds_m: [-0.1, 0.1],
            body_to_support_z_m: 0.3,
            max_step_up_m: 0.0,
            max_step_down_m: 0.0,
            max_cross_slope_deg: 20.0,
            support_uncertainty_m: 0.0,
        }),
    )
}

#[test]
fn heightfield_legacy_plane_actual_xy_drives_motion_and_exact_goal() {
    let fixture = sloped_legacy_fixture();
    let map = Tomogram::from_surface(&fixture.source).unwrap();
    assert_eq!(map.cells.len(), 1);
    let center_height = 0.5 * q();
    assert_eq!(map.cells[0].ground, center_height);
    assert_eq!(map.ground_bounds, Some([center_height; 2]));
    // 固定斜面 z=q/2+(x-q/2)/4；两点在同一细格，均不在格中心。
    let a = [0.25 * q(), 0.5 * q(), 0.4375 * q()];
    let b = [0.75 * q(), 0.5 * q(), 0.5625 * q()];
    assert_eq!(map.cell_at(a), map.cell_at(b));
    assert!((map.calibrated_reference_height(a).unwrap() - a[2]).abs() < 1e-12);
    assert!((map.calibrated_reference_height(b).unwrap() - b[2]).abs() < 1e-12);
    assert!(
        b[2] > center_height + 1e-6,
        "旧中心ground_bounds会误拒该精确目标"
    );
    let bias = 0.01;
    for (from, to) in [(a, b), (b, a)] {
        let carried = [from[0], from[1], from[2] + bias];
        let projected = map
            .project_reference_motion(carried, [to[0], to[1]], &[])
            .unwrap();
        assert_eq!([projected[0], projected[1]], [to[0], to[1]]);
        assert!((projected[2] - to[2] - bias).abs() < 1e-12, "{projected:?}");
        let route = map.plan(from, to, 0.0, &[], None).unwrap();
        assert_eq!(route.points.first(), Some(&from));
        let end = route.points.last().unwrap();
        assert_eq!([end[0], end[1]], [to[0], to[1]]);
        assert!((end[2] - to[2]).abs() < 1e-12);
        assert!(
            route
                .points
                .windows(2)
                .all(|p| map.segment_clear(p[0], p[1], &[]))
        );
    }
}

#[test]
fn heightfield_legacy_plane_exit_carries_last_sample_without_goal_slope() {
    let fixture = sloped_legacy_fixture();
    let map = Tomogram::from_surface(&fixture.source).unwrap();
    let from = [0.25 * q(), 0.5 * q(), 0.4375 * q() + 0.01];
    let to = [2.15 * q(), 0.5 * q()];
    // 原步长q*0.4产生5段：x/q=0.25,0.63,1.01,1.39,1.77,2.15。
    // 最后实测点为0.63q，其斜面高度是0.5325q；之后只能保持该值。
    assert_eq!(((to[0] - from[0]) / (q() * 0.4)).ceil(), 5.0);
    let expected_reference = 0.5325 * q();
    let end = map.project_reference_motion(from, to, &[]).unwrap();
    assert_eq!([end[0], end[1]], to);
    assert!(
        (end[2] - expected_reference - 0.01).abs() < 1e-12,
        "{end:?}"
    );
    assert!(
        (end[2] - (0.5 * q() + 0.01)).abs() > 1e-6,
        "出口不能回到格中心高度"
    );

    // 在未知位置再延长，输出只携带已有高度。此处不声明全局离散图
    // 已解决Legacy跨缺测的连续实际高度输运，也不把目标高程当作输入。
    let beyond = [4.15 * q(), to[1]];
    let continued = map.project_reference_motion(end, beyond, &[]).unwrap();
    assert_eq!([continued[0], continued[1]], beyond);
    assert!((continued[2] - end[2]).abs() < 1e-12);
    let extrapolated_plane = 0.5 * q() + 0.25 * (beyond[0] - 0.5 * q()) + 0.01;
    assert!(
        (continued[2] - extrapolated_plane).abs() > q() * 0.5,
        "未知不能延长原斜面造坡"
    );
    assert_eq!(map.cells.len(), 1);
    assert!(map.cell_at(end).is_none());
    assert_eq!(
        fixture
            .source
            .observations()
            .unwrap()
            .ground_records([2, 0])
            .count(),
        0
    );
}

const NATIVE_CONTACT_UNAVAILABLE: &str = "NativeContactGeometryUnavailable: 原生地表成员与参考外形重叠，有限接触几何尚未提供；未证明碰撞或可通行";

#[test]
fn heightfield_member_overlap_requires_contact_geometry_independent_of_step_limit() {
    for cells in [1, 2, 3] {
        let original = column([0, 0], (0..cells).map(|z| (z, [1; 8])));
        for step in [0.0, 0.3] {
            let mut cfg = config(0.3);
            cfg.step_threshold_m = step;
            let fixture = scene(
                vec![([0, 0], native(vec![original.clone()]))],
                [],
                cfg,
                None,
            );
            let map = Tomogram::from_surface(&fixture.source).unwrap();
            assert_native(&map, &fixture.source, &original);
            let from = point([0, 0], original.height());
            let error = map
                .project_reference_motion(from, center([0, 0]), &[])
                .unwrap_err();
            assert_eq!(
                error, NATIVE_CONTACT_UNAVAILABLE,
                "cells={cells} step={step}"
            );
            assert_eq!(map.project_reference_motion_typed(from, center([0, 0]), &[]).unwrap_err(),
                crate::ReferenceMotionFailure::NativeContactGeometryUnavailable);
            assert!(matches!(crate::ReferenceMotionFailure::from(NATIVE_CONTACT_UNAVAILABLE),
                crate::ReferenceMotionFailure::Other(_)), "text cannot manufacture the typed outcome");
        }
    }
}

#[test]
fn heightfield_unknown_center_keeps_reference_but_requires_native_contact_geometry() {
    let original = thin([0, 1], 0);
    let mut cfg = config(0.3);
    cfg.wall_clearance_m = 0.03;
    let fixture = scene(
        vec![(original.xy(), native(vec![original.clone()]))],
        [],
        cfg.clone(),
        None,
    );
    let map = Tomogram::from_surface(&fixture.source).unwrap();
    let from = point([0, 0], 0.6 * q());
    let goal = point([3, 0], from[2]);
    assert!(map.cell_at(from).is_none());
    assert_eq!(
        map.calibrated_reference_height(from),
        Some(from[2]),
        "原生bounds不能把缺测中心吸附到邻列"
    );
    let before = format!("{:?}", map.cells);
    let route = map.plan(from, goal, 0.0, &[], None).unwrap();
    assert!(route.points.len() > 1);
    assert!(route.points.iter().all(|p| p[2] == from[2]));
    assert_eq!(
        map.project_reference_motion(from, center([0, 0]), &[])
            .unwrap_err(),
        NATIVE_CONTACT_UNAVAILABLE
    );
    assert_eq!(format!("{:?}", map.cells), before);
    assert!(
        fixture
            .source
            .observations()
            .unwrap()
            .get([0, 0, 0])
            .is_none()
    );

    // 同一邻列另有非成员横杆时，真实障碍优先于接触几何缺失。
    let blocked = scene(
        vec![(original.xy(), native(vec![original]))],
        [[0, 1, 1]],
        cfg,
        None,
    );
    let blocked_map = Tomogram::from_surface(&blocked.source).unwrap();
    assert_eq!(blocked_map.calibrated_reference_height(from), None);
    assert_eq!(
        blocked_map
            .project_reference_motion(from, center([0, 0]), &[])
            .unwrap_err(),
        "平面运动起点存在已知障碍或不兼容地形"
    );
    let failure = blocked_map
        .plan_bounded_with_limits_typed(
            from,
            goal,
            0.0,
            &[],
            None,
            None,
            &[],
            SearchLimits { max_states: 4096 },
        )
        .unwrap_err();
    assert_eq!(
        failure,
        PlanningFailure::Other("层析起点存在已知障碍或不兼容地形".into())
    );
}

#[test]
fn heightfield_legacy_plane_below_raw_thick_band_preserves_projection_fallback() {
    let key = [0, 0, 0];
    let origin = point([0, 0], 0.5 * q());
    let plane = LayerObservation {
        members: vec![local_key(key)],
        plane_origin: origin,
        normal: [0.0, 0.0, 1.0],
        samples: vec![LayerSample {
            source: Default::default(),
            point: origin,
            source_sequence: 1,
            source_timestamp_ns: 20,
            point_index: 0,
        }],
    };
    let upper = [[0, 0, 10], [0, 0, 11], [0, 0, 12]];
    let fixture = scene(
        vec![(
            [0, 0],
            ColumnGroundRecords {
                classified_empty: false, legacy: vec![(block_key(key), plane)],
                heightfields: vec![],
            },
        )],
        upper,
        config(0.3),
        None,
    );
    let raw = scene(vec![], upper.into_iter().chain([key]), config(0.3), None);
    let map = Tomogram::from_surface(&fixture.source).unwrap();
    let raw_map = Tomogram::from_surface(&raw.source).unwrap();
    assert_eq!(map.cells.len(), 2);
    let from = [0.25 * q(), 0.5 * q(), 12.0 * q()];
    let to = [0.75 * q(), 0.5 * q()];
    let upper_id = map.cell_at(from).unwrap();
    assert_eq!(map.cells[upper_id].source_ground, None);
    assert_eq!(
        map.cells[upper_id].ground_approximation,
        GroundApproximation::LegacyContactInterval
    );
    assert_eq!(
        map.cells[upper_id].ground_interval,
        [10.0 * q(), 13.0 * q()]
    );
    assert_eq!(map.cells[upper_id].ground, 12.0 * q());
    let measured = map
        .cells
        .iter()
        .find(|cell| cell.source_ground == Some(GroundKey::LegacyPlane(key)))
        .unwrap();
    assert_eq!(measured.ground, origin[2]);
    assert_eq!(measured.ceiling, 10.0 * q());
    assert_eq!(
        map.occupancy[&[0, 0]],
        vec![[0.0, q()], [10.0 * q(), 13.0 * q()]]
    );
    let projected = map.project_reference_motion(from, to, &[]).unwrap();
    assert_eq!(
        projected,
        raw_map.project_reference_motion(from, to, &[]).unwrap()
    );
    assert_eq!([projected[0], projected[1]], to);
    assert!((projected[2] - 12.0 * q()).abs() < 1e-12);
}

#[test]
fn classified_empty_native_column_does_not_fabricate_legacy_ground() {
    let f=scene(vec![([0,0],ColumnGroundRecords{classified_empty:true,legacy:vec![],heightfields:vec![]})],
        [[0,0,0],[1,0,0]],config(0.3),None);
    let empty=crate::source_ground::candidates(&f.source,[0,0],q(),&[[0.,q()]]);
    assert!(empty.is_some_and(|values|values.is_empty()));
    assert!(crate::source_ground::candidates(&f.source,[1,0],q(),&[[0.,q()]]).is_none());
    assert!(f.source.observations().unwrap().get([0,0,0]).is_some());
    assert!(f.source.observations().unwrap().classified_empty_ground([0,0]));
    assert_eq!(f.source.query_ground([0.025,0.025],0.025).state,
        nuwax_nav_mls_planner::surface::GroundQueryState::Unknown);
    assert_eq!(f.source.query_ground([0.075,0.025],0.025).state,
        nuwax_nav_mls_planner::surface::GroundQueryState::LegacyGeometry);
}

#[path = "ground_route_tests.rs"]
mod ground_route_tests;
