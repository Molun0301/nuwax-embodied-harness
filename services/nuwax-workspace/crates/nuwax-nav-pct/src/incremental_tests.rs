//! Exact cold-build oracle for persistent column updates. Geometry uses production builders.
//! One explicitly frozen former FIFO preserves the seed-order counterexample;
//! current full and incremental graphs must agree under stable semantic seed ordering.
use crate::{Cell, ColumnUpdate, IncrementalStats, PlanningCancellation, Tomogram};
use nuwax_types::ground_layer::GroundApproximation;
use std::collections::{BTreeMap, BTreeSet};
use std::sync::{
    Arc,
    atomic::{AtomicBool, AtomicU64, Ordering},
};

type Column = [i32; 2];
type Bands = Vec<[f64; 2]>;
type Columns = BTreeMap<Column, Bands>;
type Change = (Column, Option<Bands>);

#[derive(Clone, Copy)]
struct Parameters {
    resolution: f64,
    body: f64,
    width: f64,
    up: f64,
    down: f64,
    slope: f64,
    radius: f64,
}

const FLAT: Parameters = Parameters {
    resolution: 0.125,
    body: 0.5,
    width: 0.125,
    up: 0.0,
    down: 0.0,
    slope: 0.0,
    radius: 0.375,
};

fn build_ordered(
    parameters: Parameters,
    columns: impl IntoIterator<Item = (Column, Bands)>,
) -> Tomogram {
    Tomogram::from_columns_with_quantum(
        parameters.resolution,
        parameters.resolution,
        columns,
        parameters.body,
        parameters.width,
        parameters.up,
        parameters.down,
        parameters.slope,
        parameters.radius,
    )
    .unwrap()
}

fn build(parameters: Parameters, columns: &Columns) -> Tomogram {
    build_ordered(
        parameters,
        columns.iter().map(|(&xy, bands)| (xy, bands.clone())),
    )
}

fn floor() -> Bands {
    vec![[-FLAT.resolution, 0.0]]
}

fn wall() -> Bands {
    vec![[-FLAT.resolution, 2.0]]
}

fn plane(x: std::ops::RangeInclusive<i32>, y: std::ops::RangeInclusive<i32>) -> Columns {
    x.flat_map(|x| y.clone().map(move |y| ([x, y], floor())))
        .collect()
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct Identity {
    column: Column,
    ground: u64,
    #[cfg(feature = "mls-adapter")]
    source: Option<nuwax_nav_mls_planner::surface::GroundKey>,
}

fn identity(cell: &Cell) -> Identity {
    Identity {
        column: cell.column,
        ground: cell.ground.to_bits(),
        #[cfg(feature = "mls-adapter")]
        source: cell.source_ground,
    }
}

#[derive(Debug, PartialEq, Eq)]
struct CellFact {
    layer: i32,
    approximation: GroundApproximation,
    bits: [u64; 9],
    neighbors: Vec<Identity>,
}

#[derive(Debug, PartialEq, Eq)]
struct Facts {
    parameters: [u64; 10],
    occupied_volume: bool,
    cells: BTreeMap<Identity, CellFact>,
    seed_order: Vec<Identity>,
    columns: BTreeMap<Column, Vec<Identity>>,
    known: BTreeSet<Column>,
    occupancy: BTreeMap<Column, Vec<[u64; 2]>>,
    xy_bounds: Option<[Column; 2]>,
    ground_bounds: Option<[u64; 2]>,
    window: Option<([u64; 2], u64)>,
}

fn facts(map: &Tomogram) -> Facts {
    let ids: Vec<_> = map.cells.iter().map(identity).collect();
    assert_eq!(
        ids.iter().collect::<BTreeSet<_>>().len(),
        ids.len(),
        "ambiguous cell identity"
    );
    let seed_ids: Vec<_> = map.seed_order.ids().collect();
    assert_eq!(
        seed_ids.len(),
        ids.len(),
        "seed index has missing or extra entries"
    );
    assert_eq!(
        seed_ids.iter().copied().collect::<BTreeSet<_>>(),
        (0..ids.len()).collect::<BTreeSet<_>>(),
        "seed index contains duplicate, omitted or stale IDs"
    );
    let seed_order: Vec<_> = seed_ids.into_iter().map(|id| ids[id].clone()).collect();
    let mut expected: Vec<_> = (0..ids.len()).collect();
    expected.sort_by(|&a, &b| {
        let a = &map.cells[a];
        let b = &map.cells[b];
        let order = a
            .column
            .cmp(&b.column)
            .then_with(|| a.ground.total_cmp(&b.ground));
        #[cfg(feature = "mls-adapter")]
        {
            order.then_with(|| a.source_ground.cmp(&b.source_ground))
        }
        #[cfg(not(feature = "mls-adapter"))]
        {
            order
        }
    });
    assert_eq!(
        seed_order,
        expected
            .into_iter()
            .map(|id| ids[id].clone())
            .collect::<Vec<_>>(),
        "seed index must follow XY, numeric height and source record key, independent of storage ID"
    );
    let mut referenced = vec![false; ids.len()];
    let columns = map
        .columns
        .iter()
        .map(|(&xy, members)| {
            let values = members
                .iter()
                .map(|&id| {
                    assert!(!referenced[id], "cell {id} referenced more than once");
                    referenced[id] = true;
                    assert_eq!(ids[id].column, xy);
                    ids[id].clone()
                })
                .collect();
            (xy, values)
        })
        .collect();
    assert!(referenced.into_iter().all(|value| value), "unindexed cell");
    let cells = map
        .cells
        .iter()
        .enumerate()
        .map(|(id, cell)| {
            let neighbors = cell
                .neighbors
                .iter()
                .map(|&next| ids[next].clone())
                .collect();
            (
                ids[id].clone(),
                CellFact {
                    layer: cell.layer,
                    approximation: cell.ground_approximation,
                    bits: [
                        cell.ground,
                        cell.ground_interval[0],
                        cell.ground_interval[1],
                        cell.ceiling,
                        cell.clearance,
                        cell.support_clearance,
                        cell.cost,
                        cell.terrain,
                        cell.terrain_upper,
                    ]
                    .map(f64::to_bits),
                    neighbors,
                },
            )
        })
        .collect();
    Facts {
        parameters: [
            map.resolution,
            map.source_quantum,
            map.step_up,
            map.step_down,
            map.slope,
            map.body_height,
            map.half_width,
            map.clearance_radius,
            map.minimum_cost,
            crate::reference::UNKNOWN_COST,
        ]
        .map(f64::to_bits),
        occupied_volume: map.occupied_volume,
        cells,
        seed_order,
        columns,
        known: map.known_columns.iter().copied().collect(),
        occupancy: map
            .occupancy
            .iter()
            .map(|(&xy, bands)| {
                (
                    xy,
                    bands.iter().map(|band| band.map(f64::to_bits)).collect(),
                )
            })
            .collect(),
        xy_bounds: map.xy_bounds,
        ground_bounds: map.ground_bounds.map(|bounds| bounds.map(f64::to_bits)),
        window: map
            .window
            .map(|(center, radius)| (center.map(f64::to_bits), radius.to_bits())),
    }
}

fn exact(actual: &Tomogram, cold: &Tomogram) {
    assert_eq!(facts(actual), facts(cold));
    for (xy, ids) in &actual.columns {
        for &id in ids {
            let key = identity(&actual.cells[id]);
            let other = cold.columns[xy]
                .iter()
                .copied()
                .find(|&id| identity(&cold.cells[id]) == key)
                .unwrap();
            for offset in [[0.5, 0.5], [0.125, 0.875], [0.875, 0.125]] {
                let point = [
                    (f64::from(xy[0]) + offset[0]) * actual.resolution,
                    (f64::from(xy[1]) + offset[1]) * actual.resolution,
                ];
                assert_eq!(
                    actual.ground_height_at(id, point).map(f64::to_bits),
                    cold.ground_height_at(other, point).map(f64::to_bits)
                );
            }
            #[cfg(feature = "mls-adapter")]
            {
                let record = |map: &Tomogram, id| {
                    map.source_ground_record(id).map(|record| {
                        (
                            record.key(),
                            record.members().collect::<Vec<_>>(),
                            record.occupied_bounds(map.source_quantum).map(f64::to_bits),
                            record.direction().map(|value| value.map(f64::to_bits)),
                            record.scatter().map(f64::to_bits),
                        )
                    })
                };
                assert_eq!(record(actual, id), record(cold, other));
            }
        }
    }
}

fn route_exact(actual: &Tomogram, cold: &Tomogram, start: [f64; 3], goal: [f64; 3]) {
    match (
        actual.plan(start, goal, 0.0, &[], None),
        cold.plan(start, goal, 0.0, &[], None),
    ) {
        (Ok(actual), Ok(cold)) => {
            assert_eq!(
                actual
                    .points
                    .iter()
                    .map(|point| point.map(f64::to_bits))
                    .collect::<Vec<_>>(),
                cold.points
                    .iter()
                    .map(|point| point.map(f64::to_bits))
                    .collect::<Vec<_>>()
            );
            assert_eq!(
                (actual.stats.cells, actual.stats.expanded),
                (cold.stats.cells, cold.stats.expanded)
            );
        }
        (Err(actual), Err(cold)) => assert_eq!(actual, cold),
        pair => panic!("incremental route outcome differs from cold build: {pair:?}"),
    }
}

fn point(map: &Tomogram, column: Column, ground: f64) -> [f64; 3] {
    [
        (f64::from(column[0]) + 0.5) * map.resolution,
        (f64::from(column[1]) + 0.5) * map.resolution,
        ground,
    ]
}

fn cell(map: &Tomogram, xy: Column, ground: f64) -> &Cell {
    &map.cells[*map.columns[&xy]
        .iter()
        .find(|&&id| map.cells[id].ground.to_bits() == ground.to_bits())
        .unwrap()]
}

fn updates(changes: &[Change]) -> impl Iterator<Item = ColumnUpdate> + '_ {
    changes.iter().map(|(column, bands)| ColumnUpdate {
        column: *column,
        bands: bands.clone(),
    })
}

fn apply_model(columns: &mut Columns, changes: &[Change]) {
    for (xy, bands) in changes {
        match bands {
            Some(bands) => {
                columns.insert(*xy, bands.clone());
            }
            None => {
                columns.remove(xy);
            }
        }
    }
}

fn checked_update(
    old: &Tomogram,
    columns: &mut Columns,
    parameters: Parameters,
    changes: &[Change],
) -> (Tomogram, IncrementalStats) {
    let before = facts(old);
    let (next, stats) = old.with_column_updates(updates(changes)).unwrap();
    assert_eq!(facts(old), before, "the retained input snapshot changed");
    apply_model(columns, changes);
    exact(&next, &build(parameters, columns));
    assert!(!stats.rebuilt, "raw column update unexpectedly rebuilt");
    (next, stats)
}

#[test]
fn incremental_oracle_remaps_global_ids_without_reordering_candidates() {
    let columns = plane(-3..=3, -3..=3);
    let first = build(FLAT, &columns);
    let reverse = build_ordered(
        FLAT,
        columns.iter().rev().map(|(&xy, bands)| (xy, bands.clone())),
    );
    assert_ne!(identity(&first.cells[0]), identity(&reverse.cells[0]));
    exact(&first, &reverse);
    route_exact(
        &first,
        &reverse,
        point(&first, [-2, 0], 0.0),
        point(&first, [2, 0], 0.0),
    );
}

#[test]
fn incremental_isolated_edit_swaps_remote_tail_without_replaying_remote_fields() {
    let near = [0, 0];
    // These are only the remote graph's local facts. Full-map parameters and
    // bounds are independently compared with the stable cold build below.
    let remote_facts = |mut value: Facts| {
        value.cells.retain(|key, _| key.column != near);
        value.columns.remove(&near);
        value.known.remove(&near);
        value.occupancy.remove(&near);
        value.seed_order.retain(|key| key.column != near);
        (
            value.cells,
            value.columns,
            value.known,
            value.occupancy,
            value.seed_order,
        )
    };
    let mut work = Vec::new();
    for side in [16, 64] {
        let mut columns = plane(100..=(99 + side), 100..=(99 + side));
        columns.insert(near, floor());
        let old = build(FLAT, &columns);
        assert_eq!(old.cells[0].column, near);
        assert!(
            old.cells[0].neighbors.is_empty(),
            "the removed node has no remote successors"
        );
        let tail = identity(&old.cells[old.cells.len() - 1]);
        assert_eq!(tail.column, [99 + side, 99 + side]);
        let retained_remote = remote_facts(facts(&old));

        let (next, stats) = checked_update(
            &old,
            &mut columns,
            FLAT,
            &[(near, Some(vec![[1.875, 2.0]]))],
        );
        assert_eq!(
            identity(&next.cells[0]),
            tail,
            "the remote tail must actually move to ID zero"
        );
        assert!(cell(&next, near, 2.0).neighbors.is_empty());
        assert_eq!(remote_facts(facts(&next)), retained_remote);
        assert_eq!(stats.columns_prepared, 1);
        assert!(
            stats.topology_recomputed > 1,
            "remote references still need ID repair"
        );
        assert_eq!(
            stats.terrain_replayed, 1,
            "only the new isolated node needs terrain replay"
        );
        assert_eq!(
            stats.distance_invalidated, 2,
            "one isolated node is invalidated once in each of the wall and missing fields"
        );
        work.push((
            side * side,
            stats.terrain_replayed,
            stats.distance_invalidated,
        ));
    }
    assert!(work[1].0 > work[0].0);
    assert_eq!(
        (work[0].1, work[0].2),
        (work[1].1, work[1].2),
        "remote plane growth must not increase field repair work for a pure ID move"
    );
}

#[test]
fn incremental_cold_build_rejects_duplicate_xy_columns_with_exact_error() {
    let result = Tomogram::from_columns(
        FLAT.resolution,
        [([0, 0], floor()), ([0, 0], vec![[1.875, 2.0]])],
        FLAT.body,
        FLAT.width,
        FLAT.up,
        FLAT.down,
        FLAT.slope,
    );
    assert_eq!(result.unwrap_err(), "层析输入含重复 XY 列");
}

/// Frozen former terrain FIFO: storage-ID seeding, production neighbors, unchanged
/// threshold and queue behavior. Only this test reference retains the former order.
fn frozen_terrain_in_cell_order(map: &Tomogram) -> Vec<f64> {
    let mut terrain = Vec::with_capacity(map.cells.len());
    let mut queue = std::collections::VecDeque::new();
    for (id, cell) in map.cells.iter().enumerate() {
        let value = cell
            .neighbors
            .iter()
            .map(|&next| (map.cells[next].ground - cell.ground).abs() / map.resolution)
            .fold(0.0, f64::max)
            .min(2.0);
        terrain.push(value);
        if value > 0.0 {
            queue.push_back(id);
        }
    }
    let decay = 2.0 * map.resolution / map.clearance_radius.max(map.resolution);
    while let Some(id) = queue.pop_front() {
        let value = terrain[id] - decay;
        if value <= 0.0 {
            continue;
        }
        for &next in &map.cells[id].neighbors {
            if value > terrain[next] + 1e-9 {
                terrain[next] = value;
                queue.push_back(next);
            }
        }
    }
    terrain
}

/// Keep the former two outcomes as evidence while requiring the current stable
/// XY/height/source-key seeding to agree across storage order and edit history.
#[test]
fn incremental_stable_seed_order_preserves_frozen_fifo_counterexample() {
    let parameters = Parameters {
        resolution: 1.0,
        body: 0.25,
        width: 0.0,
        up: 0.0,
        down: 1.1,
        slope: 0.0,
        radius: 4.0,
    };
    let u = [-1, 0];
    let t = [0, 0];
    let v = [1, 0];
    let u_band = vec![[0.0, 1.0]];
    let t_band = vec![[-1.0, 0.0]];
    let v_band = vec![[5e-10, 1.0 + 5e-10]];
    let first = build_ordered(
        parameters,
        [
            (u, u_band.clone()),
            (t, t_band.clone()),
            (v, v_band.clone()),
        ],
    );
    let reversed = build_ordered(
        parameters,
        [(v, v_band.clone()), (t, t_band.clone()), (u, u_band)],
    );
    let assert_real_connect = |map: &Tomogram| {
        for (xy, height) in [(u, 1.0), (v, 1.0 + 5e-10)] {
            assert_eq!(
                cell(map, xy, height)
                    .neighbors
                    .iter()
                    .map(|&id| map.cells[id].column)
                    .collect::<Vec<_>>(),
                vec![t],
            );
        }
        assert!(cell(map, t, 0.0).neighbors.is_empty());
    };
    assert_real_connect(&first);
    assert_real_connect(&reversed);
    assert_eq!(
        first
            .cells
            .iter()
            .map(|cell| cell.column)
            .collect::<Vec<_>>(),
        vec![u, t, v]
    );
    assert_eq!(
        reversed
            .cells
            .iter()
            .map(|cell| cell.column)
            .collect::<Vec<_>>(),
        vec![v, t, u]
    );
    assert_eq!(
        frozen_terrain_in_cell_order(&first)[first.columns[&t][0]].to_bits(),
        0.5_f64.to_bits()
    );
    assert_eq!(
        frozen_terrain_in_cell_order(&reversed)[reversed.columns[&t][0]].to_bits(),
        0.5000000005_f64.to_bits()
    );
    assert_eq!(cell(&first, t, 0.0).terrain.to_bits(), 0.5_f64.to_bits());
    assert_eq!(cell(&reversed, t, 0.0).terrain.to_bits(), 0.5_f64.to_bits());
    exact(&first, &reversed);

    let retained = facts(&first);
    let changed_u = vec![[0.1, 1.0]];
    let (incremental, stats) = first
        .with_column_updates([ColumnUpdate {
            column: u,
            bands: Some(changed_u.clone()),
        }])
        .unwrap();
    assert!(!stats.rebuilt);
    assert_eq!(facts(&first), retained);
    assert_eq!(
        cell(&incremental, u, 1.0).ground.to_bits(),
        cell(&first, u, 1.0).ground.to_bits()
    );
    assert_ne!(
        cell(&incremental, u, 1.0).ground_interval.map(f64::to_bits),
        cell(&first, u, 1.0).ground_interval.map(f64::to_bits)
    );
    assert_eq!(
        incremental
            .cells
            .iter()
            .map(|cell| cell.column)
            .collect::<Vec<_>>(),
        vec![v, t, u],
        "swap-removal changes storage order, not the stable semantic seed order"
    );
    assert_real_connect(&incremental);

    let current_order = build_ordered(
        parameters,
        incremental
            .cells
            .iter()
            .map(|cell| (cell.column, incremental.occupancy[&cell.column].clone())),
    );
    exact(&incremental, &current_order);
    assert_eq!(
        cell(&incremental, t, 0.0).terrain.to_bits(),
        0.5_f64.to_bits()
    );
    assert_eq!(
        frozen_terrain_in_cell_order(&incremental)[incremental.columns[&t][0]].to_bits(),
        0.5000000005_f64.to_bits()
    );

    let fixed_order = build_ordered(
        parameters,
        [
            (u, changed_u.clone()),
            (t, t_band.clone()),
            (v, v_band.clone()),
        ],
    );
    assert_real_connect(&fixed_order);
    assert_eq!(
        cell(&fixed_order, t, 0.0).terrain.to_bits(),
        0.5_f64.to_bits()
    );
    exact(&incremental, &fixed_order);
    let before = facts(&incremental);
    let (deleted, _) = incremental
        .with_column_updates([ColumnUpdate {
            column: u,
            bands: None,
        }])
        .unwrap();
    assert_eq!(facts(&incremental), before);
    exact(
        &deleted,
        &build_ordered(parameters, [(t, t_band), (v, v_band)]),
    );
    assert_eq!(
        cell(&deleted, t, 0.0).terrain.to_bits(),
        0.5000000005_f64.to_bits()
    );
    let before = facts(&deleted);
    let (reinserted, _) = deleted
        .with_column_updates([ColumnUpdate {
            column: u,
            bands: Some(changed_u),
        }])
        .unwrap();
    assert_eq!(facts(&deleted), before);
    assert_real_connect(&reinserted);
    assert_eq!(
        cell(&reinserted, t, 0.0).terrain.to_bits(),
        0.5_f64.to_bits()
    );
    exact(&reinserted, &fixed_order);
}

#[test]
fn incremental_wall_removal_updates_far_clearance_and_last_seed_infinity() {
    let mut columns = plane(-16..=80, -40..=40);
    for x in [32, 64] {
        for y in -40..=40 {
            columns.insert([x, y], wall());
        }
    }
    let old = build(FLAT, &columns);
    assert_eq!(
        cell(&old, [8, 0], 0.0).clearance.to_bits(),
        (23.5 * FLAT.resolution).to_bits()
    );
    let support = cell(&old, [8, 0], 0.0).support_clearance.to_bits();
    let changes: Vec<_> = (-40..=40).map(|y| ([32, y], Some(floor()))).collect();
    let (middle, stats) = checked_update(&old, &mut columns, FLAT, &changes);
    assert_eq!(
        cell(&middle, [8, 0], 0.0).clearance.to_bits(),
        (55.5 * FLAT.resolution).to_bits()
    );
    assert_eq!(
        cell(&middle, [8, 0], 0.0).support_clearance.to_bits(),
        support
    );
    assert!(cell(&middle, [8, 0], 0.0).cost < cell(&old, [8, 0], 0.0).cost);
    assert!(stats.distance_invalidated > 0);
    let changes: Vec<_> = (-40..=40).map(|y| ([64, y], Some(floor()))).collect();
    let (final_map, _) = checked_update(&middle, &mut columns, FLAT, &changes);
    assert_eq!(cell(&final_map, [8, 0], 0.0).clearance, f64::INFINITY);
    assert_eq!(
        cell(&final_map, [8, 0], 0.0).cost.to_bits(),
        1.0_f64.to_bits()
    );
    let cold = build(FLAT, &columns);
    let start = point(&final_map, [8, 0], 0.0);
    let goal = point(&final_map, [70, 0], 0.0);
    assert!(cold.plan(start, goal, 0.0, &[], None).is_ok());
    route_exact(&final_map, &cold, start, goal);
}

#[test]
fn incremental_unknown_and_known_empty_columns_keep_distinct_distance_seeds() {
    let mut columns = plane(24..=40, -8..=8);
    let old = build(FLAT, &columns);
    let (unknown, _) = checked_update(&old, &mut columns, FLAT, &[([31, 0], None)]);
    assert!(!unknown.known_columns.contains(&[31, 0]));
    assert_eq!(
        cell(&unknown, [30, 0], 0.0).support_clearance,
        FLAT.resolution * 0.5
    );
    assert_eq!(cell(&unknown, [30, 0], 0.0).clearance, f64::INFINITY);
    let (known, _) = checked_update(&unknown, &mut columns, FLAT, &[([31, 0], Some(vec![]))]);
    assert!(known.known_columns.contains(&[31, 0]));
    assert_eq!(known.occupancy[&[31, 0]], Vec::<[f64; 2]>::new());
    assert_eq!(cell(&known, [30, 0], 0.0).clearance, FLAT.resolution * 0.5);
    assert!(cell(&known, [30, 0], 0.0).support_clearance > FLAT.resolution * 0.5);
    let (restored, _) = checked_update(&known, &mut columns, FLAT, &[([31, 0], Some(floor()))]);
    exact(&restored, &old);
}

#[test]
fn incremental_step_insertion_and_removal_replay_exact_terrain_costs() {
    let parameters = Parameters {
        up: 0.25,
        down: 0.25,
        radius: 1.0,
        ..FLAT
    };
    let mut columns = plane(-12..=12, -6..=6);
    let old = build(parameters, &columns);
    let raised: Vec<_> = (-6..=6)
        .map(|y| ([0, y], Some(vec![[0.125, 0.25]])))
        .collect();
    let (step, stats) = checked_update(&old, &mut columns, parameters, &raised);
    assert_eq!(
        cell(&step, [4, 0], 0.0).terrain.to_bits(),
        1.25_f64.to_bits()
    );
    assert_eq!(
        cell(&step, [4, 0], 0.0).terrain_upper.to_bits(),
        1.25_f64.to_bits()
    );
    assert!(stats.terrain_replayed > 0);
    assert!(stats.upper_relaxed > 0);
    let removed: Vec<_> = (-6..=6).map(|y| ([0, y], Some(floor()))).collect();
    let (restored, _) = checked_update(&step, &mut columns, parameters, &removed);
    assert_eq!(
        cell(&restored, [4, 0], 0.0).terrain.to_bits(),
        0.0_f64.to_bits()
    );
    exact(&restored, &old);
}

#[test]
fn incremental_delete_all_columns_then_reinsert_restores_bounds_and_indices() {
    let mut columns = plane(-1..=1, -1..=1);
    let original = columns.clone();
    let old = build(FLAT, &columns);
    let removed: Vec<_> = columns.keys().map(|&xy| (xy, None)).collect();
    let (empty, _) = checked_update(&old, &mut columns, FLAT, &removed);
    assert!(empty.cells.is_empty());
    assert!(empty.known_columns.is_empty());
    assert!(empty.occupancy.is_empty());
    assert_eq!(empty.xy_bounds, None);
    assert_eq!(empty.ground_bounds, None);
    assert_eq!(
        empty.minimum_cost.to_bits(),
        crate::reference::UNKNOWN_COST.to_bits()
    );
    let restored: Vec<_> = original
        .into_iter()
        .map(|(xy, bands)| (xy, Some(bands)))
        .collect();
    let (restored, _) = checked_update(&empty, &mut columns, FLAT, &restored);
    exact(&restored, &old);
}

#[test]
fn incremental_xy_block_edges_repair_cardinals_diagonals_and_costs() {
    for edge in [0, 32] {
        let mut columns = plane((edge - 4)..=(edge + 4), -4..=4);
        let old = build(FLAT, &columns);
        let origin = [edge - 1, -1];
        let diagonal = [edge, 0];
        let (blocked, _) = checked_update(
            &old,
            &mut columns,
            FLAT,
            &[([edge, -1], Some(wall())), ([edge - 1, 0], Some(wall()))],
        );
        assert!(
            !cell(&blocked, origin, 0.0)
                .neighbors
                .iter()
                .any(|&id| blocked.cells[id].column == diagonal)
        );
        let (restored, _) = checked_update(
            &blocked,
            &mut columns,
            FLAT,
            &[([edge, -1], Some(floor())), ([edge - 1, 0], Some(floor()))],
        );
        assert!(
            cell(&restored, origin, 0.0)
                .neighbors
                .iter()
                .any(|&id| restored.cells[id].column == diagonal)
        );
        exact(&restored, &old);
        route_exact(
            &restored,
            &old,
            point(&old, [edge - 3, -2], 0.0),
            point(&old, [edge + 3, 2], 0.0),
        );
    }
}

#[test]
fn incremental_disjoint_update_orders_match_each_prefix_and_final_rebuild() {
    let base = plane(-4..=36, -5..=5);
    let old = build(FLAT, &base);
    let a = vec![([31, 0], Some(wall())), ([32, 1], Some(vec![]))];
    let b = vec![
        ([-1, 0], None),
        ([0, 1], Some(vec![[-0.125, 0.0], [1.875, 2.0]])),
    ];
    let run = |first: &[Change], second: &[Change]| {
        let mut columns = base.clone();
        let (middle, _) = checked_update(&old, &mut columns, FLAT, first);
        checked_update(&middle, &mut columns, FLAT, second).0
    };
    let ab = run(&a, &b);
    let ba = run(&b, &a);
    let mut columns = base.clone();
    let combined: Vec<_> = a.iter().chain(&b).cloned().collect();
    let (batch, _) = checked_update(&old, &mut columns, FLAT, &combined);
    exact(&ab, &ba);
    exact(&ab, &batch);
    let (replayed, _) = checked_update(&batch, &mut columns, FLAT, &combined);
    exact(&replayed, &batch);
    route_exact(
        &ab,
        &ba,
        point(&ab, [-3, -3], 0.0),
        point(&ab, [35, 3], 0.0),
    );
}

#[test]
fn incremental_layer_replacement_preserves_tie_identity_and_neighbor_order() {
    let parameters = Parameters {
        resolution: 0.1,
        body: 0.1,
        width: 0.1,
        up: 0.6,
        down: 0.6,
        slope: 10.0,
        radius: 0.1,
    };
    let mut columns: Columns = [
        ([0, 0], vec![[-0.1, 0.0]]),
        ([-1, 0], vec![[0.4, 0.5]]),
        ([0, -1], vec![[0.4, 0.5]]),
        ([-1, -1], vec![[0.4, 0.5]]),
    ]
    .into_iter()
    .collect();
    let old = build(parameters, &columns);
    let (next, _) = checked_update(
        &old,
        &mut columns,
        parameters,
        &[
            ([-1, 0], Some(vec![[-0.6, -0.5], [0.4, 0.5]])),
            ([0, -1], Some(vec![[-0.6, -0.5], [0.4, 0.5]])),
        ],
    );
    let origin = cell(&next, [0, 0], 0.0);
    for xy in [[-1, 0], [0, -1]] {
        let selected = origin
            .neighbors
            .iter()
            .map(|&id| &next.cells[id])
            .find(|candidate| candidate.column == xy)
            .unwrap();
        assert_eq!(selected.ground.to_bits(), (-0.5_f64).to_bits());
    }
    assert!(
        !origin
            .neighbors
            .iter()
            .any(|&id| next.cells[id].column == [-1, -1])
    );
    route_exact(
        &next,
        &build(parameters, &columns),
        [0.05, 0.05, 0.0],
        [-0.05, -0.05, 0.5],
    );
}

#[test]
fn incremental_cancel_before_update_keeps_retained_map() {
    let columns = plane(-4..=4, -4..=4);
    let old = build(FLAT, &columns);
    let before = facts(&old);
    let flag = PlanningCancellation::from_flag(Arc::new(AtomicBool::new(true)));
    let epoch = PlanningCancellation::new(Arc::new(AtomicU64::new(2)), 1);
    for token in [flag, epoch] {
        let result = crate::with_planning_cancellation(&token, || {
            old.with_column_updates(updates(&[([0, 0], None)]))
        });
        assert_eq!(result.unwrap_err(), "层析规划已取消");
        assert_eq!(facts(&old), before);
        assert!(!crate::planning_cancelled());
    }
}

#[test]
fn incremental_cancel_during_update_keeps_snapshot_and_retry_matches_full() {
    let mut columns = plane(-16..=80, -16..=16);
    for y in -16..=16 {
        columns.insert([32, y], wall());
    }
    let old = build(FLAT, &columns);
    let before = facts(&old);
    let changes: Vec<_> = (-16..=16).map(|y| ([32, y], Some(floor()))).collect();
    // Fixed checkpoints exercise interrupted work. Phase names are not inferred
    // from these counts: the production checkpoint layout owns that distinction.
    for count in [1, 8, 64] {
        let fired = Arc::new(AtomicBool::new(false));
        let callback = fired.clone();
        let token = PlanningCancellation::from_flag(fired.clone());
        let result = crate::cancellation::after_checkpoint_action(
            count,
            move || callback.store(true, Ordering::Release),
            || {
                crate::with_planning_cancellation(&token, || {
                    old.with_column_updates(updates(&changes))
                })
            },
        );
        assert!(
            fired.load(Ordering::Acquire),
            "checkpoint {count} was not reached"
        );
        assert_eq!(result.unwrap_err(), "层析规划已取消", "checkpoint {count}");
        assert_eq!(facts(&old), before);
        assert!(!crate::planning_cancelled());
    }
    let fresh = PlanningCancellation::from_flag(Arc::new(AtomicBool::new(false)));
    let (retry, _) =
        crate::with_planning_cancellation(&fresh, || old.with_column_updates(updates(&changes)))
            .unwrap();
    apply_model(&mut columns, &changes);
    exact(&retry, &build(FLAT, &columns));
    assert_eq!(facts(&old), before);
}

#[cfg(feature = "mls-adapter")]
mod source {
    use super::*;
    use crate::SurfaceDelta;
    use nuwax_nav_mls_planner::{
        SurfaceObservation, SurfaceObservations, SurfaceSnapshot, mls_planner::Config,
        surface::GroundEvidence,
    };

    const Q: f32 = 0.125;
    type BlockValues = BTreeMap<[i32; 3], BTreeMap<[i32; 3], SurfaceObservation>>;

    fn configuration() -> Config {
        Config {
            world_frame: "map".into(),
            voxel_size: 0.25,
            robot_height: 0.5,
            surface_closing_radius: 0.0,
            node_spacing_m: 0.5,
            wall_clearance_m: 0.0,
            wall_buffer_m: 0.0,
            wall_buffer_weight: 0.0,
            max_slope_deg: 0.0,
            step_threshold_m: 0.0,
            min_step_tread_depth_m: 0.5,
            step_penalty_weight: 1.0,
            goal_tolerance: 0.1,
            viz_publish_hz: 0.0,
        }
    }

    fn source_values(obstacle_x: Option<i32>, thick: bool) -> BlockValues {
        let mut blocks: BlockValues = BTreeMap::new();
        let mut insert = |key: [i32; 3]| {
            blocks
                .entry(key.map(|value| value.div_euclid(32)))
                .or_default()
                .insert(
                    key,
                    SurfaceObservation {
                        observed_ns: 10,
                        hits: 2,
                    },
                );
        };
        for x in 24..=40 {
            for y in 0..=8 {
                for z in 0..=if thick { 2 } else { 0 } {
                    insert([x, y, z]);
                }
            }
        }
        if let Some(x) = obstacle_x {
            insert([x, 4, 2]);
        }
        blocks
    }

    fn fixture(blocks: &BlockValues) -> (SurfaceSnapshot, SurfaceObservations) {
        let points: Vec<_> = blocks
            .values()
            .flat_map(|values| values.keys())
            .map(|key| {
                (
                    (key[0] as f32 + 0.5) * Q,
                    (key[1] as f32 + 0.5) * Q,
                    (key[2] as f32 + 0.5) * Q,
                )
            })
            .collect();
        let base = SurfaceSnapshot::from_points(&points, configuration(), Q).unwrap();
        let mut observations = SurfaceObservations::new(Q, 32).unwrap();
        for (&key, values) in blocks {
            observations.replace_block(key, values.clone()).unwrap();
        }
        observations.mark_complete_geometry_source();
        (
            base.with_observations(Some(Arc::new(observations.clone()))),
            observations,
        )
    }

    #[test]
    fn incremental_same_source_block_insertion_orders_match_graph_bits_and_route() {
        let blocks = source_values(Some(31), false);
        let (base, _) = fixture(&blocks);
        let forward: Vec<_> = blocks.keys().copied().collect();
        let reverse: Vec<_> = forward.iter().rev().copied().collect();
        assert_ne!(
            forward, reverse,
            "fixture must submit more than one real block"
        );
        let assemble = |order: &[[i32; 3]]| {
            let mut observations = SurfaceObservations::new(Q, 32).unwrap();
            for &block in order {
                observations
                    .replace_block(block, blocks[&block].clone())
                    .unwrap();
            }
            observations.mark_complete_geometry_source();
            base.with_observations(Some(Arc::new(observations)))
        };
        let first_source = assemble(&forward);
        let second_source = assemble(&reverse);
        assert!(
            first_source.same_geometry(&second_source),
            "both share the same coarse source"
        );
        let first = Tomogram::from_surface(&first_source).unwrap();
        let second = Tomogram::from_surface(&second_source).unwrap();
        exact(&first, &second);
        route_exact(
            &first,
            &second,
            point(&first, [25, 2], f64::from(Q)),
            point(&first, [39, 6], f64::from(Q)),
        );
    }

    fn replace_source_column(
        source: &SurfaceSnapshot,
        map: &Tomogram,
        blocks: &mut BlockValues,
        observations: &mut SurfaceObservations,
        column: Column,
        levels: Option<&[i32]>,
    ) -> (SurfaceSnapshot, Tomogram) {
        let before = facts(map);
        let block = [column[0].div_euclid(32), column[1].div_euclid(32), 0];
        let values = blocks.get_mut(&block).unwrap();
        values.retain(|key, _| [key[0], key[1]] != column);
        if let Some(levels) = levels {
            for &z in levels {
                assert!(
                    (0..32).contains(&z),
                    "this fixture replaces one complete z block"
                );
                values.insert(
                    [column[0], column[1], z],
                    SurfaceObservation {
                        observed_ns: 20,
                        hits: 2,
                    },
                );
            }
        }
        let changed = observations
            .replace_block_with_geometry_changes(block, values.clone())
            .unwrap();
        assert_eq!(changed, vec![column]);
        observations.mark_complete_geometry_source();
        let next_source = source.with_observations(Some(Arc::new(observations.clone())));
        let (next, stats) = map
            .updated_from_surface(
                &next_source,
                SurfaceDelta::Columns {
                    basis: source,
                    resolution: f64::from(Q),
                    columns: &changed,
                },
            )
            .unwrap();
        assert!(!stats.rebuilt);
        let cold = Tomogram::from_surface(&next_source).unwrap();
        exact(&next, &cold);
        route_exact(
            &next,
            &cold,
            point(&next, [25, 2], f64::from(Q)),
            point(&next, [39, 6], f64::from(Q)),
        );
        assert_eq!(facts(map), before);
        (next_source, next)
    }

    #[test]
    fn incremental_source_update_orders_and_column_reinsertion_match_graph_bits_and_route() {
        let initial_blocks = source_values(None, false);
        let (initial_source, initial_observations) = fixture(&initial_blocks);
        let initial = Tomogram::from_surface(&initial_source).unwrap();
        let before = facts(&initial);
        let run = |order: [Column; 2]| {
            let mut source = initial_source.clone();
            let mut map = initial.clone();
            let mut observations = initial_observations.clone();
            let mut blocks = initial_blocks.clone();
            let occupied = [0, 2];
            for (column, levels) in [
                (order[0], Some(occupied.as_slice())),
                (order[1], Some(occupied.as_slice())),
                (order[0], None),
                (order[1], None),
                (order[1], Some(occupied.as_slice())),
                (order[0], Some(occupied.as_slice())),
            ] {
                (source, map) = replace_source_column(
                    &source,
                    &map,
                    &mut blocks,
                    &mut observations,
                    column,
                    levels,
                );
            }
            assert_eq!(map.cells[map.cells.len() - 1].column, order[0]);
            for column in order {
                assert_eq!(
                    source
                        .observations()
                        .unwrap()
                        .column_levels(column[0], column[1]),
                    Some(occupied.as_slice())
                );
            }
            map
        };
        let ab = run([[31, 4], [32, 4]]);
        let ba = run([[32, 4], [31, 4]]);
        assert_ne!(
            identity(&ab.cells[ab.cells.len() - 1]),
            identity(&ba.cells[ba.cells.len() - 1])
        );
        exact(&ab, &ba);
        route_exact(
            &ab,
            &ba,
            point(&ab, [25, 2], f64::from(Q)),
            point(&ab, [39, 6], f64::from(Q)),
        );
        assert_eq!(facts(&initial), before);
    }

    #[test]
    fn incremental_metadata_only_refresh_preserves_exact_graph_and_cache_contract() {
        let mut blocks = source_values(Some(31), false);
        let (first, mut observations) = fixture(&blocks);
        let old = Tomogram::from_surface(&first).unwrap();
        let before = facts(&old);
        for (&block, values) in &mut blocks {
            for observation in values.values_mut() {
                observation.observed_ns = 900;
                observation.hits = 17;
            }
            assert!(
                observations
                    .replace_block_with_geometry_changes(block, values.clone())
                    .unwrap()
                    .is_empty()
            );
        }
        observations.mark_complete_geometry_source();
        let timed = first.with_observations(Some(Arc::new(observations)));
        assert!(crate::same_source_geometry(&first, &timed));
        assert_eq!(
            timed
                .observations()
                .unwrap()
                .get([31, 4, 2])
                .unwrap()
                .observed_ns,
            900
        );
        let (next, stats) = old
            .updated_from_surface(&timed, SurfaceDelta::Unchanged { basis: &first })
            .unwrap();
        assert!(!stats.rebuilt);
        assert_eq!(stats.columns_prepared, 0);
        assert_eq!(stats.topology_recomputed, 0);
        assert_eq!(stats.distance_invalidated, 0);
        assert_eq!(stats.distance_relaxed, 0);
        assert_eq!(stats.terrain_replayed, 0);
        assert_eq!(stats.upper_relaxed, 0);
        exact(&next, &Tomogram::from_surface(&timed).unwrap());
        assert_eq!(facts(&old), before);
        let planner = crate::PctPlanner::default();
        let cached = planner.map(&first).unwrap();
        assert!(Arc::ptr_eq(&cached, &planner.map(&timed).unwrap()));
    }

    #[test]
    fn incremental_surface_columns_cross_block_boundary_match_full_rebuild() {
        let first_values = source_values(Some(31), false);
        let (first, mut observations) = fixture(&first_values);
        let old = Tomogram::from_surface(&first).unwrap();
        let before = facts(&old);
        let mut changed = BTreeSet::new();
        for (block, values) in source_values(Some(32), false) {
            changed.extend(
                observations
                    .replace_block_with_geometry_changes(block, values)
                    .unwrap(),
            );
        }
        assert_eq!(changed, BTreeSet::from([[31, 4], [32, 4]]));
        observations.mark_complete_geometry_source();
        let next_source = first.with_observations(Some(Arc::new(observations)));
        let changed: Vec<_> = changed.into_iter().collect();
        let (next, stats) = old
            .updated_from_surface(
                &next_source,
                SurfaceDelta::Columns {
                    basis: &first,
                    resolution: f64::from(Q),
                    columns: &changed,
                },
            )
            .unwrap();
        assert!(!stats.rebuilt);
        assert!(stats.columns_prepared > 0);
        let cold = Tomogram::from_surface(&next_source).unwrap();
        exact(&next, &cold);
        assert_eq!(facts(&old), before);
        assert!(old.cell_at(point(&old, [31, 4], f64::from(Q))).is_none());
        assert!(next.cell_at(point(&next, [31, 4], f64::from(Q))).is_some());
        assert!(next.cell_at(point(&next, [32, 4], f64::from(Q))).is_none());
        route_exact(
            &next,
            &cold,
            point(&next, [25, 2], f64::from(Q)),
            point(&next, [39, 6], f64::from(Q)),
        );
    }

    #[test]
    fn incremental_source_switch_explicit_rebuild_uses_current_evidence() {
        let (first, _) = fixture(&source_values(None, false));
        let old = Tomogram::from_surface(&first).unwrap();
        let before = facts(&old);
        let (thick, _) = fixture(&source_values(None, true));
        let volume = thick
            .clone()
            .with_ground_evidence(GroundEvidence::OccupiedVolume);
        let mut previous = old.clone();
        for source in [&thick, &volume, &first] {
            let (next, stats) = previous
                .updated_from_surface(source, SurfaceDelta::Rebuild)
                .unwrap();
            assert!(stats.rebuilt);
            exact(&next, &Tomogram::from_surface(source).unwrap());
            assert!(next.matches_surface(source));
            previous = next;
        }
        let measured = Tomogram::from_surface(&thick).unwrap();
        let solid = Tomogram::from_surface(&volume).unwrap();
        assert_eq!(
            cell(&measured, [31, 4], 0.25).ground_approximation,
            GroundApproximation::LegacyContactInterval
        );
        assert_eq!(
            cell(&solid, [31, 4], 0.375).ground_approximation,
            GroundApproximation::ExposedVolumeTop
        );
        assert_eq!(facts(&old), before);
    }

    #[test]
    fn incremental_local_surface_fallback_preserves_window_for_unchanged_and_rebuild() {
        let (source, _) = fixture(&source_values(None, false));
        let center = [4.0, 0.5];
        let radius = 0.25;
        let full = Tomogram::from_surface(&source).unwrap();
        let local = Tomogram::local_from_surface(&source, center, radius).unwrap();
        let before = facts(&local);
        assert!(!local.cells.is_empty());
        assert!(local.cells.len() < full.cells.len());
        assert_eq!(local.window, Some((center, radius)));

        for delta in [
            SurfaceDelta::Unchanged { basis: &source },
            SurfaceDelta::Rebuild,
        ] {
            let (next, stats) = local.updated_from_surface(&source, delta).unwrap();
            assert!(
                stats.rebuilt,
                "local sources currently use an explicit rebuild fallback"
            );
            assert_eq!(next.window, Some((center, radius)));
            assert!(!next.cells.is_empty());
            assert!(next.cells.len() < full.cells.len());
            let cold = Tomogram::local_from_surface(&source, center, radius).unwrap();
            exact(&next, &cold);
            assert_eq!(facts(&local), before, "the retained local snapshot changed");
        }
    }
}
