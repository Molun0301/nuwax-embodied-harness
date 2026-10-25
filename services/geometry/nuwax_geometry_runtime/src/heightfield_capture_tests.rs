//! Fixed source-classification cases. These call the actual candidate capture
//! function and Writer; they do not stand in for owner/replay or robot contact.
use super::*;
use nuwax_map_chunks::{Domain, Repository};
const Q: f32 = 0.05;

#[test]
fn dilation_bits_match_geometric_neighborhood_at_coordinate_limits() {
    for c in [[0, 0], [i32::MIN, i32::MAX], [i32::MAX - 2, i32::MIN + 3]] {
        let columns: Vec<_> = window(c).collect();
        // 前32组为混合子集，其后逐个单列：单列膨胀最容易暴露移位跨行或越出81位。
        for phase in 0..32 + columns.len() {
            let input: BTreeSet<_> = columns
                .iter()
                .enumerate()
                .filter(|(i, _)| if phase < 32 { (i * 17 + phase * 11) % 31 < phase } else { *i == phase - 32 })
                .map(|(_, &xy)| xy)
                .collect();
            let expected: BTreeSet<_> = columns
                .iter()
                .copied()
                .filter(|xy| {
                    input.iter().any(|p| {
                        (i64::from(xy[0]) - i64::from(p[0])).abs() <= 1
                            && (i64::from(xy[1]) - i64::from(p[1])).abs() <= 1
                    })
                })
                .collect();
            let actual = dilate(
                window_bits(c),
                input.iter().fold(0, |bits, &xy| bits | 1 << bit_of(c, xy)),
            );
            assert_eq!(actual.count_ones() as usize, expected.len());
            for xy in &columns {
                let bit = (i64::from(xy[0]) - i64::from(c[0]) + 4) * 9 + i64::from(xy[1])
                    - i64::from(c[1])
                    + 4;
                assert_eq!((actual >> bit) & 1 != 0, expected.contains(xy));
            }
        }
    }
}

fn writer() -> Writer {
    Writer::new(Repository::temporary().unwrap(), Q, 4 * 1024 * 1024).unwrap()
}
fn field(z: impl Fn(i32, i32) -> Vec<f64>) -> Vec<[f64; 3]> {
    (-4..=4)
        .flat_map(|x| {
            (-4..=4).flat_map({
                let z = &z;
                move |y| {
                    z(x, y).into_iter().map(move |h| {
                        [
                            (f64::from(x) + 0.5) * f64::from(Q),
                            (f64::from(y) + 0.5) * f64::from(Q),
                            h,
                        ]
                    })
                }
            })
        })
        .collect()
}
fn keyed(points: &[[f64; 3]], dynamic: impl Fn(usize) -> bool) -> Vec<Return> {
    points
        .iter()
        .enumerate()
        .map(|(i, &p)| {
            let k = super::super::voxel_key(p, Q).unwrap();
            Return::from_key(p, [k.0, k.1, k.2], i as u32 * 3 + 4, dynamic(i), Q)
        })
        .collect()
}
fn ingest(
    w: &mut Writer,
    points: &[[f64; 3]],
    stamp: u64,
    dynamic: impl Fn(usize) -> bool,
) -> Vec<u32> {
    let values = keyed(points, dynamic);
    w.observe(values.iter().map(|p| p.key), stamp).unwrap();
    capture(
        w,
        &values,
        &[],
        stamp,
        stamp,
        Arc::new(LayerSource::Auxiliary {
            sensor_id: "heightfield-fixed".into(),
            clock_id: "device".into(),
            runtime_generation: 1,
            session_generation: 1,
        }),
    )
    .unwrap()
}
fn expected(points: &[[f64; 3]], xy: Xy) -> HeightfieldColumn {
    let mut counts = BTreeMap::<i32, [u32; 8]>::new();
    for p in keyed(points, |_| false).iter().filter(|p| p.key[..2] == xy) {
        counts.entry(p.key[2]).or_default()[p.bin.rem_euclid(8) as usize] += 1;
    }
    HeightfieldColumn::from_counts(xy, Q, counts)
        .unwrap()
        .unwrap()
}
fn one(w: &mut Writer, xy: Xy) -> HeightfieldColumn {
    let c = w.ground_column(xy).unwrap();
    assert!(c.legacy.is_empty());
    assert_eq!(c.heightfields.len(), 1);
    c.heightfields.into_iter().next().unwrap().column
}
#[test]
fn heightfield_capture_flat_sparse_and_original_order_have_no_fit_gate() {
    for points in [
        field(|_, _| vec![0.01]),
        vec![[0.025, 0.025, 0.01]],
        field(|x, y| if x == y { vec![0.01] } else { vec![] }),
        field(|x, y| if (x + y) % 2 == 0 { vec![0.01] } else { vec![] }),
    ] {
        let mut a = writer();
        let indices = ingest(&mut a, &points, 20, |_| false);
        assert_eq!(indices.len(), points.len());
        let mut reversed = points.clone();
        reversed.reverse();
        let mut b = writer();
        ingest(&mut b, &reversed, 20, |_| false);
        let xys: BTreeSet<_> = keyed(&points, |_| false)
            .iter()
            .map(|p| [p.key[0], p.key[1]])
            .collect();
        for xy in xys {
            let ca = one(&mut a, xy);
            assert_eq!(ca, expected(&points, xy));
            assert_eq!(ca, one(&mut b, xy));
        }
    }
}
#[test]
fn heightfield_capture_fixed_thickness_and_bimodal_mass_use_all_returns() {
    for thickness in [0.05, 0.10, 0.15] {
        let points = field(|_, _| {
            (0..25)
                .map(|k| 0.01 + thickness * f64::from(k) / 24.)
                .collect()
        });
        let mut w = writer();
        assert_eq!(ingest(&mut w, &points, 20, |_| false).len(), points.len());
        assert_eq!(one(&mut w, [0, 0]), expected(&points, [0, 0]));
    }
    let a = field(|_, _| {
        std::iter::repeat(0.01)
            .take(49)
            .chain(std::iter::repeat(0.11).take(51))
            .collect()
    });
    let b = field(|_, _| {
        std::iter::repeat(0.01)
            .take(51)
            .chain(std::iter::repeat(0.11).take(49))
            .collect()
    });
    let mut w = writer();
    assert_eq!(ingest(&mut w, &a, 20, |_| false).len(), a.len());
    let original = one(&mut w, [0, 0]);
    assert_eq!(original, expected(&a, [0, 0]));
    assert_eq!(ingest(&mut w, &b, 21, |_| false).len(), b.len());
    assert_eq!(
        one(&mut w, [0, 0]),
        original.merge_frame(&expected(&b, [0, 0])).unwrap()
    );
}
#[test]
fn heightfield_capture_real_board_masks_and_mixed_voxels_keep_obstacles() {
    for side in [3, 5, 7] {
        let points = field(|x, y| {
            if x.abs() <= side / 2 && y.abs() <= side / 2 {
                vec![0.01, 0.11]
            } else {
                vec![0.01]
            }
        });
        let mut w = writer();
        let indices = ingest(&mut w, &points, 20, |_| false);
        assert_eq!(indices.len(), 81);
        assert_eq!(
            one(&mut w, [0, 0])
                .members()
                .map(|p| p.0)
                .collect::<Vec<_>>(),
            vec![0]
        );
        assert!(!w.region([0, 0, 2], [0, 0, 2], 1).unwrap().is_empty());
    }
    let points = field(|x, y| {
        if x == 0 && y == 0 {
            vec![0.010, 0.041]
        } else {
            vec![0.01]
        }
    });
    let mut w = writer();
    ingest(&mut w, &field(|_, _| vec![0.01]), 19, |_| false);
    let indices = ingest(&mut w, &points, 20, |_| false);
    assert_eq!(indices.len(), 80);
    assert!(w.ground_column([0, 0]).unwrap().heightfields.is_empty());
    assert!(!w.region([0, 0, 0], [0, 0, 0], 1).unwrap().is_empty());
    let mut dynamic = writer();
    let plain = field(|_, _| vec![0.01]);
    let centre = keyed(&plain, |_| false)
        .iter()
        .position(|p| p.key[..2] == [0, 0])
        .unwrap();
    assert_eq!(ingest(&mut dynamic, &plain, 20, |i| i == centre).len(), 80);
    assert!(dynamic
        .ground_column([0, 0])
        .unwrap()
        .heightfields
        .is_empty());
}
#[test]
fn heightfield_capture_wall_is_real_vertical_evidence_not_a_height_span() {
    for diagonal in [false, true] {
        let points = field(|x, y| {
            if if diagonal { x == y } else { x == 0 } {
                (0..20).map(|k| 0.03 + f64::from(k) * 0.02).collect()
            } else {
                vec![]
            }
        });
        let mut w = writer();
        assert!(ingest(&mut w, &points, 20, |_| false).is_empty());
        assert!(w.ground_column([0, 0]).unwrap().heightfields.is_empty());
        assert!(!w.region([0, 0, 0], [0, 0, 9], 32).unwrap().is_empty());
    }
    let points = vec![[0.025, 0.025, 0.01], [0.025, 0.025, 0.36]];
    let mut w = writer();
    assert_eq!(ingest(&mut w, &points, 20, |_| false).len(), 2);
    assert_eq!(w.ground_column([0, 0]).unwrap().heightfields.len(), 2);
}
#[test]
fn heightfield_capture_far_layers_do_not_merge_through_bridge_bins() {
    let far = field(|_, _| vec![0.01, 0.36]);
    let bridged = field(|_, _| vec![0.01, 0.16, 0.21, 0.36]);
    let mut w = writer();
    assert_eq!(ingest(&mut w, &far, 20, |_| false).len(), far.len());
    assert_eq!(w.ground_column([0, 0]).unwrap().heightfields.len(), 2);
    ingest(&mut w, &bridged, 21, |_| false);
    let next = w.ground_column([0, 0]).unwrap();
    assert!(next.heightfields.len() >= 2);
    assert!(next.heightfields.iter().all(|v| {
        let bounds = v.column.occupied_bounds();
        bounds[1] - bounds[0] <= 5. * f64::from(Q)
    }));
    assert!(next
        .heightfields
        .iter()
        .any(|v| v.column.members().any(|(z, _)| z == 7)));
}
#[test]
fn heightfield_capture_top_only_step_and_slope_keep_local_height() {
    for points in [
        field(|x, y| {
            vec![if x.abs() <= 1 && y.abs() <= 1 {
                0.12
            } else {
                0.01
            }]
        }),
        field(|x, _| vec![if x < 0 { 0.01 } else { 0.16 }]),
        field(|x, y| {
            vec![
                0.2 * (f64::from(x) + 0.5) * f64::from(Q)
                    + if (x + y) % 2 == 0 { 0.015 } else { -0.015 },
            ]
        }),
    ] {
        let mut w = writer();
        assert_eq!(ingest(&mut w, &points, 20, |_| false).len(), points.len());
        for xy in [[-1, 0], [0, 0], [2, 0]] {
            assert_eq!(one(&mut w, xy), expected(&points, xy));
        }
    }
}
#[test]
fn heightfield_capture_temporal_mass_missing_and_actual_deletion_are_distinct() {
    let mut w = writer();
    let mut oracle = None;
    for (stamp, z) in [(20, 0.040), (21, 0.058), (22, 0.049)] {
        let points = field(|_, _| vec![z]);
        let frame = expected(&points, [0, 0]);
        oracle = Some(oracle.map_or(frame.clone(), |old: HeightfieldColumn| {
            old.merge_frame(&frame).unwrap()
        }));
        ingest(&mut w, &points, stamp, |_| false);
        assert_eq!(one(&mut w, [0, 0]), oracle.clone().unwrap());
    }
    let thick = field(|_, _| vec![0.01, 0.11]);
    let mut w = writer();
    ingest(&mut w, &thick, 20, |_| false);
    let original = one(&mut w, [0, 0]);
    let low = field(|_, _| vec![0.01]);
    ingest(&mut w, &low, 21, |_| false);
    assert_eq!(
        one(&mut w, [0, 0]),
        original.merge_frame(&expected(&low, [0, 0])).unwrap()
    );
    w.remove([[0, 0, 2]]).unwrap();
    assert_eq!(
        one(&mut w, [0, 0])
            .members()
            .map(|p| p.0)
            .collect::<Vec<_>>(),
        vec![0]
    );
}
#[test]
fn heightfield_capture_cross_block_mass_phase_and_legacy_migration_are_explicit() {
    for offset in [0.0, 0.002] {
        let points = vec![[0.025, 0.025, 0.001 + offset]; 90]
            .into_iter()
            .chain(vec![[0.025, 0.025, 0.049 + offset]; 10])
            .collect::<Vec<_>>();
        let mut w = writer();
        ingest(&mut w, &points, 20, |_| false);
        assert_eq!(one(&mut w, [0, 0]), expected(&points, [0, 0]));
    }
    for xy_shift in [0., 32. * f64::from(Q)] {
        let mut points = field(|_, _| vec![1.59, 1.61]);
        for p in &mut points {
            p[0] += xy_shift;
            p[1] += xy_shift;
        }
        let mut w = writer();
        assert_eq!(ingest(&mut w, &points, 20, |_| false).len(), points.len());
        let xy = if xy_shift == 0. { [0, 0] } else { [32, 32] };
        assert_eq!(one(&mut w, xy), expected(&points, xy));
    }
    let mut w = writer();
    let original = field(|_, _| vec![0.01]);
    w.observe(keyed(&original, |_| false).iter().map(|p| p.key), 19)
        .unwrap();
    let mut blocks: BTreeMap<Key, BTreeMap<u16, LayerObservation>> = BTreeMap::new();
    for p in keyed(&original, |_| false) {
        let local = nuwax_map_chunks::local_key(p.key);
        blocks
            .entry(nuwax_map_chunks::block_key(p.key))
            .or_default()
            .insert(
                local,
                LayerObservation {
                    members: vec![local],
                    plane_origin: p.point,
                    normal: [0., 0., 1.],
                    samples: vec![LayerSample {
                        source: Arc::new(LayerSource::Legacy),
                        point: p.point,
                        source_sequence: 19,
                        source_timestamp_ns: 19,
                        point_index: p.index,
                    }],
                },
            );
    }
    for (b, v) in blocks {
        w.replace_layer_observations(b, v).unwrap();
    }
    ingest(&mut w, &original, 20, |_| false);
    assert_eq!(one(&mut w, [0, 0]), expected(&original, [0, 0]));
    let root = w
        .snapshot(
            Domain {
                session: "capture-fixed".into(),
                generation: 1,
                frame_generation: 1,
                producer_generation: 1,
            },
            1,
            20,
        )
        .unwrap();
    assert_eq!(root.version, 5);
}

#[test]
fn heightfield_capture_only_middle_frame_preserves_independent_old_distributions() {
    let far = field(|_, _| vec![0.01, 0.36]);
    let middle = field(|_, _| vec![0.18]);
    let mut w = writer();
    assert_eq!(ingest(&mut w, &far, 20, |_| false).len(), far.len());
    let before = w.ground_column([0, 0]).unwrap();
    assert_eq!(before.heightfields.len(), 2);
    let indices = ingest(&mut w, &middle, 21, |_| false);
    assert!(
        indices.is_empty(),
        "an ambiguous middle return is not a ground claim"
    );
    assert_eq!(
        w.ground_column([0, 0]).unwrap(),
        before,
        "missing old returns do not erase either old distribution"
    );
    let occupied = w.region([0, 0, 0], [0, 0, 7], 16).unwrap();
    assert_eq!(
        occupied.iter().map(|p| p.0[2]).collect::<Vec<_>>(),
        vec![0, 3, 7]
    );
    eprintln!(
        "H3 only_middle=.18 old=.01/.36 indices=0 old_distributions_exact=true occupancy=0,3,7"
    );
}

#[test]
fn heightfield_capture_claim_only_withdrawal_keeps_original_legacy_diagnostics() {
    let mut w = writer();
    w.observe([[0, 0, 0]], 10).unwrap();
    w.observe([[0, 0, 1]], 20).unwrap();
    let source = Arc::new(LayerSource::Auxiliary {
        sensor_id: "legacy-claim".into(),
        clock_id: "original".into(),
        runtime_generation: 1,
        session_generation: 1,
    });
    let sample = LayerSample {
        source,
        point: [0.025, 0.025, 0.06],
        source_sequence: 0,
        source_timestamp_ns: 20,
        point_index: 4,
    };
    let legacy = LayerObservation {
        members: vec![0, 1024],
        plane_origin: [0.025, 0.025, 0.06],
        normal: [0., 0., 1.],
        samples: vec![sample.clone()],
    };
    w.replace_layer_observations([0, 0, 0], [(0, legacy)].into())
        .unwrap();
    // This current voxel has actual dynamic counter-evidence. Only its ground
    // claim is withdrawn; the original sample and real occupied voxel remain.
    assert!(ingest(&mut w, &[[0.025, 0.025, 0.06]], 21, |_| true).is_empty());
    let column = w.ground_column([0, 0]).unwrap();
    assert!(column.heightfields.is_empty());
    assert_eq!(column.legacy.len(), 1);
    assert_eq!(column.legacy[0].1.members, vec![0]);
    assert_eq!(column.legacy[0].1.samples, vec![sample]);
    assert_eq!(w.region([0, 0, 0], [0, 0, 1], 4).unwrap().len(), 2);
    let root = w
        .snapshot(
            Domain {
                session: "claim-history".into(),
                generation: 1,
                frame_generation: 0,
                producer_generation: 1,
            },
            1,
            21,
        )
        .unwrap();
    assert_eq!(root.version, 5);
    let bytes = w
        .repository()
        .read_object(root.blocks[0].object, nuwax_map_chunks::MAX_BLOCK_BYTES)
        .unwrap();
    assert!(bytes.starts_with(b"NUWAXBL4"));
}
