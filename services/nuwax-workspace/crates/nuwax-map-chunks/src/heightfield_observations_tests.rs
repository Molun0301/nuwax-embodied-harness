//! Data-level oracles only: returns below are pre-associated, not classified.
//! No Writer/capture/robot path is claimed by these tests.
use super::*;

const Q: f32 = 0.05;
// Test-only conversion of fixed numeric examples into the producer-count API.
// This is not the real capture/association path or its source identity admission.
fn column(zs: impl IntoIterator<Item = f64>) -> HeightfieldColumn {
    let inverse = 1.0 / f64::from(Q);
    let mut counts = BTreeMap::<i32, [u32; HEIGHT_SUB_BINS]>::new();
    for z in zs {
        let scaled = z * inverse;
        let member = scaled.floor() as i32;
        let bin = ((scaled - f64::from(member)) * HEIGHT_SUB_BINS as f64).floor() as usize;
        counts.entry(member).or_default()[bin] += 1;
    }
    HeightfieldColumn::from_counts([0, 0], Q, counts)
        .unwrap()
        .unwrap()
}
fn repeated(z: f64, count: usize) -> impl Iterator<Item = f64> {
    std::iter::repeat(z).take(count)
}
fn mass(column: &HeightfieldColumn, z: i32) -> f64 {
    column.members[&z].iter().sum()
}

#[test]
fn heightfield_v1_thick_distributions_keep_members_without_plane() {
    let q = f64::from(Q);
    for cells in [1, 2, 3] {
        let zs: Vec<_> = (0..cells * 8)
            .map(|bin| (f64::from(bin) + 0.5) * q / 8.0)
            .collect();
        let actual = column(zs);
        assert_eq!(
            actual.members.keys().copied().collect::<Vec<_>>(),
            (0..cells).collect::<Vec<_>>()
        );
        assert_eq!(actual.occupied_bounds(), [0.0, f64::from(cells) * q]);
        assert!((actual.height() - f64::from(cells) * q * 0.5).abs() <= q / 16.0);
        assert!(actual.members.values().flatten().all(|v| *v > 0.0));
        eprintln!(
            "HFV1 thick cells={cells} height={} bounds={:?}",
            actual.height(),
            actual.occupied_bounds()
        );
    }
    let isolated = column([0.123]);
    assert_eq!(
        isolated.members.len(),
        1,
        "one return needs no plane/normal"
    );
    assert!(isolated.height().is_finite());
}

#[test]
fn heightfield_v1_column_mass_avoids_voxel_phase_and_bimodal_jump() {
    let a = column(repeated(0.001, 90).chain(repeated(0.049, 10)));
    let b = column(repeated(0.003, 90).chain(repeated(0.051, 10)));
    assert_eq!(a.members.len(), 1);
    assert_eq!(b.members.len(), 2);
    assert!((mass(&b, 0) - 0.9).abs() < 1e-15);
    assert!((mass(&b, 1) - 0.1).abs() < 1e-15);
    assert!(((b.height() - a.height()) - 0.002).abs() <= f64::from(Q) / 8.0);
    let left = column(repeated(0.04, 49).chain(repeated(0.058, 51)));
    let right = column(repeated(0.04, 51).chain(repeated(0.058, 49)));
    assert!((left.height() - right.height()).abs() <= f64::from(Q) / 8.0);
    assert_eq!(
        left.members.keys().collect::<Vec<_>>(),
        right.members.keys().collect::<Vec<_>>()
    );
    eprintln!(
        "HFV1 phase={} -> {}; bimodal={} -> {}",
        a.height(),
        b.height(),
        left.height(),
        right.height()
    );
}

#[test]
fn heightfield_v1_temporal_merge_is_bounded_and_geometry_quantized() {
    let q = f64::from(Q);
    let first = column([0.040]);
    let second = first.merge_frame(&column([0.058])).unwrap();
    let third = second.merge_frame(&column([0.049])).unwrap();
    assert_eq!(first.height(), 6.5 * q / 8.0);
    assert_eq!(third.height(), 7.5 * q / 8.0);
    assert!((second.height() - q).abs() <= q / 16.0);
    assert_eq!(
        third.members.keys().copied().collect::<Vec<_>>(),
        vec![0, 1]
    );
    assert!((mass(&third, 0) - 0.75).abs() < 1e-15);
    assert!((mass(&third, 1) - 0.25).abs() < 1e-15);
    let a = column(repeated(0.001, 99).chain([0.049]));
    let b = column(repeated(0.001, 98).chain(repeated(0.049, 2)));
    assert_ne!(a, b, "statistical distribution really changed");
    assert!(a.geometry_eq(&b), "unchanged quantized h/members/bounds");
    let moved = column([0.020]);
    assert!(!a.geometry_eq(&moved));
    let mut sustained = first.clone();
    for _ in 0..16 {
        sustained = sustained.merge_frame(&column([0.080])).unwrap();
    }
    assert!(
        (sustained.height() - 0.080).abs() <= q / 8.0,
        "no previous-output deadband freeze"
    );
    eprintln!(
        "HFV1 frames .040/.058/.049 = {:?}",
        [first.height(), second.height(), third.height()]
    );
}

#[test]
fn heightfield_v1_deletion_empty_and_underflow_never_invent_height_zero() {
    let q = f64::from(Q);
    let original = column(repeated(0.001, 99).chain([0.12]));
    assert_eq!(
        original.members.keys().copied().collect::<Vec<_>>(),
        vec![0, 2]
    );
    assert_eq!(original.occupied_bounds(), [0.0, 3.0 * q]);
    assert_eq!(
        original.height(),
        0.5 * q / 8.0,
        "trimmed high mass still remains occupied"
    );
    let live = BTreeSet::from([0]);
    let removed = original.retain_members(&live).unwrap();
    assert_eq!(removed.occupied_bounds(), [0.0, q]);
    assert!(!original.geometry_eq(&removed));
    assert!(original.retain_members(&BTreeSet::new()).is_none());
    assert!(HeightfieldColumn::from_counts([0, 0], Q, BTreeMap::new())
        .unwrap()
        .is_none());
    // Numeric finite-storage boundary, not a performance/stability sample.
    let mut decayed = column([0.001]);
    let incoming = column([0.12]);
    for _ in 0..1075 {
        decayed = decayed.merge_frame(&incoming).unwrap();
    }
    assert_eq!(mass(&decayed, 0), 0.0);
    assert!(
        decayed.members.contains_key(&0),
        "zero mass does not erase ownership"
    );
    assert!(decayed.retain_members(&live).is_none());
    assert_eq!(live, BTreeSet::from([0]), "caller occupancy untouched");
}

#[test]
fn heightfield_v1_whole_column_normalizes_across_z_blocks_and_order() {
    let q = f64::from(Q);
    for (low, high) in [
        (-0.015, 0.015),
        (-0.003, 0.003),
        (32.0 * q - 0.003, 32.0 * q + 0.003),
    ] {
        let mut zs: Vec<_> = repeated(low, 90).chain(repeated(high, 10)).collect();
        let joint = column(zs.clone());
        zs.reverse();
        assert_eq!(joint, column(zs));
        let levels: Vec<_> = joint.members.keys().copied().collect();
        assert_eq!(levels.len(), 2);
        assert_ne!(levels[0].div_euclid(32), levels[1].div_euclid(32));
        assert!((mass(&joint, levels[0]) - 0.9).abs() < 1e-15);
        assert!((mass(&joint, levels[1]) - 0.1).abs() < 1e-15);
        // Storage fragments retain fractions. They must not each re-normalize.
        let fragments: Vec<_> = joint
            .members()
            .map(|(z, bins)| (z.div_euclid(32), *bins))
            .collect();
        assert!((fragments[0].1.iter().sum::<f64>() - 0.9).abs() < 1e-15);
        assert!((fragments[1].1.iter().sum::<f64>() - 0.1).abs() < 1e-15);
        assert_eq!(
            decode_heightfield_column(&encode_heightfield_column(&joint).unwrap()).unwrap(),
            joint
        );
    }
    let points = vec![-0.2, 0.001, 0.023, 0.049, 0.051, 0.12];
    assert_eq!(
        column(points.clone()),
        column(points.iter().copied().chain(points.iter().copied()))
    );
}

#[test]
fn heightfield_v1_partial_bin_trim_preserves_sparse_outliers_and_holes() {
    let q = f64::from(Q);
    let actual = column(repeated(0.5 * q / 8.0, 15).chain(repeated(7.5 * q / 8.0, 85)));
    // Trim retains .05 low + .75 high, mean=7.0625 sub-bins, rounds to7.5.
    // Untrimmed mean=6.45 would round to6.5: this tests partial-bin trimming.
    assert_eq!(actual.height(), 7.5 * q / 8.0);
    assert_eq!(actual.occupied_bounds(), [0.0, q]);
    assert!(actual.members[&0][0] > 0.0 && actual.members[&0][7] > 0.0);
    let negative = column([-3.5 * q / 8.0]);
    assert_eq!(negative.height(), -3.5 * q / 8.0);
    let sparse = column([0.001, 0.15]);
    assert!(!sparse.members.contains_key(&1));
    assert_eq!(
        sparse.members.len(),
        2,
        "bounds do not fill unobserved voxels"
    );
    // Exact half-bin symmetric inputs, translated across negative/positive zero.
    // One world-grid tie rule must translate by exactly one whole bin.
    let width = q / 8.0;
    let ties: Vec<_> = (-2..=2)
        .map(|offset| {
            let centre = f64::from(offset) * width;
            let value = column([centre - 0.5 * width, centre + 0.5 * width]);
            (offset, value.height(), (f64::from(offset) + 0.5) * width)
        })
        .collect();
    eprintln!("HFV1 exact-boundary ties (offset, actual, expected)={ties:?}");
    assert_eq!(
        column([0.0]),
        column([-0.0]),
        "signed zero selects the same real bin"
    );
    for (_, actual, expected) in &ties {
        assert_eq!(actual, expected);
    }
    for pair in ties.windows(2) {
        assert_eq!(pair[1].1 - pair[0].1, width);
    }
}

#[test]
fn heightfield_v1_codec_rejects_invalid_mass_and_implicit_legacy_upgrade() {
    let actual = column([0.001, 0.07]);
    let encoded = encode_heightfield_column(&actual).unwrap();
    let value: serde_json::Value = serde_json::from_slice(&encoded).unwrap();
    assert_eq!(value["representation"], "heightfield_v1");
    assert!(serde_json::from_slice::<crate::LayerObservation>(&encoded).is_err());
    let legacy = crate::LayerObservation {
        members: vec![0],
        plane_origin: [0.025, 0.025, 0.001],
        normal: [0.0, 0.0, 1.0],
        samples: vec![crate::LayerSample {
            source: std::sync::Arc::new(crate::LayerSource::Legacy),
            point: [0.025, 0.025, 0.001],
            source_sequence: 1,
            source_timestamp_ns: 1,
            point_index: 0,
        }],
    };
    let old_bytes = serde_json::to_vec(&legacy).unwrap();
    assert_eq!(
        serde_json::from_slice::<crate::LayerObservation>(&old_bytes).unwrap(),
        legacy
    );
    assert!(decode_heightfield_column(&old_bytes).is_err());
    let mut invalid = value.clone();
    invalid["representation"] = "heightfield_v2".into();
    assert!(decode_heightfield_column(&serde_json::to_vec(&invalid).unwrap()).is_err());
    let mut invalid = value.clone();
    invalid["normal"] = serde_json::json!([0, 0, 1]);
    assert!(decode_heightfield_column(&serde_json::to_vec(&invalid).unwrap()).is_err());
    for weight in [0.0, -1.0, 2.0] {
        let mut invalid = value.clone();
        invalid["members"] = serde_json::json!([[0, [weight, 0, 0, 0, 0, 0, 0, 0]]]);
        assert!(decode_heightfield_column(&serde_json::to_vec(&invalid).unwrap()).is_err());
    }
    let mut invalid = value.clone();
    invalid["members"]
        .as_array_mut()
        .unwrap()
        .push(value["members"][0].clone());
    assert!(decode_heightfield_column(&serde_json::to_vec(&invalid).unwrap()).is_err());
    let mut invalid = value;
    invalid["members"] = serde_json::json!([]);
    assert!(decode_heightfield_column(&serde_json::to_vec(&invalid).unwrap()).is_err());
}

#[test]
fn heightfield_v1_input_boundaries_do_not_mutate_valid_columns() {
    let original = column([0.001]);
    let bytes = encode_heightfield_column(&original).unwrap();
    let counts = BTreeMap::from([(0, [1, 0, 0, 0, 0, 0, 0, 0])]);
    for q in [0.0, -0.05, f32::NAN, f32::INFINITY] {
        assert!(HeightfieldColumn::from_counts([0, 0], q, counts.clone()).is_err());
    }
    assert!(
        HeightfieldColumn::from_counts([0, 0], Q, BTreeMap::from([(0, [0; 8])]))
            .unwrap()
            .is_none()
    );
    let other = HeightfieldColumn::from_counts([1, 0], Q, counts.clone())
        .unwrap()
        .unwrap();
    assert!(original.merge_frame(&other).is_err());
    let other_q = HeightfieldColumn::from_counts([0, 0], 0.1, counts)
        .unwrap()
        .unwrap();
    assert!(original.merge_frame(&other_q).is_err());
    assert_eq!(encode_heightfield_column(&original).unwrap(), bytes);
    assert_eq!(decode_heightfield_column(&bytes).unwrap(), original);
}
