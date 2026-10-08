use super::*;

fn statistics(points: &[[f64; 3]]) -> ObservationStatistics {
    let mut stats = ObservationStatistics::from_point(points[0]).unwrap();
    for &p in &points[1..] {
        stats.push(p).unwrap();
    }
    stats
}
fn near(a: f64, b: f64, tolerance: f64) {
    assert!(
        (a - b).abs() <= tolerance,
        "{a:?} != {b:?}, tol={tolerance}"
    );
}
fn near_moments(a: CenteredMoments3, b: CenteredMoments3, tolerance: f64) {
    assert_eq!(a.count(), b.count());
    for (x, y) in a
        .mean()
        .into_iter()
        .chain(a.m2())
        .zip(b.mean().into_iter().chain(b.m2()))
    {
        near(x, y, tolerance);
    }
}
fn points() -> Vec<[f64; 3]> {
    (0..137)
        .map(|i| {
            let x = i as f64 * 0.0037;
            [1e6 + x, -2e6 + x.sin(), 3e6 + x * x - 0.2 * x]
        })
        .collect()
}

#[test]
fn direct_samples_incremental_and_batch_merge_agree() {
    let p = points();
    let direct = statistics(&p);
    let mut batches = p.chunks(13).map(statistics);
    let first = batches.next().unwrap();
    let merged = batches.try_fold(first, |a, b| a.merged(&b)).unwrap();
    near_moments(direct.moments(), merged.moments(), 2e-7);
    assert_eq!(direct.observed_extent(), merged.observed_extent());
    // Independent two-pass oracle, not only two invocations of the same merge.
    let mean: [f64; 3] =
        std::array::from_fn(|j| p.iter().map(|v| v[j]).sum::<f64>() / p.len() as f64);
    for j in 0..3 {
        // Large frame translations incur a few representable f64 steps.
        near(
            direct.moments().mean()[j],
            mean[j],
            8.0 * f64::EPSILON * mean[j].abs().max(1.0),
        );
    }
    for (k, (i, j)) in PAIRS.into_iter().enumerate() {
        let m2: f64 = p.iter().map(|v| (v[i] - mean[i]) * (v[j] - mean[j])).sum();
        near(direct.moments().m2()[k], m2, 2e-7);
    }
}

#[test]
fn single_point_scatter_is_unknown() {
    let single = ObservationStatistics::from_point([1.0, 2.0, 3.0]).unwrap();
    assert_eq!(single.moments().count(), 1);
    assert_eq!(single.moments().sample_scatter(), None);
    let same = single.merged(&single).unwrap();
    // Two actually supplied identical returns have empirical zero scatter.
    // This still conveys no calibrated uncertainty or independence assumption.
    assert_eq!(same.moments().sample_scatter(), Some([0.0; 6]));
}

#[test]
fn rigid_transform_matches_transformed_raw_returns_with_cross_terms() {
    let p = [
        [-2.0, 0.3, 0.5],
        [1.0, -0.7, 1.4],
        [0.1, 2.0, -0.8],
        [0.6, 0.4, 3.0],
    ];
    let a = 0.37_f64;
    let b = -0.62_f64;
    let r = [
        [a.cos() * b.cos(), -a.sin(), a.cos() * b.sin()],
        [a.sin() * b.cos(), a.cos(), a.sin() * b.sin()],
        [-b.sin(), 0.0, b.cos()],
    ];
    let t = [1.3, -0.9, 4.2];
    let transformed: Vec<_> = p
        .iter()
        .map(|p| std::array::from_fn(|i| t[i] + (0..3).map(|j| r[i][j] * p[j]).sum::<f64>()))
        .collect();
    let predicted = statistics(&p).moments().rigid_transform(r, t).unwrap();
    near_moments(predicted, statistics(&transformed).moments(), 2e-14);
    assert!(predicted.m2()[1].abs() > 0.01);
    assert_eq!(
        statistics(&p)
            .moments()
            .rigid_transform([[2.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], t),
        Err(StatisticsError::InvalidRotation)
    );
}

#[test]
fn failed_math_update_preserves_input() {
    let p = ObservationStatistics::from_point([f64::MAX, 0.0, 0.0]).unwrap();
    let mut q = p;
    assert_eq!(
        q.push([-f64::MAX, 0.0, 0.0]),
        Err(StatisticsError::NonFinite)
    );
    assert_eq!(q, p);
    for invalid in [f64::NAN, f64::INFINITY] {
        assert_eq!(q.push([invalid, 0.0, 0.0]), Err(StatisticsError::NonFinite));
        assert_eq!(q, p);
    }
    let mut m = p.moments();
    m.count = 1_u64 << 53;
    assert_eq!(m.merged(&p.moments()), Err(StatisticsError::CountOverflow));
}

#[test]
fn known_cross_moments_and_sample_denominator_are_not_mean_uncertainty() {
    let stats = statistics(&[[0.0, 0.0, 0.0], [2.0, 4.0, 6.0]]);
    assert_eq!(stats.moments().mean(), [1.0, 2.0, 3.0]);
    assert_eq!(stats.moments().m2(), [2.0, 4.0, 6.0, 8.0, 12.0, 18.0]);
    assert_eq!(
        stats.moments().sample_scatter(),
        Some([2.0, 4.0, 6.0, 8.0, 12.0, 18.0])
    );
    assert_eq!(stats.observed_extent(), ([0.0, 0.0, 0.0], [2.0, 4.0, 6.0]));
}
