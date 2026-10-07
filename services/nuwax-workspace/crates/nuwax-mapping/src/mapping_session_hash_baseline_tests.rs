#[cfg(test)]
mod legacy_fingerprint_cost_trial {
    use super::*;
    use std::{hint::black_box, time::Instant};

    #[test]
    #[ignore = "manual isolated legacy fingerprint measurement, not full ingestion"]
    fn measure_legacy_fingerprint_cost() {
        const POINTS: usize = 100_000;
        const WARMUP: usize = 4;
        const REPS: usize = 16;
        let points: Vec<_> = (0..POINTS).map(|i| Point3::new(
            0.1 + (i % 500) as f64 * 0.01,
            0.1 + ((i / 500) % 200) as f64 * 0.01,
            0.1 + (i % 17) as f64 * 0.01,
        )).collect();
        for _ in 0..WARMUP { black_box(MappingSession::points_fingerprint(black_box(&points))); }
        let mut samples_ns = Vec::with_capacity(REPS);
        for _ in 0..REPS {
            let start = Instant::now();
            black_box(MappingSession::points_fingerprint(black_box(&points)));
            samples_ns.push(start.elapsed().as_nanos());
        }
        println!("legacy_fingerprint_cost points={POINTS} warmup={WARMUP} measured={REPS} source=fixed_synthetic_map_frame scope=fingerprint_only samples_ns={samples_ns:?} total_ns={}", samples_ns.iter().sum::<u128>());
    }
}
