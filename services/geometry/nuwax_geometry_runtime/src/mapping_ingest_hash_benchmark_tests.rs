// Fixed synthetic map-frame preparation benchmark; no backend submit or transport.
mod canonical_ingest_cost_trial {
    use super::*;
    use std::{hint::black_box, time::Instant};

    const POINTS: usize = 100_000;
    fn points() -> Vec<Point3<f64>> {
        (0..POINTS).map(|i| Point3::new(
            0.1 + (i % 500) as f64 * 0.01,
            0.1 + ((i / 500) % 200) as f64 * 0.01,
            0.1 + (i % 17) as f64 * 0.01,
        )).collect()
    }

    #[test]
    #[ignore = "manual fixed-frame prepare-only measurement; run exact selector separately"]
    fn canonical_prepare_lock_cost() {
        const WARMUP: usize = 4;
        const REPS: usize = 16;
        let points = points();
        let mut value = active_adapter(sample_config());
        value.session = MappingSession::with_capacity(POINTS);
        let adapter = parking_lot::Mutex::new(value);
        let mut wait_ns = Vec::with_capacity(REPS);
        let mut held_ns = Vec::with_capacity(REPS);
        for index in 0..WARMUP + REPS {
            let sequence = index as u64 + 1;
            let receipt_ns = 2_000_000_000 + sequence * 1_000_000;
            let source_ns = 1_000_000_000 + sequence * 1_000_000;
            let waiting = Instant::now();
            let mut guard = adapter.lock();
            let acquired = Instant::now();
            let prepared = black_box(&mut *guard).prepare_backend_compute(
                black_box(&points), (0.0, 0.0), 12, receipt_ns, "map", source_ns, MappingCloudSource::Canonical(AdmittedFrameKey::from_admitted_source(1, 7, sequence as i64).unwrap()),
            ).expect("valid admitted map-frame fixture").expect("each new frame prepares work");
            let prepared_at = Instant::now();
            drop(guard);
            if index >= WARMUP {
                wait_ns.push(acquired.duration_since(waiting).as_nanos());
                held_ns.push(prepared_at.duration_since(acquired).as_nanos());
            }
            // Exact checks, token retirement, and output destruction stay outside timing.
            assert_eq!(prepared.input.registered_points_map, points);
            assert_eq!(prepared.input.source_timestamp_ns, source_ns);
            assert_eq!(prepared.token.source_frame_sequence, sequence);
            let mut guard = adapter.lock();
            assert_eq!(guard.session.current_size(), POINTS);
            assert_eq!(guard.session.generation(), sequence);
            assert_eq!(guard.session.last_seen_at_ns(), receipt_ns);
            assert_eq!(guard.revision, 0);
            assert_eq!(guard.coalesced_frame_count, 0);
            assert!(guard.fail_backend_compute(&prepared.token));
            drop(guard);
            drop(prepared);
        }
        let mut guard = adapter.lock();
        assert_eq!(guard.session.make_contiguous_points(), points.as_slice());
        assert!(guard.in_flight_compute.is_none());
        drop(guard);
        println!("canonical_ingest_cost points={POINTS} warmup={WARMUP} measured={REPS} source=fixed_synthetic_map_frame backend=existing_sample_config scope=prepare_only_no_submit setup_checks_drop_retirement=excluded wait_ns={wait_ns:?} held_ns={held_ns:?} wait_total_ns={} held_total_ns={}",
            wait_ns.iter().sum::<u128>(), held_ns.iter().sum::<u128>());
    }
}
