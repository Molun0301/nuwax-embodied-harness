// Run exactly one ignored selector per process; neither selector runs rectangles
// or another map size before its FailClosed control.
mod virtual_mask_fail_closed_isolation {
    use super::*;
    use crate::forbidden_zones::{copy_trial_reference, rectangle_scan_tests::assert_map_eq,
        ForbiddenZone, LoadedForbiddenZones, ZoneLoadStatus, ZoneMode, ZonePoint, ZoneShape};
    use std::hint::black_box;
    use std::time::{Duration, Instant};

    fn frozen_virtual(adapter: &MappingAdapter) -> Result<Option<MapState>, String> {
        let Some(loaded) = adapter.applied_forbidden_zones.as_ref() else { return Ok(None); };
        if !matches!(loaded.status, ZoneLoadStatus::FailClosed(_))
            && !loaded.zones.iter().any(|zone| zone.enabled) { return Ok(None); }
        let mut free = adapter.raw_map.clone()
            .ok_or_else(|| "applied forbidden zones have no physical map geometry".to_owned())?;
        free.data.fill(0);
        copy_trial_reference::compose_effective_map(&free, loaded).map(Some)
    }

    fn adapter_fixture(side: u32) -> MappingAdapter {
        let mut adapter = MappingAdapter::new(sample_config());
        adapter.raw_map = Some(MapState {
            width: side, height: side, resolution: 0.25, origin_x: -0.0, origin_y: -2.0,
            frame_id: "physical-map-copy-trial".to_owned(), revision: 91,
            timestamp_ns: 1_234_567_890_123,
            data: (0..side as usize * side as usize).map(|i| [-1, 100, 0, 42][i % 4]).collect(),
        });
        let extent = f64::from(side) * 0.25;
        adapter.applied_forbidden_zones = Some(LoadedForbiddenZones {
            signature: "copy-trial-applied-only".to_owned(), generation: 17,
            status: ZoneLoadStatus::FailClosed("fixture malformed file".to_owned()),
            zones: vec![ForbiddenZone {
                id: "inside-rectangle".to_owned(), shape: ZoneShape::Rectangle, mode: ZoneMode::Inside,
                frame_id: "physical-map-copy-trial".to_owned(),
                vertices: vec![
                    ZonePoint { x: extent * 0.25, y: -2.0 + extent * 0.25 },
                    ZonePoint { x: extent * 0.75, y: -2.0 + extent * 0.75 },
                ],
                center: None, radius_m: None, stroke_width_m: None, enabled: true,
                metadata: serde_json::json!({}),
            }],
        });
        adapter
    }

    fn assert_valid(adapter: &MappingAdapter, before: &MapState) {
        let actual = adapter.virtual_obstacles_snapshot().expect("valid production layout")
            .expect("production FailClosed mask");
        let expected = frozen_virtual(adapter).expect("valid frozen layout")
            .expect("frozen FailClosed mask");
        assert_map_eq("isolated FailClosed output", &actual, &expected);
        assert!(actual.data.iter().all(|value| *value == 100));
        assert_map_eq("isolated FailClosed raw input", adapter.raw_map.as_ref().unwrap(), before);
    }

    fn measure_one(adapter: &MappingAdapter) -> Duration {
        let start = Instant::now();
        let map = black_box(adapter).virtual_obstacles_snapshot()
            .expect("timed fixture must have a valid layout")
            .expect("timed fixture must have an applied mask");
        drop(black_box(map)); // Returned MapState destruction is timed.
        start.elapsed()
    }

    fn run_isolated(side: u32) {
        const WARMUP_CALLS: usize = 2;
        const MEASURED_CALLS: usize = 16;
        let adapter = adapter_fixture(side);
        let raw_before = adapter.raw_map.as_ref().unwrap().clone();
        for _ in 0..WARMUP_CALLS {
            black_box(measure_one(&adapter));
        }
        let mut samples = Vec::with_capacity(MEASURED_CALLS);
        for _ in 0..MEASURED_CALLS {
            samples.push(measure_one(&adapter));
        }
        // The frozen copy workload runs only after all production samples.
        assert_valid(&adapter, &raw_before);
        let nanos: Vec<_> = samples.iter().map(Duration::as_nanos).collect();
        let total: Duration = samples.iter().copied().sum();
        println!("virtual_mask_fail_closed_isolation side={side} mode=fail_closed arm=production warmup_calls={WARMUP_CALLS} measured_calls={MEASURED_CALLS} result_drop=included samples_ns={nanos:?} total_ns={} mean_us={:.3}",
            total.as_nanos(), total.as_secs_f64() * 1_000_000.0 / MEASURED_CALLS as f64);
    }

    #[test]
    #[ignore = "manual isolated control; run this exact selector alone in a fresh process"]
    fn isolated_fail_closed_512() { run_isolated(512); }

    #[test]
    #[ignore = "manual isolated control; run this exact selector alone in a fresh process"]
    fn isolated_fail_closed_4096() { run_isolated(4096); }
}
