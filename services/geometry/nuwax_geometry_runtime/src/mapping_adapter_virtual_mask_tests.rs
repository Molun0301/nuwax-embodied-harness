// Test-only baseline, frozen before the virtual-mask copy trial on 2026-10-06.
mod virtual_mask_copy_trial {
    use super::*;
    use crate::forbidden_zones::{
        copy_trial_reference, ForbiddenZone, LoadedForbiddenZones, ZoneLoadStatus, ZoneMode,
        ZonePoint, ZoneShape,
    };
    use std::hint::black_box;
    use std::time::{Duration, Instant};

    // Original method body, with only self -> adapter and the compose target changed.
    fn frozen_virtual_obstacles_snapshot(adapter: &MappingAdapter) -> Result<Option<MapState>, String> {
        let Some(loaded) = adapter.applied_forbidden_zones.as_ref() else {
            return Ok(None);
        };
        if !matches!(
            loaded.status,
            crate::forbidden_zones::ZoneLoadStatus::FailClosed(_)
        ) && !loaded.zones.iter().any(|zone| zone.enabled)
        {
            return Ok(None);
        }
        let mut free = adapter
            .raw_map
            .clone()
            .ok_or_else(|| "applied forbidden zones have no physical map geometry".to_string())?;
        free.data.fill(0);
        copy_trial_reference::compose_effective_map(&free, loaded).map(Some)
    }

    fn raw_fixture(side: u32) -> MapState {
        MapState {
            width: side,
            height: side,
            resolution: 0.25,
            origin_x: -0.0,
            origin_y: -2.0,
            frame_id: "physical-map-copy-trial".to_owned(),
            revision: 91,
            timestamp_ns: 1_234_567_890_123,
            data: (0..side as usize * side as usize)
                .map(|index| [-1, 100, 0, 42][index % 4])
                .collect(),
        }
    }

    fn zones_fixture(side: u32, fail_closed: bool) -> LoadedForbiddenZones {
        let extent = f64::from(side) * 0.25;
        LoadedForbiddenZones {
            signature: "copy-trial-applied-only".to_owned(),
            generation: 17,
            status: if fail_closed {
                ZoneLoadStatus::FailClosed("fixture malformed file".to_owned())
            } else {
                ZoneLoadStatus::Valid
            },
            zones: vec![ForbiddenZone {
                id: "inside-rectangle".to_owned(),
                shape: ZoneShape::Rectangle,
                mode: ZoneMode::Inside,
                frame_id: "physical-map-copy-trial".to_owned(),
                vertices: vec![
                    ZonePoint { x: extent * 0.25, y: -2.0 + extent * 0.25 },
                    ZonePoint { x: extent * 0.75, y: -2.0 + extent * 0.75 },
                ],
                center: None,
                radius_m: None,
                stroke_width_m: None,
                enabled: true,
                metadata: serde_json::json!({}),
            }],
        }
    }

    fn adapter_fixture(side: u32, fail_closed: bool) -> MappingAdapter {
        // No start_session, checkpoint install, backend, or socket startup.
        let mut adapter = MappingAdapter::new(sample_config());
        adapter.raw_map = Some(raw_fixture(side));
        adapter.applied_forbidden_zones = Some(zones_fixture(side, fail_closed));
        adapter
    }

    fn assert_map_eq(actual: &MapState, expected: &MapState) {
        assert_eq!(actual.width, expected.width);
        assert_eq!(actual.height, expected.height);
        assert_eq!(actual.resolution.to_bits(), expected.resolution.to_bits());
        assert_eq!(actual.origin_x.to_bits(), expected.origin_x.to_bits());
        assert_eq!(actual.origin_y.to_bits(), expected.origin_y.to_bits());
        assert_eq!(actual.frame_id, expected.frame_id);
        assert_eq!(actual.revision, expected.revision);
        assert_eq!(actual.timestamp_ns, expected.timestamp_ns);
        assert_eq!(actual.data, expected.data);
    }

    fn assert_result_eq(adapter: &MappingAdapter) {
        match (
            adapter.virtual_obstacles_snapshot(),
            frozen_virtual_obstacles_snapshot(adapter),
        ) {
            (Ok(Some(actual)), Ok(Some(expected))) => assert_map_eq(&actual, &expected),
            (Ok(None), Ok(None)) => {},
            (Err(actual), Err(expected)) => assert_eq!(actual, expected),
            _ => panic!("production/frozen virtual result variants differ"),
        }
    }

    #[test]
    fn parity_headers_payload_and_isolation() {
        for fail_closed in [false, true] {
            let adapter = adapter_fixture(8, fail_closed);
            let raw_before = adapter.raw_map.as_ref().unwrap().clone();
            let expected = frozen_virtual_obstacles_snapshot(&adapter).unwrap().unwrap();
            let mut actual = adapter.virtual_obstacles_snapshot().unwrap().unwrap();
            assert_map_eq(&actual, &expected);
            assert!(actual.data.iter().all(|value| matches!(value, 0 | 100)));
            if fail_closed {
                assert!(actual.data.iter().all(|value| *value == 100));
            } else {
                assert_eq!(actual.data[0], 0);
                assert_eq!(actual.data[4 + 4 * 8], 100);
            }
            assert_map_eq(adapter.raw_map.as_ref().unwrap(), &raw_before);
            actual.data.fill(7);
            actual.frame_id.push_str("-mutated-output");
            actual.origin_x = 99.0;
            actual.revision += 1;
            assert_map_eq(adapter.raw_map.as_ref().unwrap(), &raw_before);
            assert_map_eq(&adapter.virtual_obstacles_snapshot().unwrap().unwrap(), &expected);
        }
    }

    #[test]
    fn gates_and_layout_errors_match_frozen() {
        let mut empty = zones_fixture(2, false);
        empty.status = ZoneLoadStatus::Empty;
        empty.zones.clear();
        let mut disabled = zones_fixture(2, false);
        disabled.zones[0].enabled = false;
        for applied in [None, Some(empty), Some(disabled)] {
            for raw in [None, Some(MapState::default())] {
                let mut adapter = MappingAdapter::new(sample_config());
                adapter.raw_map = raw;
                adapter.applied_forbidden_zones = applied.clone();
                assert_result_eq(&adapter);
                assert!(adapter.virtual_obstacles_snapshot().unwrap().is_none());
            }
        }
        for fail_closed in [false, true] {
            let mut adapter = adapter_fixture(2, fail_closed);
            adapter.raw_map = None;
            assert_result_eq(&adapter);
            assert_eq!(adapter.virtual_obstacles_snapshot().err().as_deref(),
                Some("applied forbidden zones have no physical map geometry"));
            let valid = raw_fixture(2);
            let mut invalid = Vec::new();
            let mut raw = valid.clone(); raw.width = 0; invalid.push(raw);
            let mut raw = valid.clone(); raw.height = 0; invalid.push(raw);
            let mut raw = valid.clone(); raw.data.pop(); invalid.push(raw);
            let mut raw = valid.clone(); raw.data.push(0); invalid.push(raw);
            for resolution in [0.0, -0.0, -1.0, f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
                let mut raw = valid.clone(); raw.resolution = resolution; invalid.push(raw);
            }
            for raw in invalid {
                let before = raw.clone();
                adapter.raw_map = Some(raw);
                assert_result_eq(&adapter);
                assert_eq!(adapter.virtual_obstacles_snapshot().err().as_deref(),
                    Some("raw occupancy layout is invalid"));
                assert_map_eq(adapter.raw_map.as_ref().unwrap(), &before);
            }
            // Existing layout validation does not reject nonfinite origins.
            for origin in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let mut raw = valid.clone();
                raw.origin_x = origin;
                raw.origin_y = origin;
                adapter.raw_map = Some(raw);
                assert_result_eq(&adapter);
                assert!(adapter.virtual_obstacles_snapshot().unwrap().is_some());
            }
        }
        let mut adapter = adapter_fixture(2, true);
        adapter.applied_forbidden_zones.as_mut().unwrap().zones.clear();
        assert_result_eq(&adapter);
        assert_eq!(adapter.virtual_obstacles_snapshot().unwrap().unwrap().data, vec![100; 4]);
    }

    #[test]
    fn borrowed_compose_preserves_physical_cells() {
        let raw = raw_fixture(8);
        let before = raw.clone();
        for fail_closed in [false, true] {
            let zones = zones_fixture(8, fail_closed);
            let expected = copy_trial_reference::compose_effective_map(&raw, &zones).unwrap();
            let mut actual = crate::forbidden_zones::compose_effective_map(&raw, &zones).unwrap();
            assert_map_eq(&actual, &expected);
            if !fail_closed {
                assert_eq!(&actual.data[..4], &[-1, 100, 0, 42]);
                assert_eq!(actual.data[4 + 4 * 8], 100);
            }
            actual.data.fill(7);
            assert_map_eq(&raw, &before);
            assert_map_eq(&crate::forbidden_zones::compose_effective_map(&raw, &zones).unwrap(), &expected);
        }
    }

    fn measure_one(adapter: &MappingAdapter, frozen: bool) -> Duration {
        let start = Instant::now();
        let map = if frozen {
            frozen_virtual_obstacles_snapshot(black_box(adapter))
        } else {
            black_box(adapter).virtual_obstacles_snapshot()
        }
        .expect("timed fixture must have a valid layout")
        .expect("timed fixture must have an applied mask");
        // Returned MapState destruction is deliberately included for both arms.
        drop(black_box(map));
        start.elapsed()
    }

    #[test]
    #[ignore = "manual bounded same-process copy trial; use cached Geometry bin test profile"]
    fn copy_cost_baseline() {
        const WARMUP_PAIRS: usize = 2;
        for (side, fail_closed, measured_pairs) in [
            (512, true, 16), (512, false, 4),
            (2048, true, 16), (2048, false, 4), (4096, true, 16),
        ] {
            let adapter = adapter_fixture(side, fail_closed);
            let raw_before = adapter.raw_map.as_ref().unwrap().clone();
            // Fixture construction, output equality, and source checks are untimed.
            assert_result_eq(&adapter);
            for pair in 0..WARMUP_PAIRS {
                for frozen in [pair % 2 != 0, pair % 2 == 0] {
                    black_box(measure_one(&adapter, frozen));
                }
            }
            let mut samples = [Vec::with_capacity(measured_pairs), Vec::with_capacity(measured_pairs)];
            for pair in 0..measured_pairs {
                for frozen in [pair % 2 != 0, pair % 2 == 0] {
                    samples[usize::from(frozen)].push(measure_one(&adapter, frozen));
                }
            }
            assert_result_eq(&adapter);
            assert_map_eq(adapter.raw_map.as_ref().unwrap(), &raw_before);
            for (arm, durations) in ["production", "frozen"].into_iter().zip(samples) {
                let nanos: Vec<_> = durations.iter().map(Duration::as_nanos).collect();
                let total: Duration = durations.iter().copied().sum();
                println!("virtual_mask_copy_trial side={side} mode={} arm={arm} warmup_pairs={WARMUP_PAIRS} measured_pairs={measured_pairs} result_drop=included samples_ns={nanos:?} total_ns={} mean_us={:.3}",
                    if fail_closed { "fail_closed" } else { "inside_rectangle" },
                    total.as_nanos(),
                    total.as_secs_f64() * 1_000_000.0 / measured_pairs as f64);
            }
        }
    }
}
