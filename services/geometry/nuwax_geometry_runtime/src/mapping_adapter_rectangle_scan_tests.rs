mod rectangle_scan_virtual_corpus {
    use super::*;
    use crate::forbidden_zones::{copy_trial_reference, rectangle_scan_tests as corpus, ZoneLoadStatus};

    // Independent caller oracle: preserve virtual-mask gates, but compose the
    // all-free input using the unchanged pre-copy-trial raster implementation.
    fn expected_virtual(adapter: &MappingAdapter) -> Result<Option<MapState>, String> {
        let Some(loaded) = adapter.applied_forbidden_zones.as_ref() else { return Ok(None); };
        if !matches!(loaded.status, ZoneLoadStatus::FailClosed(_))
            && !loaded.zones.iter().any(|zone| zone.enabled) { return Ok(None); }
        let mut free = adapter.raw_map.clone()
            .ok_or_else(|| "applied forbidden zones have no physical map geometry".to_owned())?;
        free.data.fill(0);
        copy_trial_reference::compose_effective_map(&free, loaded).map(Some)
    }

    #[test]
    fn rectangle_scan_actual_virtual_caller_matches_frozen() {
        for case in corpus::corpus() {
            let before = case.raw.clone();
            let mut adapter = MappingAdapter::new(sample_config());
            adapter.raw_map = Some(case.raw);
            adapter.applied_forbidden_zones = Some(case.loaded);
            corpus::assert_same_outcome(&case.name, case.panics,
                || adapter.virtual_obstacles_snapshot(), || expected_virtual(&adapter));
            corpus::assert_map_eq(&case.name, adapter.raw_map.as_ref().unwrap(), &before);
            if case.name.starts_with("rounding/") {
                assert_eq!(adapter.virtual_obstacles_snapshot().unwrap().unwrap().data,
                    [0, 0, 0, 0, 0, 100, 100, 100], "{}", case.name);
            }
        }
    }

    #[test]
    fn rectangle_scan_virtual_missing_map_and_no_applied_gates() {
        for case in corpus::corpus().into_iter().filter(|case|
            matches!(case.name.as_str(), "partial" | "empty-enabled" | "valid-disabled" |
                "empty-disabled" | "fail-closed-enabled" | "fail-closed-disabled")
            || case.name.starts_with("no-zones/")) {
            let mut adapter = MappingAdapter::new(sample_config());
            adapter.applied_forbidden_zones = Some(case.loaded);
            corpus::assert_same_outcome(&case.name, false,
                || adapter.virtual_obstacles_snapshot(), || expected_virtual(&adapter));
        }
        for raw in [None, Some(MapState::default())] {
            let mut adapter = MappingAdapter::new(sample_config()); adapter.raw_map = raw;
            assert!(adapter.virtual_obstacles_snapshot().unwrap().is_none());
        }
    }
}
