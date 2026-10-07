mod admitted_mapping_frame_tests {
    use super::*;

    fn key(sequence: i64) -> MappingCloudSource {
        MappingCloudSource::Canonical(AdmittedFrameKey::from_admitted_source(1, 7, sequence).unwrap())
    }

    #[test]
    fn accepted_frame_replay_after_backend_failure_does_not_append_or_allocate_token() {
        let mut adapter = active_adapter(sample_config());
        let points = [Point3::new(0.25, 0.25, 0.2)];
        let prepared = adapter.prepare_backend_compute(&points, (0.0, 0.0), 12,
            1_000_000_000, "map", 0, key(3)).unwrap().unwrap();
        assert_eq!(prepared.input.source_timestamp_ns, 0);
        assert_eq!(adapter.session.last_seen_at_ns(), 1_000_000_000);
        assert_eq!(adapter.latest_pointcloud_timestamp_ns, 0);
        assert!(adapter.fail_backend_compute(&prepared.token));
        let before = (adapter.session.current_size(), adapter.session.generation(),
            adapter.pointcloud_count, adapter.next_source_frame_sequence,
            adapter.next_token_sequence, adapter.revision, adapter.global_revision);
        assert!(adapter.prepare_backend_compute(&points, (9.0, 8.0), 48,
            1_000_000_001, "map", 0, key(3)).unwrap().is_none());
        assert_eq!(before, (adapter.session.current_size(), adapter.session.generation(),
            adapter.pointcloud_count, adapter.next_source_frame_sequence,
            adapter.next_token_sequence, adapter.revision, adapter.global_revision));
        assert_eq!(adapter.session.make_contiguous_points(), points.as_slice());
        assert!(adapter.in_flight_compute.is_none());
        assert!(adapter.take_latest_accepted_map_points().is_empty());
    }

    #[test]
    fn coalesced_frame_key_remains_eligible_and_distinct_identical_geometry_is_accepted() {
        let mut adapter = active_adapter(sample_config());
        let points = [Point3::new(0.25, 0.25, 0.2)];
        let first = adapter.prepare_backend_compute(&points, (0.0, 0.0), 12,
            1_000_000_000, "map", 10, key(1)).unwrap().unwrap();
        assert!(adapter.prepare_backend_compute(&points, (0.0, 0.0), 12,
            1_000_000_001, "map", 11, key(2)).unwrap().is_none());
        assert_eq!(adapter.coalesced_frame_count, 1);
        assert_eq!(adapter.session.generation(), 1);
        assert!(adapter.fail_backend_compute(&first.token));
        let second = adapter.prepare_backend_compute(&points, (0.0, 0.0), 12,
            1_000_000_001, "map", 11, key(2)).unwrap().unwrap();
        assert_eq!(adapter.session.generation(), 2);
        assert_eq!(adapter.session.current_size(), 2);
        assert_eq!(second.token.source_frame_sequence, 2);
        assert_eq!(second.input.registered_points_map, points);
        assert!(adapter.fail_backend_compute(&second.token));
        assert!(adapter.prepare_backend_compute(&points, (0.0, 0.0), 12,
            1_000_000_002, "map", 10, key(1)).unwrap().is_none());
        assert_eq!(adapter.session.generation(), 2);
        assert_eq!(adapter.revision, 0);
    }

    #[test]
    fn invalid_frame_does_not_remember_canonical_key_and_old_receipt_stays_stale() {
        let mut adapter = active_adapter(sample_config());
        let points = [Point3::new(0.25, 0.25, 0.2)];
        assert!(adapter.prepare_backend_compute(&points, (0.0, 0.0), 12,
            1_000_000_000, "other-map", 1, key(1)).is_err());
        assert_eq!(adapter.session.generation(), 0);
        let first = adapter.prepare_backend_compute(&points, (0.0, 0.0), 12,
            1_000_000_000, "map", 1, key(1)).unwrap().unwrap();
        assert!(adapter.fail_backend_compute(&first.token));
        let second = adapter.prepare_backend_compute(&points, (0.0, 0.0), 12,
            2_000_000_000, "map", 2, key(2)).unwrap().unwrap();
        assert!(adapter.fail_backend_compute(&second.token));
        assert!(matches!(adapter.prepare_backend_compute(&points, (0.0, 0.0), 12,
            1_000_000_000, "map", 1, key(1)), Err(nuwax_mapping::SessionError::StaleRevision { .. })));
        assert_eq!(adapter.session.generation(), 2);
        assert_eq!(adapter.next_source_frame_sequence, 2);
    }
}
