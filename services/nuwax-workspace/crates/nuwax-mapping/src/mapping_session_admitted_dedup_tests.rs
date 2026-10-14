use super::*;

fn point(x: f64) -> Point3<f64> {
    Point3::new(x, 2.0, 3.0)
}

fn admitted(backend: i32, generation: i64, sequence: i64) -> AdmittedFrameKey {
    AdmittedFrameKey::from_admitted_source(backend, generation, sequence)
        .expect("positive admitted source identity")
}

fn reset_fingerprint_calls() {
    POINTS_FINGERPRINT_CALLS.with(|calls| calls.set(0));
}

fn fingerprint_calls() -> usize {
    POINTS_FINGERPRINT_CALLS.with(std::cell::Cell::get)
}

#[test]
fn admitted_key_requires_positive_backend_generation_and_sequence() {
    for backend in [i32::MIN, -1, 0] {
        assert_eq!(AdmittedFrameKey::from_admitted_source(backend, 1, 1), None);
    }
    for invalid in [i64::MIN, -1, 0] {
        assert_eq!(AdmittedFrameKey::from_admitted_source(1, invalid, 1), None);
        assert_eq!(AdmittedFrameKey::from_admitted_source(1, 1, invalid), None);
    }
    let first = admitted(1, 1, 1);
    let copied = first;
    let mut keys = HashSet::new();
    assert!(keys.insert(first));
    assert!(!keys.insert(copied));
    assert!(keys.insert(admitted(i32::MAX, i64::MAX, i64::MAX)));
    assert_eq!(keys.len(), 2);
}

#[test]
fn admitted_replay_changes_no_points_generation_evictions_or_watermark() {
    let mut session = MappingSession::with_capacity(2);
    let points = [point(0.0), point(1.0), point(2.0)];
    let key = admitted(1, 10, 20);
    let initial = session.feed_admitted_pointcloud(&points, key, 100).unwrap();
    assert_eq!((initial.accepted, initial.evicted), (2, 1));
    let before = session.snapshot_stats();
    let window = session.clone_points();
    let order = session.dedup_order.clone();

    for received_at_ns in [100, 101, 1_000] {
        let replay = session
            .feed_admitted_pointcloud(&points, key, received_at_ns)
            .unwrap();
        assert_eq!(replay, FeedStats::dedup_only(3, 2, 2, 100));
        assert_eq!(session.snapshot_stats(), before);
        assert_eq!(session.clone_points(), window);
        assert_eq!(session.dedup_order, order);
    }
}

#[test]
fn each_admitted_identity_component_distinguishes_identical_geometry() {
    let mut session = MappingSession::with_capacity(8);
    let points = [point(1.0)];
    for key in [
        admitted(1, 1, 1),
        admitted(1, 1, 2),
        admitted(1, 2, 1),
        admitted(2, 1, 1),
    ] {
        let stats = session.feed_admitted_pointcloud(&points, key, 100).unwrap();
        assert_eq!(stats.accepted, 1);
        assert_eq!(stats.dedup_skipped, 0);
    }
    assert_eq!(session.current_size(), 4);
    assert_eq!(session.generation(), 4);
    assert_eq!(session.dedup_set_size(), 4);
}

#[test]
fn admitted_session_trusts_identity_without_revalidating_frame_content() {
    let mut session = MappingSession::with_capacity(8);
    let key = admitted(1, 1, 1);
    session
        .feed_admitted_pointcloud(&[point(1.0)], key, 100)
        .unwrap();
    let before = session.snapshot_stats();
    reset_fingerprint_calls();

    // 故意违反调用契约：该内容变化必须由 canonical 上游拒绝，session 不重新扫描。
    let replay = session
        .feed_admitted_pointcloud(&[point(9.0), point(10.0)], key, 100)
        .unwrap();
    assert_eq!(replay.dedup_skipped, 2);
    assert_eq!(replay.accepted, 0);
    assert_eq!(session.clone_points(), vec![point(1.0)]);
    assert_eq!(session.snapshot_stats(), before);
    assert_eq!(fingerprint_calls(), 0);
}

#[test]
fn admitted_and_legacy_keys_have_separate_tags_in_the_same_history() {
    let mut session = MappingSession::with_capacity(8);
    let points = [point(1.0)];
    let key = admitted(1, 1, 1);
    let legacy = MappingSession::frame_dedup_key(&points, (0.0, 0.0), 32, 100);
    assert_ne!(legacy, FrameDedupKey::Admitted(key));

    assert_eq!(
        session
            .feed_pointcloud(&points, (0.0, 0.0), 32, 100)
            .unwrap()
            .accepted,
        1
    );
    assert_eq!(
        session
            .feed_admitted_pointcloud(&points, key, 100)
            .unwrap()
            .accepted,
        1
    );
    assert_eq!(session.dedup_set_size(), 2);
    assert!(session.dedup_set.contains(&legacy));
    assert!(session.dedup_set.contains(&FrameDedupKey::Admitted(key)));
    assert_eq!(
        session
            .feed_pointcloud(&points, (0.0, 0.0), 32, 100)
            .unwrap()
            .dedup_skipped,
        1
    );
    assert_eq!(
        session
            .feed_admitted_pointcloud(&points, key, 100)
            .unwrap()
            .dedup_skipped,
        1
    );
    assert_eq!(session.generation(), 2);
}

#[test]
fn mixed_history_shares_fifo_expiry_without_refreshing_duplicate_keys() {
    let mut session = MappingSession::with_capacity(8).with_dedup_history_capacity(2);
    let points = [point(1.0)];
    let first = admitted(1, 1, 1);
    let second = admitted(1, 1, 2);
    let legacy = MappingSession::frame_dedup_key(&points, (0.0, 0.0), 32, 11);
    session.feed_admitted_pointcloud(&points, first, 10).unwrap();
    session.feed_pointcloud(&points, (0.0, 0.0), 32, 11).unwrap();
    session.feed_admitted_pointcloud(&points, second, 12).unwrap();
    assert_eq!(
        session.dedup_order,
        VecDeque::from([legacy, FrameDedupKey::Admitted(second)])
    );
    assert!(!session.dedup_set.contains(&FrameDedupKey::Admitted(first)));
    assert_eq!(
        session
            .feed_pointcloud(&points, (0.0, 0.0), 32, 11)
            .unwrap()
            .dedup_skipped,
        1
    );
    assert_eq!(session.dedup_order.front(), Some(&legacy));

    assert_eq!(
        session
            .feed_admitted_pointcloud(&points, first, 10)
            .unwrap()
            .accepted,
        1
    );
    assert!(!session.dedup_set.contains(&legacy));
    assert_eq!(
        session
            .feed_pointcloud(&points, (0.0, 0.0), 32, 11)
            .unwrap()
            .accepted,
        1
    );
    assert_eq!(session.dedup_set_size(), 2);
    assert_eq!(session.dedup_order.len(), 2);
    assert!(!session.dedup_set.contains(&FrameDedupKey::Admitted(second)));
    assert_eq!(session.generation(), 5);
}

#[test]
fn disabling_history_disables_both_feed_paths_deduplication() {
    let mut session = MappingSession::with_capacity(8).with_dedup_history_capacity(0);
    let points = [point(1.0)];
    let key = admitted(1, 1, 1);
    reset_fingerprint_calls();
    for _ in 0..2 {
        assert_eq!(
            session
                .feed_admitted_pointcloud(&points, key, 100)
                .unwrap()
                .accepted,
            1
        );
        assert_eq!(
            session
                .feed_pointcloud(&points, (0.0, 0.0), 32, 100)
                .unwrap()
                .accepted,
            1
        );
    }
    assert_eq!(session.current_size(), 4);
    assert_eq!(session.generation(), 4);
    assert!(session.dedup_set.is_empty());
    assert!(session.dedup_order.is_empty());
    assert_eq!(fingerprint_calls(), 2);
}

#[test]
fn reset_clears_both_identity_tags_and_preserves_session_configuration() {
    let mut session = MappingSession::with_capacity(2)
        .with_dedup_history_capacity(2)
        .with_stale_threshold_ns(17);
    let points = [point(1.0), point(2.0)];
    let key = admitted(1, 1, 1);
    session.feed_pose(1.0, 2.0, 3.0);
    session.feed_pointcloud(&points, (0.0, 0.0), 32, 100).unwrap();
    session.feed_admitted_pointcloud(&points, key, 101).unwrap();
    assert_eq!(session.dedup_set_size(), 2);
    assert_eq!(session.snapshot_stats().evicted_point_count, 2);
    session.reset_state();
    assert_eq!(
        session.snapshot_stats(),
        SessionStats {
            capacity: 2,
            current_size: 0,
            last_seen_at_ns: 0,
            last_pose: [0.0; 3],
            dedup_set_size: 0,
            generation: 0,
            evicted_point_count: 0,
        }
    );
    assert!(session.dedup_order.is_empty());
    assert_eq!(session.dedup_history_capacity, 2);
    assert_eq!(session.stale_threshold_ns, 17);
    assert_eq!(
        session
            .feed_admitted_pointcloud(&points, key, 101)
            .unwrap()
            .accepted,
        2
    );
    assert_eq!(
        session
            .feed_pointcloud(&points, (0.0, 0.0), 32, 100)
            .unwrap()
            .accepted,
        2
    );
    assert_eq!(session.dedup_set_size(), 2);
}

#[test]
fn stale_admitted_replay_is_rejected_before_deduplication() {
    let mut session = MappingSession::with_capacity(8).with_stale_threshold_ns(10);
    let points = [point(1.0)];
    let key = admitted(1, 1, 1);
    session.feed_admitted_pointcloud(&points, key, 100).unwrap();
    session
        .feed_admitted_pointcloud(&points, admitted(1, 1, 2), 111)
        .unwrap();
    let before = session.snapshot_stats();
    let order = session.dedup_order.clone();
    let window = session.clone_points();
    let err = session.feed_admitted_pointcloud(&points, key, 100).unwrap_err();
    assert!(matches!(
        err,
        SessionError::StaleRevision {
            last_seen_at_ns: 111,
            now_ns: 100,
            threshold_ns: 10,
        }
    ));
    assert_eq!(session.snapshot_stats(), before);
    assert_eq!(session.dedup_order, order);
    assert_eq!(session.clone_points(), window);
}

#[test]
fn stale_inputs_skip_fingerprinting_and_precede_zero_capacity_errors() {
    let mut session = MappingSession::with_capacity(0).with_stale_threshold_ns(10);
    session
        .feed_admitted_pointcloud(&[], admitted(1, 1, 1), 100)
        .unwrap();
    let before = session.snapshot_stats();
    reset_fingerprint_calls();
    assert!(matches!(
        session.feed_pointcloud(&[point(1.0)], (0.0, 0.0), 32, 89),
        Err(SessionError::StaleRevision { .. })
    ));
    assert!(matches!(
        session.feed_admitted_pointcloud(&[point(1.0)], admitted(1, 1, 2), 89),
        Err(SessionError::StaleRevision { .. })
    ));
    assert_eq!(fingerprint_calls(), 0);
    assert_eq!(session.snapshot_stats(), before);
}

#[test]
fn capacity_failure_remembers_no_admitted_key_or_freshness() {
    let mut session = MappingSession::with_capacity(0);
    let before = session.snapshot_stats();
    let key = admitted(1, 1, 1);
    reset_fingerprint_calls();
    for _ in 0..2 {
        assert!(matches!(
            session.feed_admitted_pointcloud(&[point(1.0)], key, 100),
            Err(SessionError::ResourceExhausted {
                capacity: 0,
                current_size: 0,
                attempted_points: 1,
            })
        ));
        assert_eq!(session.snapshot_stats(), before);
        assert!(session.dedup_order.is_empty());
    }
    assert_eq!(fingerprint_calls(), 0);
}

#[test]
fn empty_admitted_feed_preserves_legacy_acceptance_semantics_at_zero_capacity() {
    let mut session = MappingSession::with_capacity(0);
    let key = admitted(1, 1, 1);
    assert_eq!(
        session.feed_admitted_pointcloud(&[], key, 100).unwrap(),
        FeedStats::accepted_with_eviction(0, 0, 0, 0, 100)
    );
    assert_eq!(session.generation(), 0);
    assert_eq!(session.dedup_set_size(), 1);
    assert_eq!(
        session.feed_admitted_pointcloud(&[], key, 101).unwrap(),
        FeedStats::dedup_only(0, 0, 0, 100)
    );
    assert_eq!(session.last_seen_at_ns(), 100);
    assert_eq!(session.dedup_set_size(), 1);
}

#[test]
fn admitted_feeds_retain_rolling_tail_and_do_not_reapply_replay_evictions() {
    let mut session = MappingSession::with_capacity(3);
    let first = [point(0.0), point(1.0)];
    session
        .feed_admitted_pointcloud(&first, admitted(1, 1, 1), 100)
        .unwrap();
    let second = session
        .feed_admitted_pointcloud(&[point(2.0), point(3.0)], admitted(1, 1, 2), 101)
        .unwrap();
    assert_eq!((second.accepted, second.evicted), (2, 1));
    assert_eq!(session.clone_points(), vec![point(1.0), point(2.0), point(3.0)]);
    let oversized = [point(4.0), point(5.0), point(6.0), point(7.0), point(8.0)];
    let third = session
        .feed_admitted_pointcloud(&oversized, admitted(1, 1, 3), 102)
        .unwrap();
    assert_eq!((third.accepted, third.evicted), (3, 5));
    assert_eq!(session.clone_points(), vec![point(6.0), point(7.0), point(8.0)]);
    assert_eq!(session.snapshot_stats().evicted_point_count, 6);
    assert_eq!(session.generation(), 3);
    let before = session.snapshot_stats();
    assert_eq!(
        session
            .feed_admitted_pointcloud(&oversized, admitted(1, 1, 3), 102)
            .unwrap(),
        FeedStats::dedup_only(5, 3, 3, 102)
    );
    // 点预算淘汰与身份历史独立，已离开点窗口的首帧仍然去重。
    assert_eq!(
        session
            .feed_admitted_pointcloud(&first, admitted(1, 1, 1), 100)
            .unwrap(),
        FeedStats::dedup_only(2, 3, 3, 102)
    );
    assert_eq!(session.snapshot_stats(), before);
}

#[test]
fn admitted_feeds_share_existing_freshness_tolerance_and_zero_timestamp_rules() {
    let mut session = MappingSession::with_capacity(8).with_stale_threshold_ns(10);
    let points = [point(1.0)];
    session
        .feed_pointcloud(&points, (0.0, 0.0), 32, 100)
        .unwrap();
    for (sequence, time) in [(1, 90), (2, 0)] {
        assert_eq!(
            session
                .feed_admitted_pointcloud(&points, admitted(1, 1, sequence), time)
                .unwrap()
                .accepted,
            1
        );
        assert_eq!(session.last_seen_at_ns(), 100);
    }
    session
        .feed_admitted_pointcloud(&points, admitted(1, 1, 3), 111)
        .unwrap();
    assert!(matches!(
        session.feed_pointcloud(&points, (0.0, 0.0), 32, 100),
        Err(SessionError::StaleRevision { .. })
    ));
    assert_eq!(session.last_seen_at_ns(), 111);
}

#[test]
fn admitted_path_never_fingerprints_and_each_fresh_legacy_attempt_does_once() {
    let mut session = MappingSession::with_capacity(8).with_stale_threshold_ns(10);
    let points = [point(1.0), point(2.0)];
    let key = admitted(1, 1, 1);
    reset_fingerprint_calls();
    session.feed_admitted_pointcloud(&points, key, 100).unwrap();
    session.feed_admitted_pointcloud(&points, key, 101).unwrap();
    assert_eq!(fingerprint_calls(), 0);
    session.feed_pointcloud(&points, (0.0, 0.0), 32, 100).unwrap();
    assert_eq!(fingerprint_calls(), 1);
    session.feed_pointcloud(&points, (0.0, 0.0), 32, 100).unwrap();
    assert_eq!(fingerprint_calls(), 2);
    assert!(matches!(
        session.feed_pointcloud(&points, (0.0, 0.0), 32, 89),
        Err(SessionError::StaleRevision { .. })
    ));
    assert_eq!(fingerprint_calls(), 2);

    let mut zero_capacity = MappingSession::with_capacity(0);
    assert!(matches!(
        zero_capacity.feed_pointcloud(&points, (0.0, 0.0), 32, 100),
        Err(SessionError::ResourceExhausted { .. })
    ));
    assert_eq!(fingerprint_calls(), 3);
    assert_eq!(zero_capacity.dedup_set_size(), 0);
}

#[test]
fn legacy_same_timestamp_content_pose_step_and_timestamp_identity_is_unchanged() {
    let mut session = MappingSession::with_capacity(16);
    let points = [point(1.0)];
    session.feed_pointcloud(&points, (1.0, 2.0), 32, 100).unwrap();
    assert_eq!(
        session
            .feed_pointcloud(&points, (1.0001, 2.0001), 32, 100)
            .unwrap()
            .dedup_skipped,
        1
    );
    for (input, pose, step, time) in [
        ([Point3::new(9.0, 2.0, 3.0)], (1.0, 2.0), 32, 100),
        ([Point3::new(1.0, 9.0, 3.0)], (1.0, 2.0), 32, 100),
        ([Point3::new(1.0, 2.0, 9.0)], (1.0, 2.0), 32, 100),
        (points, (1.002, 2.0), 32, 100),
        (points, (1.0, 2.002), 32, 100),
        (points, (1.0, 2.0), 48, 100),
        (points, (1.0, 2.0), 32, 101),
    ] {
        let stats = session.feed_pointcloud(&input, pose, step, time).unwrap();
        assert_eq!(stats.accepted, 1);
        assert_eq!(stats.dedup_skipped, 0);
    }
    assert_eq!(session.current_size(), 8);
    assert_eq!(session.generation(), 8);
    assert_eq!(session.dedup_set_size(), 8);
}
