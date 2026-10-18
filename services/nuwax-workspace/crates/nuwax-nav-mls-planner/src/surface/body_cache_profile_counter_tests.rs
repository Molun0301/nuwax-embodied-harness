// Test-only counter wiring. Capacity fixtures populate the original private
// maps directly; these tests do not perform or time 8193 geometry queries.
use super::*;
use crate::body_query_profile::{self as profile, Counter, Record};

fn capture<T>(query: impl FnOnce() -> T) -> (T, Record) {
    struct Cleanup(bool);
    impl Drop for Cleanup {
        fn drop(&mut self) {
            if self.0 { let _ = profile::end(); }
        }
    }
    profile::begin();
    let mut cleanup = Cleanup(true);
    let result = query();
    let record = profile::end();
    cleanup.0 = false;
    (result, record)
}

fn pose_events(record: &Record) -> [u64; 3] {
    [Counter::PoseCacheHits, Counter::PoseCacheMisses, Counter::PoseCacheCapacityClears]
        .map(|counter| record.counters[counter as usize])
}

fn motion_events(record: &Record) -> [u64; 4] {
    [Counter::MotionCacheHits, Counter::MotionCacheMisses,
        Counter::MotionCacheCapacityClears, Counter::MotionCacheBoundaryBypasses]
        .map(|counter| record.counters[counter as usize])
}

fn pose_key(pose: ([f64; 3], [f64; 4])) -> [u64; 7] {
    [pose.0[0], pose.0[1], pose.0[2], pose.1[0], pose.1[1], pose.1[2], pose.1[3]]
        .map(f64::to_bits)
}

fn motion_key(from: ([f64; 3], [f64; 4]), to: ([f64; 3], [f64; 4])) -> [u64; 14] {
    [from.0[0], from.0[1], from.0[2], from.1[0], from.1[1], from.1[2], from.1[3],
        to.0[0], to.0[1], to.0[2], to.1[0], to.1[1], to.1[2], to.1[3]]
        .map(f64::to_bits)
}

#[test]
fn pose_success_and_collision_count_real_misses_and_hits() {
    let pose = ([0.05, 0.05, 0.5], [0.0, 0.0, 0.0, 1.0]);
    for (map, collision) in [
        (scene(&[], false).fork_for_planning(), false),
        (scene(&[(0.55, 0.05, 0.55)], false).fork_for_planning(), true),
    ] {
        assert!(map.body_query_cache.lock().unwrap().poses.is_empty());
        let mut witness = None;
        let (cold, cold_record) = capture(||
            map.validate_body_pose_with_witness(pose.0, pose.1, &mut witness));
        let cold_witness = witness;
        witness = None; // Require the hit to restore, not merely retain, the witness.
        let (warm, warm_record) = capture(||
            map.validate_body_pose_with_witness(pose.0, pose.1, &mut witness));
        assert_eq!(cold, if collision { Err(SurfaceError::InsufficientClearance) } else { Ok(()) });
        assert_eq!(warm, cold);
        assert_eq!(witness, cold_witness, "cache hit retains the original rejection");
        assert_eq!(witness.is_some(), collision);
        assert_eq!(pose_events(&cold_record), [0, 1, 0]);
        assert_eq!(pose_events(&warm_record), [1, 0, 0]);
        assert_eq!(motion_events(&cold_record), [0; 4]);
        assert_eq!(motion_events(&warm_record), [0; 4]);
        assert_eq!(map.body_query_cache.lock().unwrap().poses.len(), 1);
    }
}

#[test]
fn motion_success_and_interior_collision_count_real_misses_and_hits() {
    let q = [0.0, 0.0, 0.0, 1.0];
    for (map, from, to, collision) in [
        (scene(&[], false).fork_for_planning(), ([0.05, 0.05, 0.5], q), ([0.25, 0.05, 0.5], q), false),
        (ground_band_sweep_scene(0.525).fork_for_planning(),
            ([-0.25, 0.0295, 0.375], q), ([0.06, 0.3395, 0.375], q), true),
    ] {
        assert!(map.body_query_cache.lock().unwrap().motions.is_empty());
        let mut witness = None;
        let (cold, cold_record) = capture(||
            map.validate_body_motion_with_witness(from, to, &[], &mut witness));
        let cold_witness = witness;
        witness = None; // Require the hit to restore, not merely retain, the witness.
        let (warm, warm_record) = capture(||
            map.validate_body_motion_with_witness(from, to, &[], &mut witness));
        assert_eq!(cold, if collision { Err(SurfaceError::InsufficientClearance) } else { Ok(()) });
        assert_eq!(warm, cold);
        assert_eq!(witness, cold_witness, "motion hit retains the first rejected sample");
        assert_eq!(witness.is_some(), collision);
        if let Some(witness) = witness {
            assert_ne!(witness.pose, from);
            assert_ne!(witness.pose, to);
        }
        assert_eq!(motion_events(&cold_record), [0, 1, 0, 0]);
        assert_eq!(motion_events(&warm_record), [1, 0, 0, 0]);
        assert_eq!(pose_events(&warm_record), [0; 3], "motion hit bypasses the checked sweep");
        assert_eq!(map.body_query_cache.lock().unwrap().motions.len(), 1);
    }
}

#[test]
fn nonempty_boundary_bypasses_even_a_warm_motion_without_counting_a_miss() {
    let map = scene(&[], false).fork_for_planning();
    let from = ([0.05, 0.05, 0.5], [0.0, 0.0, 0.0, 1.0]);
    let to = ([0.25, 0.05, 0.5], from.1);
    map.validate_body_motion(from, to, &[]).unwrap();
    let outside = [[-1.0, -1.0], [-0.5, -1.0], [-0.5, -0.5], [-1.0, -0.5]];
    let inside = [[-2.0, -2.0], [2.0, -2.0], [2.0, 2.0], [-2.0, 2.0]];
    for (boundary, expected) in [
        (outside, Err(SurfaceError::InvalidBoundary)), (inside, Ok(())),
    ] {
        let mut witness = None;
        let (result, record) = capture(||
            map.validate_body_motion_with_witness(from, to, &boundary, &mut witness));
        assert_eq!(result, expected);
        assert!(witness.is_none());
        assert_eq!(motion_events(&record), [0, 0, 0, 1]);
        // Endpoint/body checks still run; their pose-cache events are independent.
        assert_eq!(map.body_query_cache.lock().unwrap().motions.len(), 1);
    }
}

#[test]
fn pose_capacity_clear_counter_follows_the_original_miss_only_branch() {
    let seed = ([0.05, 0.05, 0.5], [0.0, 0.0, 0.0, 1.0]);
    let next = ([0.25, 0.05, 0.5], seed.1);
    for size in [8191_usize, 8192] {
        let map = scene(&[], false).fork_for_planning();
        map.validate_body_pose(seed.0, seed.1).unwrap();
        {
            let mut cache = map.body_query_cache.lock().unwrap();
            assert_eq!(cache.poses.len(), 1);
            // Never queried: deliberately non-pose keys. Only the table length
            // is synthetic, not the seed hit or following real geometry miss.
            for i in 1..size {
                let mut key = [u64::MAX; 7]; key[0] = i as u64;
                assert!(cache.poses.insert(key, CachedBodyResult::Ok(())).is_none());
            }
            assert_eq!(cache.poses.len(), size);
            assert!(!cache.poses.contains_key(&pose_key(next)));
        }
        let (hit, hit_record) = capture(|| map.validate_body_pose(seed.0, seed.1));
        assert_eq!(hit, Ok(()));
        assert_eq!(pose_events(&hit_record), [1, 0, 0]);
        assert_eq!(map.body_query_cache.lock().unwrap().poses.len(), size);
        let mut witness = None;
        let (miss, miss_record) = capture(||
            map.validate_body_pose_with_witness(next.0, next.1, &mut witness));
        assert_eq!(miss, Ok(()));
        assert!(witness.is_none());
        assert_eq!(pose_events(&miss_record), [0, 1, u64::from(size == 8192)]);
        assert_eq!(motion_events(&miss_record), [0; 4]);
        let cache = map.body_query_cache.lock().unwrap();
        assert_eq!(cache.poses.len(), if size == 8192 { 1 } else { 8192 });
        assert!(cache.poses.contains_key(&pose_key(next)));
        assert_eq!(cache.poses.contains_key(&pose_key(seed)), size != 8192);
    }
}

#[test]
fn motion_capacity_clear_counter_follows_the_original_miss_only_branch() {
    let from = ([0.05, 0.05, 0.5], [0.0, 0.0, 0.0, 1.0]);
    let to = ([0.25, 0.05, 0.5], from.1);
    for size in [8191_usize, 8192] {
        let map = scene(&[], false).fork_for_planning();
        map.validate_body_motion(from, to, &[]).unwrap();
        {
            let mut cache = map.body_query_cache.lock().unwrap();
            assert_eq!(cache.motions.len(), 1);
            // No geometry is replayed to fill this test-only capacity fixture.
            for i in 1..size {
                let mut key = [u64::MAX; 14]; key[0] = i as u64;
                assert!(cache.motions.insert(key, CachedBodyResult::Ok(())).is_none());
            }
            assert_eq!(cache.motions.len(), size);
            assert!(!cache.motions.contains_key(&motion_key(to, from)));
        }
        let (hit, hit_record) = capture(|| map.validate_body_motion(from, to, &[]));
        assert_eq!(hit, Ok(()));
        assert_eq!(motion_events(&hit_record), [1, 0, 0, 0]);
        assert_eq!(pose_events(&hit_record), [0; 3]);
        assert_eq!(map.body_query_cache.lock().unwrap().motions.len(), size);
        let mut witness = None;
        let (miss, miss_record) = capture(||
            map.validate_body_motion_with_witness(to, from, &[], &mut witness));
        assert_eq!(miss, Ok(()));
        assert!(witness.is_none());
        assert_eq!(motion_events(&miss_record), [0, 1, u64::from(size == 8192), 0]);
        let cache = map.body_query_cache.lock().unwrap();
        assert_eq!(cache.motions.len(), if size == 8192 { 1 } else { 8192 });
        assert!(cache.motions.contains_key(&motion_key(to, from)));
        assert_eq!(cache.motions.contains_key(&motion_key(from, to)), size != 8192);
    }
}
