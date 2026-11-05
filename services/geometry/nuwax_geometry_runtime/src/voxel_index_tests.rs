
use super::*;
#[test]
fn index_tracks_insert_remove_negative_edges_and_bounded_fallback() {
    let mut index = VoxelIndex::new();
    for key in [(-1, 0, 0), (-2, 0, 0), (i32::MIN, 0, 0), (i32::MAX, 0, 0)] {
        assert!(index.insert(key));
    }
    assert!(!index.insert((-1, 0, 0)));
    index.remove(&(-1, 0, 0));
    assert!(index.coarse_contains(&(-1, 0, 0)));
    index.remove(&(-2, 0, 0));
    assert!(!index.coarse_contains(&(-1, 0, 0)));
    let restored = VoxelIndex::from(index.points.clone());
    assert_eq!(restored, index);
    index.clear();
    index.index_limit = 2;
    for x in [0, 8, 16] {
        index.insert((x, 0, 0));
    }
    assert!(!index.coarse_enabled());
    assert_eq!(index.len(), 3);
    assert!(index.coarse_contains(&(100, 100, 100)));
    index.clear();
    assert!(index.coarse_enabled());
}
#[test]
fn coarse_membership_matches_independent_fine_set_after_mutations() {
    let mut index = VoxelIndex::new();
    let mut reference = HashSet::new();
    let mut seed = 123_u64;
    for round in 0..2000 {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        let key = (
            (seed as i32).rem_euclid(65) - 32,
            ((seed >> 16) as i32).rem_euclid(65) - 32,
            ((seed >> 32) as i32).rem_euclid(65) - 32,
        );
        if round % 3 == 0 {
            assert_eq!(index.remove(&key), reference.remove(&key));
        } else {
            assert_eq!(index.insert(key), reference.insert(key));
        }
        for block in [coarse_key(key), (0, 0, 0), (-1, -1, -1)] {
            assert_eq!(
                index.coarse_contains(&block),
                reference.iter().any(|key| coarse_key(*key) == block)
            );
        }
        assert_eq!(&*index, &reference);
    }
}
