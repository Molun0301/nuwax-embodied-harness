
use super::*;
#[test]
fn append_and_swap_remove_touch_only_affected_pages() {
    let mut pages = PagedVoxelSnapshot::default();
    for i in 0..5000 {
        pages.inserted((i, 0, 0), 10000);
    }
    pages.committed();
    pages.removed((100, 0, 0), 10000);
    assert_eq!(pages.keys()[100], (4999, 0, 0));
    assert_eq!(pages.dirty_pages().collect::<Vec<_>>(), vec![0, 4]);
    pages.inserted((8000, 0, 0), 10000);
    assert_eq!(pages.keys().len(), 5000);
    assert_eq!(pages.dirty_pages().collect::<Vec<_>>(), vec![0, 4]);
}
#[test]
fn incremental_slots_match_membership_without_historical_reordering() {
    let mut pages = PagedVoxelSnapshot::default();
    let mut voxels = HashSet::new();
    for round in 0..2000 {
        let key = ((round * 31) % 701, 0, 0);
        if round % 3 == 0 {
            if voxels.remove(&key) {
                pages.removed(key, 10000);
            }
        } else if voxels.insert(key) {
            pages.inserted(key, 10000);
        }
        assert!(!pages.prepare(&voxels));
        assert_eq!(pages.keys().iter().copied().collect::<HashSet<_>>(), voxels);
        assert_eq!(pages.keys.len(), pages.positions.len());
        pages.committed();
    }
    pages.invalidate();
    assert!(pages.prepare(&voxels));
    assert!(!pages.prepare(&voxels));
    assert_eq!(pages.keys().iter().copied().collect::<HashSet<_>>(), voxels);
}
