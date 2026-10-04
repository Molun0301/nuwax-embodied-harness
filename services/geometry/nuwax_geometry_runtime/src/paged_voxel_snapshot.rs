//! Dense storage addresses with bounded dirty pages. Addresses are not semantic
//! point identities: a removal moves the final occupant into the vacated slot.
use ahash::{AHashMap as HashMap, AHashSet as HashSet};
use std::collections::BTreeSet;
type VoxelKey = (i32, i32, i32);
pub(super) const CHUNK_POINTS: usize = 1024;

#[derive(Default)]
pub(super) struct PagedVoxelSnapshot {
    keys: Vec<VoxelKey>,
    positions: HashMap<VoxelKey, usize>,
    dirty: BTreeSet<usize>,
    rebuild: bool,
}
impl PagedVoxelSnapshot {
    pub(super) fn inserted(&mut self, key: VoxelKey, _budget: usize) {
        if self.rebuild || self.positions.contains_key(&key) { return; }
        let index = self.keys.len();
        self.positions.insert(key, index);
        self.keys.push(key);
        self.dirty.insert(index / CHUNK_POINTS);
    }
    pub(super) fn removed(&mut self, key: VoxelKey, _budget: usize) {
        if self.rebuild { return; }
        let Some(index) = self.positions.remove(&key) else { return; };
        let last = self.keys.len() - 1;
        self.keys.swap_remove(index);
        if index < self.keys.len() { self.positions.insert(self.keys[index], index); }
        self.dirty.insert(index / CHUNK_POINTS);
        self.dirty.insert(last / CHUNK_POINTS);
    }
    pub(super) fn invalidate(&mut self) {
        self.keys.clear();
        self.positions.clear();
        self.dirty.clear();
        self.rebuild = true;
    }
    /// A global rewrite changes every point identity/layout; only this path scans
    /// the whole source set. Ordinary additions/deletions never re-sort it.
    pub(super) fn needs_rebuild(&self) -> bool { self.rebuild }
    pub(super) fn prepare(&mut self, voxels: &HashSet<VoxelKey>) -> bool {
        if !self.rebuild { return false; }
        self.keys.extend(voxels.iter().copied());
        self.keys.sort_unstable();
        self.positions.extend(self.keys.iter().enumerate().map(|(i, key)| (*key, i)));
        self.dirty.extend(0..self.keys.len().div_ceil(CHUNK_POINTS));
        self.rebuild = false;
        true
    }
    pub(super) fn keys(&self) -> &[VoxelKey] { &self.keys }
    pub(super) fn dirty_pages(&self) -> impl Iterator<Item=usize> + '_ { self.dirty.iter().copied() }
    pub(super) fn committed(&mut self) { self.dirty.clear(); }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn append_and_swap_remove_touch_only_affected_pages() {
        let mut pages = PagedVoxelSnapshot::default();
        for i in 0..5000 { pages.inserted((i, 0, 0), 10000); }
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
            if round % 3 == 0 { if voxels.remove(&key) { pages.removed(key, 10000); } }
            else if voxels.insert(key) { pages.inserted(key, 10000); }
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
}
