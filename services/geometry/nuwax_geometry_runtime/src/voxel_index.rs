//! 精确体素集合及有界粗块索引；粗块只用于证明不存在数据，不参与地图量化。
use super::VoxelKey;
use ahash::{AHashMap as HashMap, AHashSet as HashSet};
use std::ops::Deref;

const MAX_COARSE_BLOCKS: usize = 262_144;

pub(super) fn coarse_key(key: VoxelKey) -> VoxelKey {
    (
        key.0.div_euclid(8),
        key.1.div_euclid(8),
        key.2.div_euclid(8),
    )
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct VoxelIndex {
    points: HashSet<VoxelKey>,
    coarse: Option<HashMap<VoxelKey, u16>>,
    index_limit: usize,
}

impl Default for VoxelIndex {
    fn default() -> Self {
        Self::with_capacity(0)
    }
}

impl VoxelIndex {
    pub(super) fn new() -> Self {
        Self::default()
    }
    pub(super) fn with_capacity(capacity: usize) -> Self {
        Self {
            points: HashSet::with_capacity(capacity),
            coarse: Some(HashMap::new()),
            index_limit: MAX_COARSE_BLOCKS,
        }
    }
    fn add_coarse(coarse: &mut Option<HashMap<VoxelKey, u16>>, key: VoxelKey, limit: usize) {
        if let Some(index) = coarse.as_mut() {
            // 一个粗块最多含512个不同细格，重复插入不会进入此处。
            *index.entry(coarse_key(key)).or_insert(0) += 1;
            if index.len() > limit {
                *coarse = None;
            }
        }
    }
    pub(super) fn insert(&mut self, key: VoxelKey) -> bool {
        if !self.points.insert(key) {
            return false;
        }
        Self::add_coarse(&mut self.coarse, key, self.index_limit);
        true
    }
    pub(super) fn remove(&mut self, key: &VoxelKey) -> bool {
        if !self.points.remove(key) {
            return false;
        }
        if let Some(index) = self.coarse.as_mut() {
            let block = coarse_key(*key);
            let count = index.get_mut(&block).expect("派生粗块索引缺少已存体素");
            *count -= 1;
            if *count == 0 {
                index.remove(&block);
            }
        }
        true
    }
    pub(super) fn clear(&mut self) {
        self.points.clear();
        if let Some(index) = self.coarse.as_mut() {
            index.clear();
        } else {
            self.coarse = Some(HashMap::new());
        }
    }
    pub(super) fn coarse_enabled(&self) -> bool {
        self.coarse.is_some()
    }
    pub(super) fn coarse_contains(&self, block: &VoxelKey) -> bool {
        self.coarse
            .as_ref()
            .is_none_or(|index| index.contains_key(block))
    }
}

impl Deref for VoxelIndex {
    type Target = HashSet<VoxelKey>;
    fn deref(&self) -> &Self::Target {
        &self.points
    }
}
impl From<HashSet<VoxelKey>> for VoxelIndex {
    fn from(points: HashSet<VoxelKey>) -> Self {
        let mut result = Self {
            points,
            ..Self::default()
        };
        for &key in &result.points {
            Self::add_coarse(&mut result.coarse, key, result.index_limit);
        }
        result
    }
}
impl FromIterator<VoxelKey> for VoxelIndex {
    fn from_iter<T: IntoIterator<Item = VoxelKey>>(iter: T) -> Self {
        Self::from(iter.into_iter().collect::<HashSet<_>>())
    }
}
impl<const N: usize> From<[VoxelKey; N]> for VoxelIndex {
    fn from(keys: [VoxelKey; N]) -> Self {
        keys.into_iter().collect()
    }
}
impl Extend<VoxelKey> for VoxelIndex {
    fn extend<T: IntoIterator<Item = VoxelKey>>(&mut self, iter: T) {
        for key in iter {
            self.insert(key);
        }
    }
}
impl IntoIterator for VoxelIndex {
    type Item = VoxelKey;
    type IntoIter = std::collections::hash_set::IntoIter<VoxelKey>;
    fn into_iter(self) -> Self::IntoIter {
        self.points.into_iter()
    }
}

#[cfg(test)]
mod tests {
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
}
