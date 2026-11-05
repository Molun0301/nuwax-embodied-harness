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
#[path = "voxel_index_tests.rs"]
mod tests;
