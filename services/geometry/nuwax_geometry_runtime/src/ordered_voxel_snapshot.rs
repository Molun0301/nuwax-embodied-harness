//! 为规范快照维护确定性字典序；新增点排序后线性合并，删除按当前集合核对。

use std::collections::HashSet;

type VoxelKey = (i32, i32, i32);

#[derive(Default)]
pub(super) struct OrderedVoxelSnapshot {
    keys: Vec<VoxelKey>,
    pending: Vec<VoxelKey>,
    rebuild: bool,
    removed: Vec<VoxelKey>,
}

impl OrderedVoxelSnapshot {
    pub(super) fn inserted(&mut self, key: VoxelKey, budget: usize) {
        if self.rebuild {
            return;
        }
        if self.pending.len() >= budget {
            // 多次增删而长期未提交时，待合并队列仍受点预算约束。
            self.invalidate();
        } else {
            self.pending.push(key);
        }
    }

    pub(super) fn removed(&mut self, key: VoxelKey, budget: usize) {
        if self.rebuild {
            return;
        }
        if self.removed.len() >= budget {
            self.invalidate();
        } else {
            self.removed.push(key);
        }
    }

    pub(super) fn invalidate(&mut self) {
        self.keys.clear();
        self.pending.clear();
        self.rebuild = true;
        self.removed.clear();
    }

    pub(super) fn ordered(&mut self, voxels: &HashSet<VoxelKey>) -> &[VoxelKey] {
        if self.rebuild || self.pending.len() + self.removed.len() > voxels.len() / 4 {
            self.keys.clear();
            self.keys.extend(voxels.iter().copied());
            self.keys.sort_unstable();
        } else {
            if !self.removed.is_empty() {
                // 只查本轮变动的键；历史百万点按序扫过，避免随机查询整个哈希表。
                self.removed.retain(|key| !voxels.contains(key));
                self.removed.sort_unstable();
                self.removed.dedup();
                let mut index = 0;
                self.keys.retain(|key| {
                    while index < self.removed.len() && self.removed[index] < *key {
                        index += 1;
                    }
                    self.removed.get(index) != Some(key)
                });
                self.pending.retain(|key| voxels.contains(key));
            }
            self.pending.sort_unstable();
            self.pending.dedup();
            // 从尾部原地合并，不再每次分配另一份全图排序数组。
            let mut left = self.keys.len();
            let mut right = self.pending.len();
            let total = left + right;
            self.keys.resize(total, (0, 0, 0));
            let mut write = total;
            while right > 0 {
                let next = if left == 0 || self.pending[right - 1] > self.keys[left - 1] {
                    right -= 1;
                    self.pending[right]
                } else if self.pending[right - 1] == self.keys[left - 1] {
                    // 同一提交前删除后重新加入的体素不能重复输出。
                    right -= 1;
                    left -= 1;
                    self.keys[left]
                } else {
                    left -= 1;
                    self.keys[left]
                };
                write -= 1;
                self.keys[write] = next;
            }
            if write != left {
                self.keys.copy_within(0..left, write - left);
                let start = write - left;
                self.keys.copy_within(start..total, 0);
                self.keys.truncate(total - start);
            }
        }
        self.pending.clear();
        self.rebuild = false;
        self.removed.clear();
        &self.keys
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn incremental_order_matches_full_sort_after_insert_delete_and_rebuild() {
        let mut ordered = OrderedVoxelSnapshot::default();
        let mut voxels = HashSet::new();
        for round in 0..80_i32 {
            for i in 0..257_i32 {
                let key = ((i * 13 + round * 31) % 601 - 300, i % 7, round % 3);
                if voxels.insert(key) {
                    ordered.inserted(key, 20000);
                }
            }
            if round % 3 == 0 {
                for &key in &voxels {
                    if (key.0 + round) % 11 == 0 {
                        ordered.removed(key, 20000);
                    }
                }
                voxels.retain(|key| (key.0 + round) % 11 != 0);
            }
            if round % 13 == 0 {
                ordered.invalidate();
            }
            let mut expected = voxels.iter().copied().collect::<Vec<_>>();
            expected.sort_unstable();
            assert_eq!(ordered.ordered(&voxels), expected);
            assert_eq!(ordered.ordered(&voxels), expected);
        }
    }

    #[test]
    fn reinsertion_and_pending_budget_preserve_exact_membership() {
        let mut ordered = OrderedVoxelSnapshot::default();
        let mut voxels = (0..100).map(|i| (i, 0, 0)).collect::<HashSet<_>>();
        ordered.invalidate();
        ordered.ordered(&voxels);
        for i in [1, 8, 50, 99] {
            voxels.remove(&(i, 0, 0));
            ordered.removed((i, 0, 0), 100);
            voxels.insert((i, 0, 0));
            ordered.inserted((i, 0, 0), 100);
        }
        assert_eq!(ordered.ordered(&voxels).len(), 100);
        for _ in 0..200 {
            ordered.removed((1, 0, 0), 100);
            ordered.inserted((1, 0, 0), 100);
        }
        assert!(ordered.pending.len() <= 100);
        assert_eq!(
            ordered.ordered(&voxels),
            (0..100).map(|i| (i, 0, 0)).collect::<Vec<_>>()
        );
        voxels.clear();
        ordered.invalidate();
        assert!(ordered.ordered(&voxels).is_empty());
    }
}
