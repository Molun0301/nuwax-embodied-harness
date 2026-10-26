//! 复用空间块到归档页的内存目录；DUFO达到原关联预算时合并父块，始终保留保守覆盖。
use ahash::{AHashMap, AHashSet};

type Blocks = AHashMap<[u32; 3], AHashSet<usize>>;

pub(super) fn allocation(capacity: usize, item: usize) -> usize {
    if capacity == 0 { return 0; }
    let buckets = (capacity * 8 / 7).next_power_of_two().max(4);
    buckets * (item + 1) + 16
}

pub(super) struct BlockDirectory {
    blocks: Option<Blocks>,
    entries: usize,
    set_bytes: usize,
    compact: bool,
    level: u32,
    pub(super) last_compactions: usize,
    pub(super) last_compaction_entries: usize,
    pub(super) last_compaction_peak_bytes: usize,
}

impl BlockDirectory {
    pub(super) fn new(compact: bool, voxel_size: f32) -> Self {
        let level = if compact {
            (f64::from(voxel_size) / super::INDEX_BLOCK_M).log2().ceil().clamp(0.0, 32.0) as u32
        } else { 0 };
        Self { blocks: Some(Blocks::new()), entries: 0, set_bytes: 0, compact, level,
            last_compactions: 0, last_compaction_entries: 0, last_compaction_peak_bytes: 0 }
    }

    pub(super) fn begin_append(&mut self) {
        self.last_compactions = 0;
        self.last_compaction_entries = 0;
        self.last_compaction_peak_bytes = 0;
    }

    pub(super) fn key(&self, raw: super::Key) -> [u32; 3] {
        if self.level == 32 { return [0; 3]; }
        [raw.0, raw.1, raw.2].map(|value| ((value as u32) ^ 0x8000_0000) >> self.level)
    }

    pub(super) fn get(&self, key: [u32; 3]) -> Option<&AHashSet<usize>> { self.blocks.as_ref()?.get(&key) }
    pub(super) fn missing(&self) -> bool { self.blocks.is_none() }
    pub(super) fn compacting(&self) -> bool { self.compact }
    pub(super) fn entries(&self) -> usize { self.entries }
    pub(super) fn level(&self) -> u32 { self.level }

    pub(super) fn insert(&mut self, raw: super::Key, page: usize) {
        loop {
            let key = self.key(raw);
            let Some(blocks) = &mut self.blocks else { return; };
            if blocks.get(&key).is_some_and(|pages| pages.contains(&page)) { return; }
            // 在增加关联之前压缩；不扩大原1M预算，也不丢掉目录转为全页扫描。
            if self.compact && self.entries == super::MAX_INDEX_ENTRIES {
                self.coarsen();
                continue;
            }
            let pages = blocks.entry(key).or_default();
            let before = allocation(pages.capacity(), std::mem::size_of::<usize>());
            pages.insert(page);
            self.set_bytes += allocation(pages.capacity(), std::mem::size_of::<usize>()) - before;
            self.entries += 1;
            if !self.compact && self.entries > super::MAX_INDEX_ENTRIES {
                // 旧后端沿用原预算耗尽行为；DUFO不会进入此分支。
                self.blocks = None;
                self.set_bytes = 0;
            }
            return;
        }
    }

    fn coarsen(&mut self) {
        assert!(self.compact && self.level < 32, "完整点云目录应在原归档页数上限内收敛");
        let old_bytes = self.memory_bytes();
        let old = self.blocks.take().expect("DUFO空间目录始终存在");
        let mut next = Blocks::new();
        let mut entries = 0;
        let mut set_bytes = 0;
        self.last_compactions += 1;
        self.last_compaction_entries += self.entries;
        for (key, pages) in old {
            let set = next.entry(key.map(|value| value >> 1)).or_insert_with(AHashSet::new);
            let before_capacity = allocation(set.capacity(), std::mem::size_of::<usize>());
            let before_length = set.len();
            set.extend(pages);
            entries += set.len() - before_length;
            set_bytes += allocation(set.capacity(), std::mem::size_of::<usize>()) - before_capacity;
        }
        self.blocks = Some(next);
        self.entries = entries;
        self.set_bytes = set_bytes;
        self.level += 1;
        // 迁移中旧/新容器可短时并存；保守按完整两份容量计，不把这部分算为常驻。
        self.last_compaction_peak_bytes = self.last_compaction_peak_bytes.max(old_bytes + self.memory_bytes());
    }

    pub(super) fn memory_bytes(&self) -> usize {
        self.blocks.as_ref().map_or(0, |blocks|
            allocation(blocks.capacity(), std::mem::size_of::<([u32; 3], AHashSet<usize>)>()) + self.set_bytes)
    }

    #[cfg(test)]
    pub(super) fn disable(&mut self) { self.blocks = None; self.set_bytes = 0; }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coarsening_keeps_signed_boundaries_and_converges_to_one_root() {
        let mut directory = BlockDirectory::new(true, 0.05);
        let samples = [((i32::MIN, -1, 0), 0), ((-1, 0, 1), 1), ((0, -1, -1), 2), ((i32::MAX, 1, -1), 3)];
        for (raw, page) in samples { directory.insert(raw, page); }
        for level in 0..=32 {
            assert_eq!(directory.level(), level);
            for (raw, page) in samples {
                assert!(directory.get(directory.key(raw)).unwrap().contains(&page), "层级{level}不得漏掉符号边界关联");
            }
            let low = directory.key((-1, -1, -1));
            let high = directory.key((0, 0, 0));
            assert!((0..3).all(|axis| low[axis] <= high[axis]));
            if level < 32 { directory.coarsen(); }
        }
        assert_eq!(directory.get([0; 3]).unwrap().len(), 4);
        assert_eq!(directory.entries(), 4);
    }
}
