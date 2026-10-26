//! 固定三层、每层64路的持久页表；修改只复制路径，并逐层维护可见点数与精确边界。
use super::Page;
use std::{ops::Index, sync::Arc};

const FANOUT: usize = 64;
const ROOT_LEVEL: u32 = 2;
const CAPACITY: usize = FANOUT * FANOUT * FANOUT;

#[derive(Clone)]
struct Summary {
    count: u64,
    low: [f32; 3],
    high: [f32; 3],
}

impl Default for Summary {
    fn default() -> Self {
        Self { count: 0, low: [f32::INFINITY; 3], high: [f32::NEG_INFINITY; 3] }
    }
}

impl Summary {
    fn include(&mut self, count: u64, low: [f32; 3], high: [f32; 3]) {
        if count == 0 { return; }
        self.count += count;
        for axis in 0..3 {
            self.low[axis] = self.low[axis].min(low[axis]);
            self.high[axis] = self.high[axis].max(high[axis]);
        }
    }
}

#[derive(Clone)]
enum Children {
    Nodes([Option<Arc<Node>>; FANOUT]),
    Pages([Option<Arc<Page>>; FANOUT]),
}

#[derive(Clone)]
struct Node {
    children: Children,
    summary: Summary,
}

impl Node {
    fn empty(level: u32) -> Self {
        Self {
            children: if level == 0 { Children::Pages(std::array::from_fn(|_| None)) }
                else { Children::Nodes(std::array::from_fn(|_| None)) },
            summary: Summary::default(),
        }
    }

    fn get(&self, index: usize, level: u32) -> Option<&Arc<Page>> {
        match &self.children {
            Children::Pages(pages) => pages[index % FANOUT].as_ref(),
            Children::Nodes(nodes) => nodes[index / FANOUT.pow(level) % FANOUT]
                .as_ref()?.get(index, level - 1),
        }
    }

    fn set(&mut self, index: usize, level: u32, page: Arc<Page>) {
        let mut summary = Summary::default();
        match &mut self.children {
            Children::Pages(pages) => {
                pages[index % FANOUT] = Some(page);
                for page in pages.iter().flatten() { summary.include(page.count, page.low, page.high); }
            }
            Children::Nodes(nodes) => {
                let node = nodes[index / FANOUT.pow(level) % FANOUT]
                    .get_or_insert_with(|| Arc::new(Self::empty(level - 1)));
                Arc::make_mut(node).set(index, level - 1, page);
                for node in nodes.iter().flatten() {
                    summary.include(node.summary.count, node.summary.low, node.summary.high);
                }
            }
        }
        self.summary = summary;
    }

    fn locate(&self, mut offset: u64, level: u32, base: usize) -> Option<(usize, u64)> {
        match &self.children {
            Children::Pages(pages) => {
                for (index, page) in pages.iter().enumerate() {
                    let Some(page) = page else { continue; };
                    if offset < page.count { return Some((base + index, offset)); }
                    offset -= page.count;
                }
            }
            Children::Nodes(nodes) => {
                for (index, node) in nodes.iter().enumerate() {
                    let Some(node) = node else { continue; };
                    if offset < node.summary.count {
                        return node.locate(offset, level - 1, base + index * FANOUT.pow(level));
                    }
                    offset -= node.summary.count;
                }
            }
        }
        None
    }

}

#[derive(Clone)]
pub(super) struct PageTable {
    root: Arc<Node>,
    length: usize,
}

impl Default for PageTable {
    fn default() -> Self { Self { root: Arc::new(Node::empty(ROOT_LEVEL)), length: 0 } }
}

impl PageTable {
    pub(super) fn len(&self) -> usize { self.length }
    pub(super) fn get(&self, index: usize) -> Option<&Arc<Page>> {
        (index < self.length).then(|| self.root.get(index, ROOT_LEVEL)).flatten()
    }
    pub(super) fn set(&mut self, index: usize, page: Arc<Page>) {
        assert!(index < self.length, "完整点云页号越界");
        Arc::make_mut(&mut self.root).set(index, ROOT_LEVEL, page);
    }
    pub(super) fn push(&mut self, page: Arc<Page>) {
        assert!(self.length < CAPACITY, "完整点云页表容量小于已限定的归档上限");
        Arc::make_mut(&mut self.root).set(self.length, ROOT_LEVEL, page);
        self.length += 1;
    }
    pub(super) fn iter(&self) -> impl Iterator<Item = &Arc<Page>> {
        (0..self.length).map(|index| &self[index])
    }
    pub(super) fn bounds(&self) -> ([f32; 3], [f32; 3]) { (self.root.summary.low, self.root.summary.high) }
    pub(super) fn locate(&self, offset: u64) -> Option<(usize, u64)> { self.root.locate(offset, ROOT_LEVEL, 0) }
    #[cfg(test)]
    pub(super) fn memory_bytes(&self) -> usize {
        // 目录只随连续页号追加、从不删除；节点数可直接由长度求出，统计不能另扫整张页表。
        (1 + self.length.div_ceil(FANOUT * FANOUT) + self.length.div_ceil(FANOUT))
            * (std::mem::size_of::<Node>() + 2 * std::mem::size_of::<usize>())
    }
    #[cfg(test)]
    pub(super) fn maximum_memory_bytes() -> usize {
        let pages = super::MAX_ARCHIVE_BYTES.div_ceil((super::PAGE_POINTS * 12) as u64) as usize;
        (1 + pages.div_ceil(FANOUT * FANOUT) + pages.div_ceil(FANOUT))
            * (std::mem::size_of::<Node>() + 2 * std::mem::size_of::<usize>())
    }
}

impl Index<usize> for PageTable {
    type Output = Arc<Page>;
    fn index(&self, index: usize) -> &Self::Output { self.get(index).expect("完整点云页号越界") }
}
