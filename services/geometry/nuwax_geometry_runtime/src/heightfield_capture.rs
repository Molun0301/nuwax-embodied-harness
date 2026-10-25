//! Fixed finite source geometry, never a navigation admission decision.
//! Integer heights are q/8 sub-bins. Masks have at most 81 actual XY columns.
use std::collections::{BTreeMap, BTreeSet};
type Xy = [i32; 2];
type Bin = i64;

fn window(c: Xy) -> impl Iterator<Item = Xy> {
    (-4..=4).flat_map(move |x| {
        (-4..=4).filter_map(move |y| Some([c[0].checked_add(x)?, c[1].checked_add(y)?]))
    })
}
/// 窗口位置编号：`(dx+4)*9 + (dy+4)`，与 `window` 的遍历顺序一致。掩码用这81位表示。
const CENTER: usize = 40;
fn bit_of(c: Xy, xy: Xy) -> usize {
    ((i64::from(xy[0]) - i64::from(c[0]) + 4) * 9 + i64::from(xy[1]) - i64::from(c[1]) + 4) as usize
}
/// 坐标可表示的窗口位置；越出 i32 的位置与 `window` 一样从不出现在掩码里。
fn window_bits(c: Xy) -> u128 {
    window(c).fold(0, |bits, xy| bits | 1 << bit_of(c, xy))
}
/// 按升序给出掩码中的位置编号。
fn bits(mut mask: u128) -> impl Iterator<Item = usize> {
    std::iter::from_fn(move || {
        (mask != 0).then(|| {
            let bit = mask.trailing_zeros() as usize;
            mask &= mask - 1;
            bit
        })
    })
}
/// 窗口内同一行（dy 相同）的全部位置。
const fn row_bits(dy: usize) -> u128 {
    let mut bits = 0;
    let mut x = 0;
    while x < 9 {
        bits |= 1 << (x * 9 + dy);
        x += 1;
    }
    bits
}
/// 3×3 膨胀后仍在窗口且坐标可表示的位置。左右移一位跨行的部分被清除；
/// 第80位左移出的第81位也要先清掉，否则再右移9位会落到第72位。
fn dilate(window: u128, input: u128) -> u128 {
    let row = (input | ((input << 1) & !row_bits(0)) | ((input >> 1) & !row_bits(8))) & ((1 << 81) - 1);
    (row | (row << 9) | (row >> 9)) & window
}
fn edge(upper: u128, lower: u128) -> bool {
    let contains = |set: u128, x: i32, y: i32| {
        (0..9).contains(&x) && (0..9).contains(&y) && (set >> (x * 9 + y)) & 1 != 0
    };
    for d in [[1, 0], [-1, 0], [0, 1], [0, -1]] {
        let tangent = [-d[1], d[0]];
        if bits(upper & lower).any(|p| {
            let p = [(p / 9) as i32, (p % 9) as i32];
            (0..3).all(|n| {
                let a = [p[0] + n * tangent[0], p[1] + n * tangent[1]];
                let b = [a[0] + d[0], a[1] + d[1]];
                contains(upper, a[0], a[1])
                    && contains(lower, a[0], a[1])
                    && contains(lower, b[0], b[1])
                    && !contains(upper, b[0], b[1])
            })
        }) {
            return true;
        }
    }
    false
}
fn vertical(bins: &[Bin]) -> bool {
    let mut first = None;
    let mut previous = None;
    for &z in bins {
        if previous.is_none_or(|p| z - p > 4) {
            first = Some(z);
        }
        if z - first.unwrap() > 32 {
            return true;
        }
        previous = Some(z);
    }
    false
}
fn upright(local: &Local, lower: u128) -> bool {
    let columns = local.vertical;
    if columns == 0 || (columns & lower).count_ones() as usize * 3 < lower.count_ones() as usize * 2 {
        return false;
    }
    let n = columns.count_ones() as f64;
    let offset = |bit: usize| [f64::from(bit as i32 / 9 - 4), f64::from(bit as i32 % 9 - 4)];
    let mean = bits(columns).fold([0.; 2], |m, bit| {
        let p = offset(bit);
        [m[0] + p[0] / n, m[1] + p[1] / n]
    });
    let [xx, xy, yy] = bits(columns).fold([0.; 3], |v, bit| {
        let p = offset(bit);
        let x = p[0] - mean[0];
        let y = p[1] - mean[1];
        [v[0] + x * x / n, v[1] + x * y / n, v[2] + y * y / n]
    });
    let lambda_min = ((xx + yy) - ((xx - yy).powi(2) + 4. * xy * xy).sqrt()) * 0.5;
    lambda_min <= 0.25
}
#[derive(Clone, Copy, Debug)]
struct Shape {
    intersection: usize,
    union: usize,
    edge: bool,
    upright: bool,
}
impl Shape {
    fn joins(self) -> bool {
        self.intersection * 3 >= self.union * 2 && !self.edge && !self.upright
    }
}
fn shape(local: &mut Local, columns: &Columns, base_residual: Bin, residual: Bin) -> Shape {
    let lower = local.mask(columns, base_residual);
    let upper = local.mask(columns, residual);
    compare_masks(local, lower, upper)
}
fn compare_masks(local: &Local, lower: u128, upper: u128) -> Shape {
    let a = dilate(local.window, lower);
    // 相同集合的交并仍为自身，且不存在只属于下层的边缘；无需重复膨胀和查边。
    let (intersection, union, has_edge) = if lower == upper {
        (a.count_ones() as usize, a.count_ones() as usize, false)
    } else {
        let b = dilate(local.window, upper);
        (
            (a & b).count_ones() as usize,
            (a | b).count_ones() as usize,
            edge(upper, lower),
        )
    };
    Shape {
        intersection,
        union,
        edge: has_edge,
        upright: upright(local, lower),
    }
}


use nuwax_map_chunks::heightfield_observations::HeightfieldColumn;
use nuwax_map_chunks::{
    ColumnGroundRecords, HeightfieldSurface, Key, LayerObservation, LayerSample, LayerSource,
    Writer, MAX_LAYER_SAMPLES,
};
use rayon::prelude::*;
use std::sync::{Arc, OnceLock};

/// One original admitted return. The owner shares its already-computed key with
/// occupancy; dynamic is evaluated once, before any classification decisions.
#[derive(Clone, Debug)]
pub(super) struct Return {
    pub point: [f64; 3],
    pub key: Key,
    pub bin: Bin,
    pub index: u32,
    pub dynamic: bool,
}
impl Return {
    pub(super) fn from_key(point: [f64; 3], key: Key, index: u32, dynamic: bool, q: f32) -> Self {
        let scaled = point[2] * (1.0 / f64::from(q));
        let sub = ((scaled - f64::from(key[2])) * 8.0).floor() as i64;
        Self {
            point,
            key,
            bin: i64::from(key[2]) * 8 + sub,
            index,
            dynamic,
        }
    }
}
fn empty_column() -> ColumnGroundRecords {
    ColumnGroundRecords {
        classified_empty: false,
        legacy: vec![],
        heightfields: vec![],
    }
}
fn legacy_height(record: &LayerObservation, xy: Xy, q: f64) -> Option<f64> {
    record.height_at([(f64::from(xy[0]) + 0.5) * q, (f64::from(xy[1]) + 0.5) * q])
}
#[derive(Clone, Copy)]
enum Old<'a> {
    Heightfield(&'a HeightfieldSurface),
    Legacy(Key, &'a LayerObservation),
}
impl Old<'_> {
    fn height(self, xy: Xy, q: f64) -> Option<f64> {
        match self {
            Self::Heightfield(v) => Some(v.column.height()),
            Self::Legacy(_, v) => legacy_height(v, xy, q),
        }
    }
    fn contains(self, z: i32) -> bool {
        match self {
            Self::Heightfield(v) => v.column.members().any(|(key, _)| key == z),
            Self::Legacy(block, v) => {
                z.div_euclid(32) == block[2]
                    && v.members
                        .binary_search(&(v.members[0] % 1024 + 1024 * z.rem_euclid(32) as u16))
                        .is_ok()
            }
        }
    }
    fn members(self) -> Vec<i32> {
        match self {
            Self::Heightfield(v) => v.column.members().map(|(z, _)| z).collect(),
            Self::Legacy(block, v) => v
                .members
                .iter()
                .map(|local| block[2] * 32 + i32::from(local / 1024))
                .collect(),
        }
    }
}
fn old_records(column: &ColumnGroundRecords) -> impl Iterator<Item = Old<'_>> {
    column
        .heightfields
        .iter()
        .map(Old::Heightfield)
        .chain(column.legacy.iter().map(|(b, v)| Old::Legacy(*b, v)))
}
/// 列集合的切比雪夫邻域，以 `(x, y最小, y最大)` 行区间表示。坐标越出 i32 的列
/// 与逐列 `checked_add` 展开一样被丢弃；区间按行排序，同行重叠或相邻者已合并。
fn neighborhood_rows(columns: &BTreeSet<Xy>, radius: i32) -> Vec<(i32, i32, i32)> {
    let r = i64::from(radius);
    let clamp = |v: i64| v.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32;
    let mut rows = Vec::with_capacity(columns.len() * (2 * radius as usize + 1));
    for xy in columns {
        let (low, high) = (clamp(i64::from(xy[1]) - r), clamp(i64::from(xy[1]) + r));
        for x in (i64::from(xy[0]) - r)..=(i64::from(xy[0]) + r) {
            if let Ok(x) = i32::try_from(x) {
                rows.push((x, low, high));
            }
        }
    }
    rows.sort_unstable();
    rows.dedup_by(|next, last| {
        let joined = last.0 == next.0 && i64::from(next.1) <= i64::from(last.2) + 1;
        if joined {
            last.2 = last.2.max(next.2);
        }
        joined
    });
    rows
}
/// 高度场中质量为正的子格，即历史记录在本帧几何中的高度证据。
fn positive(field: &HeightfieldSurface) -> impl Iterator<Item = Bin> + '_ {
    field.column.members().flat_map(|(z, mass)| {
        mass.iter()
            .enumerate()
            .filter(|(_, m)| **m > 0.)
            .map(move |(b, _)| i64::from(z) * 8 + b as i64)
    })
}
/// 本帧（或历史记录）各列的高度子格，按 `(x, y)` 升序排列，下标即列编号。每列子格升序
/// 去重后连续存放；窗口取数按行二分。内容与原按列有序集合一一对应，没有子格的列也保留。
struct Columns<'a> {
    xy: Vec<Xy>,
    starts: Vec<usize>,
    values: Vec<Bin>,
    old: Vec<Option<&'a ColumnGroundRecords>>,
}
impl<'a> Columns<'a> {
    fn with_capacity(count: usize) -> Self {
        let mut starts = Vec::with_capacity(count + 1);
        starts.push(0);
        Self { xy: Vec::with_capacity(count), starts, values: Vec::new(), old: Vec::with_capacity(count) }
    }
    /// 按 `(x, y)` 升序追加一列。
    fn push(&mut self, xy: Xy, values: impl IntoIterator<Item = Bin>, old: Option<&'a ColumnGroundRecords>) {
        let start = self.values.len();
        self.values.extend(values);
        self.values[start..].sort_unstable();
        let mut end = start;
        for i in start..self.values.len() {
            if end == start || self.values[i] != self.values[end - 1] {
                self.values[end] = self.values[i];
                end += 1;
            }
        }
        self.values.truncate(end);
        self.starts.push(end);
        self.xy.push(xy);
        self.old.push(old);
    }
    /// 本帧几何：有回波的列取非动态回波子格，其余读取列取历史高度场的正质量子格。
    fn frame(old: &'a BTreeMap<Xy, ColumnGroundRecords>, current: &BTreeMap<Xy, Vec<&Return>>) -> Self {
        let mut columns = Self::with_capacity(old.len() + current.len());
        let mut olds = old.iter().peekable();
        for (&xy, points) in current {
            while let Some((&key, record)) = olds.next_if(|(key, _)| **key < xy) {
                columns.push(key, record.heightfields.iter().flat_map(positive), Some(record));
            }
            let record = olds.next_if(|(key, _)| **key == xy).map(|(_, record)| record);
            columns.push(xy, points.iter().filter(|p| !p.dynamic).map(|p| p.bin), record);
        }
        for (&key, record) in olds {
            columns.push(key, record.heightfields.iter().flat_map(positive), Some(record));
        }
        columns
    }
    /// 只由历史高度场组成的几何，用于判断旧切片之间是否有真实边缘。
    fn history(old: &'a BTreeMap<Xy, ColumnGroundRecords>) -> Self {
        let mut columns = Self::with_capacity(old.len());
        for (&xy, record) in old {
            columns.push(xy, record.heightfields.iter().flat_map(positive), Some(record));
        }
        columns
    }
    fn bins(&self, index: usize) -> &[Bin] {
        &self.values[self.starts[index]..self.starts[index + 1]]
    }
    /// 窗口内已有列的编号，位置与 `window` 相同。
    fn around(&self, c: Xy) -> [Option<usize>; 81] {
        self.square(c, 4)
    }
    /// 以 `c` 为中心、半径 `r` 的方窗内已有列的编号，位置为 `(dx+r)*(2r+1) + dy+r`，
    /// 半径 4 时即 `window` 的位编号；坐标越出 i32 的位置为空。
    fn square<const N: usize>(&self, c: Xy, r: i32) -> [Option<usize>; N] {
        let mut cells = [None; N];
        let (low, high) = (c[1].saturating_sub(r), c[1].saturating_add(r));
        for dx in -r..=r {
            let Some(x) = c[0].checked_add(dx) else {
                continue;
            };
            let mut index = self.xy.partition_point(|xy| *xy < [x, low]);
            while let Some(&xy) = self.xy.get(index).filter(|xy| xy[0] == x && xy[1] <= high) {
                let dy = i64::from(xy[1]) - i64::from(c[1]);
                cells[(i64::from(dx + r) * i64::from(2 * r + 1) + dy + i64::from(r)) as usize] = Some(index);
                index += 1;
            }
        }
        cells
    }
    /// 与 `bins.range(low..=high).next().is_some()` 相同；先用该列上下界直接判定。
    fn hits(&self, index: usize, low: Bin, high: Bin) -> bool {
        let values = self.bins(index);
        match (values.first(), values.last()) {
            (Some(&min), Some(&max)) if max >= low && min <= high => {
                min >= low || max <= high || values[values.partition_point(|&z| z < low)] <= high
            }
            _ => false,
        }
    }
}
/// 单列种子只取决于该列高度、该列旧记录和窗口内各列高度。只有一个子格时它就是
/// 唯一候选；覆盖度按升序子格与每个邻列做一次归并，计数与逐个区间查询相同。
/// `window` 给出该列 9×9 窗口内已有列的编号（顺序不影响计数），只在需要时取。
fn select_seed<I: IntoIterator<Item = usize>>(
    columns: &Columns,
    index: usize,
    q: f64,
    window: impl FnOnce() -> I,
) -> Option<Bin> {
    let values = columns.bins(index);
    if values.len() <= 1 {
        return values.first().copied();
    }
    let xy = columns.xy[index];
    let mut coverage = vec![0_usize; values.len()];
    let (low, high) = (values[0] - 4, values[values.len() - 1] + 4);
    for n in window() {
        let other = columns.bins(n);
        // 邻列子格全在本列子格 ±4 之外时不贡献覆盖度。
        if other.first().is_none_or(|&first| first > high) || other.last().is_some_and(|&last| last < low) {
            continue;
        }
        let mut j = 0;
        for (k, &z) in values.iter().enumerate() {
            while j < other.len() && other[j] < z - 4 {
                j += 1;
            }
            if j < other.len() && other[j] <= z + 4 {
                coverage[k] += 1;
            }
        }
    }
    values
        .iter()
        .copied()
        .zip(coverage)
        .map(|(z, coverage)| {
            // 有关联旧记录优先；距离取这些记录中最近的高度，没有高度时为无穷远。
            let mut referenced = false;
            let mut distance: Option<f64> = None;
            for v in columns.old[index].into_iter().flat_map(old_records) {
                if !v.contains(z.div_euclid(8) as i32) {
                    continue;
                }
                referenced = true;
                if let Some(h) = v.height(xy, q) {
                    let d = ((z as f64 + 0.5) * q / 8.0 - h).abs();
                    distance = Some(distance.filter(|best| best.total_cmp(&d).is_le()).unwrap_or(d));
                }
            }
            (z, !referenced, std::cmp::Reverse(coverage), distance.unwrap_or(f64::INFINITY))
        })
        .min_by(|a, b| {
            a.1.cmp(&b.1)
                .then(a.2.cmp(&b.2))
                .then(a.3.total_cmp(&b.3))
                .then(a.0.cmp(&b.0))
        })
        .map(|value| value.0)
}
/// 单帧内按列记忆种子、邻列参考高度和竖直判定。它们只在当前列窗口内被读取，
/// 首次用到时由任一线程计算；每项都是本帧固定输入的纯函数，与计算先后无关，不跨帧保留。
struct Context<'a> {
    columns: Columns<'a>,
    q: f64,
    seeds: Vec<OnceLock<Option<Bin>>>,
    heights: Vec<OnceLock<Option<f64>>>,
    vertical: Vec<OnceLock<bool>>,
}
impl<'a> Context<'a> {
    fn new(columns: Columns<'a>, q: f64) -> Self {
        let count = columns.xy.len();
        Self {
            columns,
            q,
            seeds: (0..count).map(|_| OnceLock::new()).collect(),
            heights: (0..count).map(|_| OnceLock::new()).collect(),
            vertical: (0..count).map(|_| OnceLock::new()).collect(),
        }
    }
    fn seed(&self, index: usize) -> Option<Bin> {
        *self.seeds[index].get_or_init(|| {
            let xy = self.columns.xy[index];
            select_seed(&self.columns, index, self.q, || self.columns.around(xy).into_iter().flatten())
        })
    }
    fn vertical(&self, index: usize) -> bool {
        *self.vertical[index].get_or_init(|| vertical(self.columns.bins(index)))
    }
    /// 邻列只在唯一关联（或唯一）旧记录给出高度时作参考。
    fn reference_height(&self, index: usize) -> Option<f64> {
        *self.heights[index].get_or_init(|| self.compute_reference_height(index))
    }
    fn compute_reference_height(&self, index: usize) -> Option<f64> {
        let n = self.columns.xy[index];
        match self.columns.old[index] {
            Some(records) => {
                let seed = self.seed(index);
                // 只需记录数、关联数和各自第一条，不收集临时向量。
                let (mut all, mut first) = (0, None);
                let (mut associated, mut first_associated) = (0, None);
                for r in old_records(records) {
                    all += 1;
                    first = first.or(Some(r));
                    if seed.is_some_and(|z| r.contains(z.div_euclid(8) as i32)) {
                        associated += 1;
                        first_associated = first_associated.or(Some(r));
                    }
                }
                let reference = if associated == 1 {
                    first_associated
                } else if associated == 0 && all == 1 {
                    first
                } else {
                    None
                };
                reference.and_then(|r| r.height(n, self.q))
            }
            None => None,
        }
    }
    fn local(&self, c: Xy) -> Local {
        let cells = self.columns.around(c);
        let mut seeds = [None; 81];
        let mut vertical = 0;
        // 邻列的 9×9 窗口都落在以 c 为中心的 17×17 方窗内，需要时取一次，免去逐列二分。
        let mut wide: Option<[Option<usize>; 289]> = None;
        for (bit, cell) in cells.iter().enumerate() {
            if let Some(index) = *cell {
                seeds[bit] = *self.seeds[index].get_or_init(|| {
                    select_seed(&self.columns, index, self.q, || {
                        let wide = wide.get_or_insert_with(|| self.columns.square(c, 8));
                        let (x, y) = (bit / 9, bit % 9);
                        (0..9)
                            .flat_map(move |i| (0..9).map(move |j| (x + i) * 17 + y + j))
                            .filter_map(|position| wide[position])
                    })
                });
                if self.vertical(index) {
                    vertical |= 1 << bit;
                }
            }
        }
        Local {
            c,
            cells,
            seeds,
            vertical,
            window: window_bits(c),
            masks: Vec::new(),
            references: None,
        }
    }
}
/// 以一列为中心的窗口：邻列编号、种子、竖直位置，以及本列内按残差记忆的掩码。
struct Local {
    c: Xy,
    cells: [Option<usize>; 81],
    seeds: [Option<Bin>; 81],
    vertical: u128,
    window: u128,
    masks: Vec<(Bin, u128)>,
    references: Option<[Option<f64>; 81]>,
}
impl Local {
    fn mask(&mut self, columns: &Columns, residual: Bin) -> u128 {
        if let Some(&(_, mask)) = self.masks.iter().find(|(r, _)| *r == residual) {
            return mask;
        }
        let mut mask = 0;
        for bit in 0..81 {
            if let (Some(index), Some(seed)) = (self.cells[bit], self.seeds[bit]) {
                if columns.hits(index, seed + residual - 2, seed + residual + 2) {
                    mask |= 1 << bit;
                }
            }
        }
        self.masks.push((residual, mask));
        mask
    }
}
#[derive(Debug)]
struct Group {
    seed: Bin,
    bins: BTreeSet<Bin>,
}
fn groups(local: &mut Local, columns: &Columns, old: Option<&ColumnGroundRecords>, q: f64) -> Vec<Group> {
    let xy = local.c;
    let (Some(index), Some(first)) = (local.cells[CENTER], local.seeds[CENTER]) else {
        return vec![];
    };
    let lower = local.mask(columns, 0);
    if upright(local, lower) {
        return vec![];
    }
    let mut remaining: BTreeSet<Bin> = columns.bins(index).iter().copied().collect();
    let mut result = Vec::<Group>::new();
    let mut next = Some(first);
    while let Some(seed) = next {
        let base = seed - first;
        let mut group = Group {
            seed,
            bins: [seed].into(),
        };
        remaining.remove(&seed);
        let mut candidates: Vec<_> = remaining.iter().copied().collect();
        candidates.sort_by_key(|z| ((z - seed).abs(), *z));
        for z in candidates {
            let low = (*group.bins.first().unwrap()).min(z);
            let high = (*group.bins.last().unwrap()).max(z);
            if high - low <= 32 && shape(local, columns, base, z - first).joins() {
                group.bins.insert(z);
                remaining.remove(&z);
            }
        }
        result.push(group);
        next = remaining
            .iter()
            .copied()
            .filter(|z| {
                result
                    .iter()
                    .flat_map(|g| &g.bins)
                    .all(|old| (z - old).abs() > 32)
                    || (old.is_some_and(|records| {
                        old_records(records).any(|r| {
                            r.contains(z.div_euclid(8) as i32)
                                && !r.contains(first.div_euclid(8) as i32)
                        })
                    }) && (shape(local, columns, 0, z - first).edge
                        || old.is_some_and(|records| {
                            old_records(records)
                                .filter(|r| r.contains(z.div_euclid(8) as i32))
                                .any(|upper| {
                                    old_records(records)
                                        .filter(|r| r.contains(first.div_euclid(8) as i32))
                                        .any(|lower| {
                                            upper
                                                .height(xy, q)
                                                .zip(lower.height(xy, q))
                                                .is_some_and(|(a, b)| (a - b).abs() > 4.0 * q)
                                        })
                                })
                        })))
            })
            .min_by_key(|z| ((z - first).abs(), *z));
    }
    result
}

/// CR-L1: old geometry is only a reference coordinate, never new bin mass.
/// A missing/ambiguous reference is not evidence of a changed surface.
fn legacy_shape(local: &mut Local, context: &Context, candidate: Bin, central: Old<'_>) -> Option<Shape> {
    let (xy, q) = (local.c, context.q);
    let centre = central.height(xy, q)?;
    let delta = (candidate as f64 + 0.5) * q / 8.0 - centre;
    // 邻列参考与正在比较的旧记录无关，本列内只取一次；中心位置每次用该记录高度。
    let references = match local.references {
        Some(references) => references,
        None => {
            let mut references = [None; 81];
            for (bit, cell) in local.cells.iter().enumerate() {
                if let Some(index) = *cell {
                    if bit != CENTER {
                        references[bit] = context.reference_height(index);
                    }
                }
            }
            *local.references.insert(references)
        }
    };
    let mut refs = [None; 81];
    for (bit, cell) in local.cells.iter().enumerate() {
        if cell.is_none() {
            continue;
        }
        refs[bit] = if bit == CENTER { Some(centre) } else { references[bit] };
    }
    let keys = refs
        .iter()
        .enumerate()
        .filter(|(_, height)| height.is_some())
        .fold(0_u128, |keys, (bit, _)| keys | 1 << bit);
    if keys == 0 {
        return None;
    }
    let lower = local.mask(&context.columns, 0) & keys;
    let upper = bits(keys)
        .filter(|&bit| {
            let h = refs[bit].unwrap();
            // 偏差随子格单调不减，满足 |偏差| <= q/4 的子格连续；
            // 取第一个偏差不低于 -q/4 的子格判断即可，与逐个比较结果相同。
            let offset = |z: Bin| ((z as f64 + 0.5) * q / 8.0 - h) - delta;
            let values = context.columns.bins(local.cells[bit].unwrap());
            values
                .get(values.partition_point(|&z| offset(z) < -q / 4.0))
                .is_some_and(|&z| offset(z).abs() <= q / 4.0)
        })
        .fold(0_u128, |upper, bit| upper | 1 << bit);
    Some(compare_masks(local, lower, upper))
}

// Existing independent surfaces cannot be joined by a frame that sees only a
// middle return. Near-band independence needs an actual original slice edge;
// neither two record names nor a diagnostic sample is such evidence.
fn independent_old_pair<'a>(
    local: &mut Local,
    columns: &Columns,
    a: Old<'_>,
    b: Old<'_>,
    old: &'a BTreeMap<Xy, ColumnGroundRecords>,
    historical: &OnceLock<Context<'a>>,
    q: f64,
) -> bool {
    let xy = local.c;
    if a.height(xy, q)
        .zip(b.height(xy, q))
        .is_some_and(|(x, y)| (x - y).abs() > 4.0 * q)
    {
        return true;
    }
    let slice_edge = |left: &[Bin], right: &[Bin], local: &mut Local, columns: &Columns| {
        let Some(first) = local.seeds[CENTER] else {
            return false;
        };
        let masks: BTreeMap<_, _> = left
            .iter()
            .chain(right)
            .copied()
            .collect::<BTreeSet<_>>()
            .into_iter()
            .map(|z| (z, local.mask(columns, z - first)))
            .collect();
        left.iter().any(|&x| {
            right
                .iter()
                .any(|&y| x != y && edge(masks[&x.max(y)], masks[&x.min(y)]))
        })
    };
    // Current evidence is usable only if both actual old member sets were seen.
    if let Some(index) = local.cells[CENTER] {
        let current = columns.bins(index);
        let left: Vec<_> = current
            .iter()
            .copied()
            .filter(|z| a.contains(z.div_euclid(8) as i32))
            .collect();
        let right: Vec<_> = current
            .iter()
            .copied()
            .filter(|z| b.contains(z.div_euclid(8) as i32))
            .collect();
        if slice_edge(&left, &right, local, columns) {
            return true;
        }
    }
    let (Old::Heightfield(a), Old::Heightfield(b)) = (a, b) else {
        return false;
    };
    let left: Vec<_> = positive(a).collect();
    let right: Vec<_> = positive(b).collect();
    if left.is_empty() || right.is_empty() {
        return false;
    }
    let prior = historical.get_or_init(|| Context::new(Columns::history(old), q));
    let mut prior_local = prior.local(xy);
    slice_edge(&left, &right, &mut prior_local, &prior.columns)
}

/// Capture one already-admitted frame. Source identity/replay admission belongs
/// to the original owner; this function neither invents identity nor skips IO.
pub(super) fn capture(
    writer: &mut Writer,
    returns: &[Return],
    removed: &[Key],
    sequence: u64,
    timestamp_ns: u64,
    source: Arc<LayerSource>,
) -> Result<Vec<u32>, String> {
    let (updates, indices) = classify(writer, returns, removed, sequence, timestamp_ns, source)?;
    if !updates.is_empty() {
        writer.replace_ground_columns(updates.iter().map(|(xy, value)| (*xy, value)))?;
    }
    Ok(indices)
}
/// 本帧分类：返回待替换的列记录和归入地面的点索引，写入由 `capture` 完成。
fn classify(
    writer: &mut Writer,
    returns: &[Return],
    removed: &[Key],
    sequence: u64,
    timestamp_ns: u64,
    source: Arc<LayerSource>,
) -> Result<(BTreeMap<Xy, ColumnGroundRecords>, Vec<u32>), String> {
    let q = f64::from(writer.voxel_size_m());
    let touched: BTreeSet<_> = returns
        .iter()
        .map(|p| [p.key[0], p.key[1]])
        .chain(removed.iter().map(|k| [k[0], k[1]]))
        .collect();
    let mut current: BTreeMap<Xy, Vec<&Return>> = BTreeMap::new();
    for p in returns {
        current.entry([p.key[0], p.key[1]]).or_default().push(p);
    }
    // 原写入集为本帧触及列外扩4列、读取集再外扩8列；切比雪夫邻域半径可相加，
    // 读取集即触及列外扩12列。按行区间交给Writer，不再逐列展开成集合。诊断样本
    // 只在有回波的列合并时使用，其余列只取几何。
    let old = writer.ground_columns_in_rows(&neighborhood_rows(&touched, 4 + 8), |xy| current.contains_key(&xy))?;
    let context = Context::new(Columns::frame(&old, &current), q);
    let historical = OnceLock::new();
    let voxel = writer.voxel_size_m();
    // 原实现按升序遍历写入集，并跳过没有本帧回波的列；本帧回波列都在写入集内。
    // 各列只读本帧固定输入，按列并行计算后按升序合并；遇错返回列序上第一个错误，
    // 与串行遍历遇错即停的结果相同。
    let columns: Vec<_> = current.iter().collect();
    let frame = Frame { context: &context, old: &old, historical: &historical, q, voxel, source: &source, sequence, timestamp_ns };
    let results: Vec<_> = columns.par_iter().map(|&(&xy, points)| frame.column(xy, points)).collect();
    let mut updates = BTreeMap::new();
    let mut indices = Vec::new();
    for (&(&xy, _), result) in columns.iter().zip(results) {
        let (next, column_indices) = result?;
        indices.extend(column_indices);
        if let Some(next) = next {
            updates.insert(xy, next);
        }
    }
    indices.sort_unstable();
    indices.dedup();
    Ok((updates, indices))
}
/// 一帧分类的只读输入，供各列并行使用。
struct Frame<'f, 'a> {
    context: &'f Context<'a>,
    old: &'a BTreeMap<Xy, ColumnGroundRecords>,
    historical: &'f OnceLock<Context<'a>>,
    q: f64,
    voxel: f32,
    source: &'f Arc<LayerSource>,
    sequence: u64,
    timestamp_ns: u64,
}
impl Frame<'_, '_> {
    /// 一个本帧回波列：返回变化后的列记录（无变化为 `None`）和归入地面的点索引。
    fn column(&self, xy: Xy, points: &[&Return]) -> Result<(Option<ColumnGroundRecords>, Vec<u32>), String> {
        let Self { context, old, historical, q, voxel, source, sequence, timestamp_ns } = *self;
        let mut indices = Vec::new();
        // Absence of a return is not counter-evidence. Original Writer::remove
        // already retracts removed occupancy. Halo participates in geometry only.
        let mut local = context.local(xy);
        let grouping = groups(&mut local, &context.columns, old.get(&xy), q);
        let mut voxels: BTreeMap<i32, Vec<&Return>> = BTreeMap::new();
        for p in points {
            voxels.entry(p.key[2]).or_default().push(*p);
        }
        let mut safe: BTreeMap<i32, usize> = BTreeMap::new();
        let mut denied = BTreeSet::new();
        for (&z, points) in &voxels {
            let selected = |p: &&Return| {
                if p.dynamic {
                    None
                } else {
                    grouping.iter().position(|g| g.bins.contains(&p.bin))
                }
            };
            let first = selected(&points[0]);
            if let Some(g) = first.filter(|_| points.iter().all(|p| selected(p) == first)) {
                safe.insert(z, g);
            } else {
                denied.insert(z);
            }
        }
        let previous = old.get(&xy);
        let olds: Vec<_> = previous.into_iter().flat_map(old_records).collect();
        // 两轮关联读取同一列、本帧不可变上下文，复用判断而不复用观察或样本。
        let mut compatibility = BTreeMap::new();
        let mut compatible = |index: usize, seed: Bin, local: &mut Local, context: &Context| {
            *compatibility.entry((index, seed)).or_insert_with(|| {
                let record = olds[index];
                record
                    .height(xy, q)
                    .is_some_and(|h| ((seed as f64 + 0.5) * q / 8.0 - h).abs() <= 4.0 * q)
                    && legacy_shape(local, context, seed, record).is_none_or(Shape::joins)
            })
        };
        let mut withheld = BTreeSet::new();
        for (g, group) in grouping
            .iter()
            .enumerate()
            .filter(|(g, _)| safe.values().any(|v| v == g))
        {
            let candidates: Vec<_> = olds
                .iter()
                .copied()
                .enumerate()
                .filter(|(index, _)| compatible(*index, group.seed, &mut local, context))
                .map(|(_, record)| record)
                .collect();
            if candidates.iter().enumerate().any(|(i, &a)| {
                candidates[i + 1..].iter().any(|&b| {
                    independent_old_pair(&mut local, &context.columns, a, b, old, historical, q)
                })
            }) {
                withheld.insert(g);
            }
        }
        // Ambiguous middle returns are still occupancy. They are not a reason
        // to retract an unseen old member, so never add these keys to denied.
        safe.retain(|_, g| !withheld.contains(g));
        let mut frames = BTreeMap::new();
        for (g, group) in grouping.iter().enumerate() {
            let mut counts: BTreeMap<i32, [u32; 8]> = BTreeMap::new();
            let mut samples = Vec::new();
            for p in points.iter().filter(|p| safe.get(&p.key[2]) == Some(&g)) {
                counts.entry(p.key[2]).or_default()[p.bin.rem_euclid(8) as usize] += 1;
                indices.push(p.index);
                samples.push(LayerSample {
                    source: source.clone(),
                    point: p.point,
                    source_sequence: sequence,
                    source_timestamp_ns: timestamp_ns,
                    point_index: p.index,
                });
            }
            if let Some(frame) = HeightfieldColumn::from_counts(xy, voxel, counts)?
            {
                frames.insert(g, (group, frame, samples));
            }
        }
        let mut next = empty_column();
        let mut assignments: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        let mut survivors = BTreeMap::new();
        for (i, record) in olds.iter().copied().enumerate() {
            let keep: BTreeSet<_> = record
                .members()
                .into_iter()
                .filter(|z| !denied.contains(z))
                .collect();
            let matching: Vec<_> = frames
                .iter()
                .filter(|(_, (g, _, _))| compatible(i, g.seed, &mut local, context))
                .map(|(&g, _)| g)
                .collect();
            if matching.len() == 1 {
                let group = matching[0];
                let keep: BTreeSet<_> = keep
                    .into_iter()
                    .filter(|z| safe.get(z).is_none_or(|g| *g == group))
                    .collect();
                survivors.insert(i, keep);
                assignments.entry(group).or_default().push(i);
            } else {
                // A currently classified voxel has one owner. Ambiguous old
                // records keep only unrelated members, never duplicate claims.
                let keep = keep.into_iter().filter(|z| !safe.contains_key(z)).collect();
                retain_old(&mut next, record, &keep, q);
            }
        }
        for (g, (_, mut frame, mut samples)) in frames {
            let assigned = assignments.remove(&g).unwrap_or_default();
            let mut prior = Vec::new();
            let mut carry = BTreeSet::new();
            for i in assigned {
                let keep = &survivors[&i];
                carry.extend(keep);
                match olds[i] {
                    Old::Heightfield(value) => {
                        if let Some(column) = value.column.retain_members(keep) {
                            prior.push(column);
                        }
                        samples.extend(
                            value
                                .samples
                                .iter()
                                .filter(|s| sample_member(s, q).is_some_and(|z| keep.contains(&z)))
                                .cloned(),
                        );
                    }
                    Old::Legacy(_, value) => samples.extend(
                        value
                            .samples
                            .iter()
                            .filter(|s| sample_member(s, q).is_some_and(|z| keep.contains(&z)))
                            .cloned(),
                    ),
                }
            }
            // Two independently normalized old fields do not encode their
            // relative historical weight. Only one actual prior may EWMA.
            if prior.len() == 1 {
                frame = prior.pop().unwrap().merge_frame(&frame)?;
            }
            frame.add_zero_mass_members(carry);
            retain_samples(&mut samples, source);
            next.heightfields.push(HeightfieldSurface {
                column: frame,
                samples,
            });
        }
        // This actual XY was classified, even when mixed/dynamic returns leave
        // no ground declaration. Preserve that distinction beside occupancy.
        next.classified_empty = next.legacy.is_empty() && next.heightfields.is_empty();
        Ok((previous.is_none_or(|before| before != &next).then_some(next), indices))
    }
}
fn sample_member(sample: &LayerSample, q: f64) -> Option<i32> {
    super::voxel_key(sample.point, q as f32).map(|key| key.2)
}
fn retain_old(out: &mut ColumnGroundRecords, old: Old<'_>, keep: &BTreeSet<i32>, q: f64) {
    match old {
        Old::Heightfield(value) => {
            if let Some(column) = value.column.retain_members(keep) {
                out.heightfields.push(HeightfieldSurface {
                    column,
                    samples: value
                        .samples
                        .iter()
                        .filter(|s| sample_member(s, q).is_some_and(|z| keep.contains(&z)))
                        .cloned()
                        .collect(),
                });
            }
        }
        Old::Legacy(block, value) => {
            let mut value = value.clone();
            value
                .members
                .retain(|local| keep.contains(&(block[2] * 32 + i32::from(*local / 1024))));
            // Claim withdrawal is not occupancy deletion. Keep real historical
            // diagnostics; BL4 proves newer samples against their actual voxels.
            if !value.members.is_empty() {
                out.legacy.push((block, value));
            }
        }
    }
}
fn retain_samples(samples: &mut Vec<LayerSample>, current: &LayerSource) {
    samples.sort_by(|a, b| {
        (
            &a.source,
            a.source_timestamp_ns,
            a.source_sequence,
            a.point_index,
        )
            .cmp(&(
                &b.source,
                b.source_timestamp_ns,
                b.source_sequence,
                b.point_index,
            ))
    });
    samples.dedup_by(|a, b| {
        a.source == b.source
            && a.source_timestamp_ns == b.source_timestamp_ns
            && a.source_sequence == b.source_sequence
            && a.point_index == b.point_index
    });
    if samples.len() <= MAX_LAYER_SAMPLES {
        return;
    }
    let mut groups = Vec::new();
    let mut start = 0;
    while start < samples.len() {
        let end = start
            + samples[start..]
                .iter()
                .take_while(|s| s.source == samples[start].source)
                .count();
        groups.push((start, end));
        start = end;
    }
    if let Some(i) = groups
        .iter()
        .position(|(start, _)| samples[*start].source.as_ref() == current)
    {
        groups.rotate_left(i);
    }
    let mut selected = vec![false; samples.len()];
    let mut count = 0;
    while count < MAX_LAYER_SAMPLES {
        for (start, end) in &mut groups {
            if *end > *start {
                *end -= 1;
                selected[*end] = true;
                count += 1;
                if count == MAX_LAYER_SAMPLES {
                    break;
                }
            }
        }
    }
    let mut i = 0;
    samples.retain(|_| {
        let keep = selected[i];
        i += 1;
        keep
    });
}

#[cfg(test)]
#[path = "heightfield_capture_tests.rs"]
mod tests;
