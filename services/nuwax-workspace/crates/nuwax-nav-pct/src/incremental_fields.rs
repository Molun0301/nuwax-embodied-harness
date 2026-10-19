//! 沿真实旧依赖撤销，再由有根边界修复；不按固定几何半径截断距离。
//!
//! terrain_upper 只证明哪些边不可能写入。最终 terrain 在旧/新可能写入边
//! 的弱连通闭包内，按冷建相同的稳定几何种子键和原邻居顺序重演 FIFO。
use crate::cancellation::checkpoint;
use crate::incremental::{IdRemap, IncrementalStats};
use crate::Tomogram;
use ahash::AHashSet;
use std::cmp::{Ordering, Reverse};
use std::collections::{BinaryHeap, VecDeque};

#[derive(Clone, Copy, Debug)]
struct Entry {
    value: f64,
    id: usize,
}
impl PartialEq for Entry {
    fn eq(&self, other: &Self) -> bool {
        self.value.to_bits() == other.value.to_bits() && self.id == other.id
    }
}
impl Eq for Entry {}
impl PartialOrd for Entry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Entry {
    fn cmp(&self, other: &Self) -> Ordering {
        self.value
            .total_cmp(&other.value)
            .then_with(|| self.id.cmp(&other.id))
    }
}

#[derive(Clone, Copy)]
enum Field {
    Wall,
    Missing,
    Upper,
}
impl Field {
    fn get(self, map: &Tomogram, id: usize) -> f64 {
        match self {
            Self::Wall => map.cells[id].clearance,
            Self::Missing => map.cells[id].support_clearance,
            Self::Upper => map.cells[id].terrain_upper,
        }
    }
    fn set(self, map: &mut Tomogram, id: usize, value: f64) {
        match self {
            Self::Wall => map.cells[id].clearance = value,
            Self::Missing => map.cells[id].support_clearance = value,
            Self::Upper => map.cells[id].terrain_upper = value,
        }
    }
    fn follows(self, map: &Tomogram, from: usize, to: usize) -> bool {
        let a = self.get(map, from);
        let b = self.get(map, to);
        match self {
            Self::Wall | Self::Missing => {
                a.is_finite() && b.is_finite() && (a + map.resolution).to_bits() == b.to_bits()
            }
            Self::Upper => {
                let candidate = a - decay(map);
                candidate > 0.0 && candidate.to_bits() == b.to_bits()
            }
        }
    }
}

fn decay(map: &Tomogram) -> f64 {
    2.0 * map.resolution / map.clearance_radius.max(map.resolution)
}
fn terrain_base(map: &Tomogram, id: usize) -> f64 {
    let cell = &map.cells[id];
    cell.neighbors
        .iter()
        .map(|&next| (map.cells[next].ground - cell.ground).abs() / map.resolution)
        .fold(0.0, f64::max)
        .min(2.0)
}

/// outdegree 至多八，但同一邻列可以有多个层指向本点，不能限制 indegree。
fn predecessors(map: &Tomogram, id: usize) -> Result<Vec<usize>, String> {
    let [x, y] = map.cells[id].column;
    let mut result = Vec::new();
    for dx in -1..=1 {
        for dy in -1..=1 {
            checkpoint()?;
            let (Some(x), Some(y)) = (x.checked_add(dx), y.checked_add(dy)) else {
                continue;
            };
            for &candidate in map.columns.get(&[x, y]).into_iter().flatten() {
                checkpoint()?;
                if map.cells[candidate].neighbors.contains(&id) {
                    result.push(candidate);
                }
            }
        }
    }
    Ok(result)
}

fn distance_seed(map: &Tomogram, id: usize, field: Field) -> Result<bool, String> {
    let [x, y] = map.cells[id].column;
    for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
        checkpoint()?;
        let column = [x + dx, y + dy];
        let known = map.known_columns.contains(&column);
        match field {
            Field::Missing if !known => return Ok(true),
            Field::Wall if known && map.neighbor_in(id, column, map.resolution).is_none() => {
                return Ok(true);
            }
            _ => {}
        }
    }
    Ok(false)
}

/// 保守地撤销所有旧 tight 后继。先清空整个锥再修复，故即使浮点加减形成
/// 等值环，也不会由无根旧标签互相维持。这里只遍历真实可达的旧依赖。
fn invalidated(
    old: &Tomogram,
    remap: &IdRemap,
    dirty: &AHashSet<usize>,
    field: Field,
) -> Result<AHashSet<usize>, String> {
    let mut result = dirty.clone();
    let mut seen = AHashSet::new();
    let mut queue = VecDeque::new();
    for &id in dirty {
        checkpoint()?;
        if let Some(previous) = remap.new_to_old(id) {
            if seen.insert(previous) {
                queue.push_back(previous);
            }
        }
    }
    while let Some(id) = queue.pop_front() {
        checkpoint()?;
        if let Some(current) = remap.old_to_new(id) {
            result.insert(current);
        }
        for &next in &old.cells[id].neighbors {
            if field.follows(old, id, next) && seen.insert(next) {
                queue.push_back(next);
            }
        }
    }
    Ok(result)
}

fn relax_distance(
    map: &mut Tomogram,
    field: Field,
    id: usize,
    value: f64,
    heap: &mut BinaryHeap<Reverse<Entry>>,
    touched: &mut AHashSet<usize>,
    stats: &mut IncrementalStats,
) {
    if value < field.get(map, id) {
        field.set(map, id, value);
        heap.push(Reverse(Entry { value, id }));
        touched.insert(id);
        stats.distance_relaxed += 1;
    }
}

fn update_distance(
    old: &Tomogram,
    next: &mut Tomogram,
    remap: &IdRemap,
    dirty: &AHashSet<usize>,
    field: Field,
    stats: &mut IncrementalStats,
) -> Result<AHashSet<usize>, String> {
    let invalid = invalidated(old, remap, dirty, field)?;
    let mut touched = invalid.clone();
    // 必须完成全部失效后，才能从存活边界读取有限值。
    for &id in &invalid {
        checkpoint()?;
        field.set(next, id, f64::INFINITY);
    }
    stats.distance_invalidated += invalid.len();
    let mut heap = BinaryHeap::new();
    for &id in &invalid {
        checkpoint()?;
        if distance_seed(next, id, field)? {
            let value = next.resolution * 0.5;
            relax_distance(next, field, id, value, &mut heap, &mut touched, stats);
        }
        for previous in predecessors(next, id)? {
            if invalid.contains(&previous) {
                continue;
            }
            let value = field.get(next, previous) + next.resolution;
            relax_distance(next, field, id, value, &mut heap, &mut touched, stats);
        }
    }
    while let Some(Reverse(entry)) = heap.pop() {
        checkpoint()?;
        if entry.value.to_bits() != field.get(next, entry.id).to_bits() {
            continue;
        }
        // 保留逐边加法，不能改成 hop*q 或对角 sqrt(2)*q。
        let value = entry.value + next.resolution;
        for id in next.cells[entry.id].neighbors.clone() {
            relax_distance(next, field, id, value, &mut heap, &mut touched, stats);
        }
    }
    Ok(touched)
}

fn relax_upper(
    map: &mut Tomogram,
    id: usize,
    value: f64,
    heap: &mut BinaryHeap<Entry>,
    touched: &mut AHashSet<usize>,
) -> bool {
    if value > map.cells[id].terrain_upper {
        map.cells[id].terrain_upper = value;
        heap.push(Entry { value, id });
        touched.insert(id);
        true
    } else {
        false
    }
}

fn propagate_upper(
    map: &mut Tomogram,
    heap: &mut BinaryHeap<Entry>,
    touched: &mut AHashSet<usize>,
) -> Result<usize, String> {
    let mut relaxed = 0;
    let decay = decay(map);
    while let Some(entry) = heap.pop() {
        checkpoint()?;
        if entry.value.to_bits() != map.cells[entry.id].terrain_upper.to_bits() {
            continue;
        }
        let value = entry.value - decay;
        if value <= 0.0 {
            continue;
        }
        for next in map.cells[entry.id].neighbors.clone() {
            if relax_upper(map, next, value, heap, touched) {
                relaxed += 1;
            }
        }
    }
    Ok(relaxed)
}

/// 仅用于首建，与已有完整 compute_costs 同次完成；更新路径不调用此全图入口。
pub(crate) fn initialize_upper(map: &mut Tomogram) -> Result<(), String> {
    checkpoint()?;
    let mut heap = BinaryHeap::new();
    for id in 0..map.cells.len() {
        checkpoint()?;
        let value = terrain_base(map, id);
        map.cells[id].terrain_upper = value;
        if value > 0.0 {
            heap.push(Entry { value, id });
        }
    }
    propagate_upper(map, &mut heap, &mut AHashSet::new())?;
    checkpoint()
}

fn update_upper(
    old: &Tomogram,
    next: &mut Tomogram,
    remap: &IdRemap,
    dirty: &AHashSet<usize>,
    stats: &mut IncrementalStats,
) -> Result<AHashSet<usize>, String> {
    let invalid = invalidated(old, remap, dirty, Field::Upper)?;
    let mut touched = invalid.clone();
    let mut heap = BinaryHeap::new();
    for &id in &invalid {
        checkpoint()?;
        let value = terrain_base(next, id);
        next.cells[id].terrain_upper = value;
        if value > 0.0 {
            heap.push(Entry { value, id });
        }
    }
    let decay = decay(next);
    for &id in &invalid {
        checkpoint()?;
        for previous in predecessors(next, id)? {
            if invalid.contains(&previous) {
                continue;
            }
            let value = next.cells[previous].terrain_upper - decay;
            if value > 0.0 && relax_upper(next, id, value, &mut heap, &mut touched) {
                stats.upper_relaxed += 1;
            }
        }
    }
    stats.upper_relaxed += propagate_upper(next, &mut heap, &mut touched)?;
    Ok(touched)
}

fn active_edge(map: &Tomogram, from: usize, to: usize) -> bool {
    let value = map.cells[from].terrain_upper - decay(map);
    value > 0.0 && value > terrain_base(map, to) + 1e-9
}

/// 两套 ID 通过 remap 连接。旧删除点仍可连接旧依赖，新插入点只在新图存在。
fn replay_region(
    old: &Tomogram,
    next: &Tomogram,
    remap: &IdRemap,
    dirty: &AHashSet<usize>,
    upper_touched: &AHashSet<usize>,
) -> Result<AHashSet<usize>, String> {
    let mut previous = AHashSet::new();
    let mut current = AHashSet::new();
    let mut queue = VecDeque::new();
    for &id in dirty.iter().chain(upper_touched) {
        checkpoint()?;
        if current.insert(id) {
            queue.push_back((false, id));
        }
    }
    while let Some((is_old, id)) = queue.pop_front() {
        checkpoint()?;
        if is_old {
            if let Some(mapped) = remap.old_to_new(id) {
                if current.insert(mapped) {
                    queue.push_back((false, mapped));
                }
            }
        } else if let Some(mapped) = remap.new_to_old(id) {
            if previous.insert(mapped) {
                queue.push_back((true, mapped));
            }
        }
        let map = if is_old { old } else { next };
        let seen = if is_old { &mut previous } else { &mut current };
        for &target in &map.cells[id].neighbors {
            if active_edge(map, id, target) && seen.insert(target) {
                queue.push_back((is_old, target));
            }
        }
        for source in predecessors(map, id)? {
            if active_edge(map, source, id) && seen.insert(source) {
                queue.push_back((is_old, source));
            }
        }
    }
    Ok(current)
}

fn replay_terrain(map: &mut Tomogram, region: &AHashSet<usize>) -> Result<(), String> {
    let mut ids: Vec<_> = region.iter().copied().collect();
    // 仅排序实际闭包。与冷建共享稳定几何键，存储 ID、哈希迭代和发现顺序
    // 均不决定种子次序；保留完整稳定序列在本闭包上的相对顺序。
    ids.sort_unstable_by_key(|&id| crate::seed_order::SeedKey::for_cell(&map.cells[id]));
    let mut queue = VecDeque::new();
    for id in ids {
        checkpoint()?;
        let value = terrain_base(map, id);
        map.cells[id].terrain = value;
        if value > 0.0 {
            queue.push_back(id);
        }
    }
    let decay = decay(map);
    while let Some(id) = queue.pop_front() {
        checkpoint()?;
        let value = map.cells[id].terrain - decay;
        if value <= 0.0 {
            continue;
        }
        for next in map.cells[id].neighbors.clone() {
            if !region.contains(&next) {
                continue;
            }
            if value > map.cells[next].terrain + 1e-9 {
                map.cells[next].terrain = value;
                // 原队列允许重复，且 pop 时读最新值；两点都必须保留。
                queue.push_back(next);
            }
        }
    }
    checkpoint()
}

/// dirty 覆盖新增、几何/种子键/terrain基值变化、逻辑出边或seed可能变化的
/// origin，以及每个已删除点的存活出邻居。目标ground变化可改变前驱的terrain
/// 基值，即使该逻辑出边未变也须覆盖。仅存储ID移动且语义不变时可不加入dirty，
/// 但remap和拓扑引用必须完整。拥有方须保证旧/新参数相同，并丢弃取消的候选。
/// 返回最终 cost 输入确实改变的当前 ID；此模块不发布快照、不改全局最小值。
pub(crate) fn update(
    old: &Tomogram,
    next: &mut Tomogram,
    remap: &IdRemap,
    dirty: &AHashSet<usize>,
    stats: &mut IncrementalStats,
) -> Result<AHashSet<usize>, String> {
    checkpoint()?;
    let parameters = |map: &Tomogram| {
        [
            map.resolution,
            map.clearance_radius,
            map.half_width,
            map.body_height,
            map.step_up,
            map.step_down,
            map.slope,
        ]
        .map(f64::to_bits)
    };
    if parameters(old) != parameters(next) {
        return Err("层析增量字段要求相同的物理参数".into());
    }
    if dirty.iter().any(|&id| id >= next.cells.len()) {
        return Err("层析增量字段含失效的新单元编号".into());
    }
    if dirty.is_empty() {
        return Ok(AHashSet::new());
    }
    let mut touched = update_distance(old, next, remap, dirty, Field::Wall, stats)?;
    touched.extend(update_distance(
        old,
        next,
        remap,
        dirty,
        Field::Missing,
        stats,
    )?);
    let upper_touched = update_upper(old, next, remap, dirty, stats)?;
    let region = replay_region(old, next, remap, dirty, &upper_touched)?;
    stats.terrain_replayed += region.len();
    replay_terrain(next, &region)?;
    touched.extend(region);
    let mut changed = AHashSet::new();
    for id in touched {
        checkpoint()?;
        let Some(previous) = remap.new_to_old(id) else {
            changed.insert(id);
            continue;
        };
        let a = &old.cells[previous];
        let b = &next.cells[id];
        if [a.clearance, a.support_clearance, a.terrain].map(f64::to_bits)
            != [b.clearance, b.support_clearance, b.terrain].map(f64::to_bits)
        {
            changed.insert(id);
        }
    }
    checkpoint()?;
    Ok(changed)
}
