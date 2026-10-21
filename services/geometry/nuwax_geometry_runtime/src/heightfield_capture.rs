//! Fixed finite source geometry, never a navigation admission decision.
//! Integer heights are q/8 sub-bins. Masks have at most 81 actual XY columns.
use std::collections::{BTreeMap, BTreeSet};
type Xy = [i32; 2];
type Bin = i64;
type Bins = BTreeMap<Xy, BTreeSet<Bin>>;
type Seeds = BTreeMap<Xy, Bin>;

fn window(c: Xy) -> impl Iterator<Item = Xy> {
    (-4..=4).flat_map(move |x| {
        (-4..=4).filter_map(move |y| Some([c[0].checked_add(x)?, c[1].checked_add(y)?]))
    })
}
fn mask(c: Xy, residual: Bin, bins: &Bins, seeds: &Seeds) -> BTreeSet<Xy> {
    window(c)
        .filter(|xy| match (bins.get(xy), seeds.get(xy)) {
            (Some(bins), Some(seed)) => bins
                .range((seed + residual - 2)..=(seed + residual + 2))
                .next()
                .is_some(),
            _ => false,
        })
        .collect()
}
fn dilate(c: Xy, input: &BTreeSet<Xy>) -> u128 {
    let mut mask = 0;
    for xy in input
        .iter()
        .flat_map(|xy| {
            (-1..=1).flat_map(move |x| {
                (-1..=1).filter_map(move |y| Some([xy[0].checked_add(x)?, xy[1].checked_add(y)?]))
            })
        })
        .filter(|xy| {
            (i64::from(xy[0]) - i64::from(c[0])).abs() <= 4
                && (i64::from(xy[1]) - i64::from(c[1])).abs() <= 4
        })
    {
        // 原窗口最多81列；相对整数坐标一一对应，不改变膨胀边界或溢出语义。
        let x = i64::from(xy[0]) - i64::from(c[0]) + 4;
        let y = i64::from(xy[1]) - i64::from(c[1]) + 4;
        mask |= 1_u128 << (x * 9 + y);
    }
    mask
}
fn edge(upper: &BTreeSet<Xy>, lower: &BTreeSet<Xy>) -> bool {
    for d in [[1, 0], [-1, 0], [0, 1], [0, -1]] {
        let tangent = [-d[1], d[0]];
        if upper.intersection(lower).any(|p| {
            (0..3).all(|n| {
                let (Some(ax), Some(ay)) = (
                    p[0].checked_add(n * tangent[0]),
                    p[1].checked_add(n * tangent[1]),
                ) else {
                    return false;
                };
                let a = [ax, ay];
                let (Some(bx), Some(by)) = (a[0].checked_add(d[0]), a[1].checked_add(d[1])) else {
                    return false;
                };
                let b = [bx, by];
                upper.contains(&a)
                    && lower.contains(&a)
                    && lower.contains(&b)
                    && !upper.contains(&b)
            })
        }) {
            return true;
        }
    }
    false
}
fn vertical(bins: &BTreeSet<Bin>) -> bool {
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
fn upright(c: Xy, lower: &BTreeSet<Xy>, bins: &Bins) -> bool {
    let columns: Vec<_> = window(c)
        .filter(|xy| bins.get(xy).is_some_and(vertical))
        .collect();
    if columns.is_empty()
        || columns.iter().filter(|xy| lower.contains(*xy)).count() * 3 < lower.len() * 2
    {
        return false;
    }
    let n = columns.len() as f64;
    let mean = columns.iter().fold([0.; 2], |m, xy| {
        [
            m[0] + f64::from(xy[0] - c[0]) / n,
            m[1] + f64::from(xy[1] - c[1]) / n,
        ]
    });
    let [xx, xy, yy] = columns.iter().fold([0.; 3], |v, p| {
        let x = f64::from(p[0] - c[0]) - mean[0];
        let y = f64::from(p[1] - c[1]) - mean[1];
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
fn shape(c: Xy, base_residual: Bin, residual: Bin, bins: &Bins, seeds: &Seeds) -> Shape {
    let lower = mask(c, base_residual, bins, seeds);
    let upper = mask(c, residual, bins, seeds);
    compare_masks(c, &lower, &upper, bins)
}
fn compare_masks(c: Xy, lower: &BTreeSet<Xy>, upper: &BTreeSet<Xy>, bins: &Bins) -> Shape {
    let a = dilate(c, lower);
    // 相同集合的交并仍为自身，且不存在只属于下层的边缘；无需重复膨胀和查边。
    let (intersection, union, has_edge) = if lower == upper {
        (a.count_ones() as usize, a.count_ones() as usize, false)
    } else {
        let b = dilate(c, upper);
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
        upright: upright(c, lower, bins),
    }
}

use nuwax_map_chunks::heightfield_observations::HeightfieldColumn;
use nuwax_map_chunks::{
    ColumnGroundRecords, HeightfieldSurface, Key, LayerObservation, LayerSample, LayerSource,
    Writer, MAX_LAYER_SAMPLES,
};
use std::sync::Arc;

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
fn expand(columns: &BTreeSet<Xy>, radius: i32) -> BTreeSet<Xy> {
    columns
        .iter()
        .flat_map(|xy| {
            (-radius..=radius).flat_map(move |x| {
                (-radius..=radius)
                    .filter_map(move |y| Some([xy[0].checked_add(x)?, xy[1].checked_add(y)?]))
            })
        })
        .collect()
}
fn select_seeds(bins: &Bins, old: &BTreeMap<Xy, ColumnGroundRecords>, q: f64) -> Seeds {
    bins.iter()
        .filter_map(|(&xy, values)| {
            values
                .iter()
                .copied()
                .map(|z| {
                    let refs: Vec<_> = old
                        .get(&xy)
                        .into_iter()
                        .flat_map(old_records)
                        .filter(|v| v.contains(z.div_euclid(8) as i32))
                        .collect();
                    let distance = refs
                        .iter()
                        .filter_map(|v| v.height(xy, q))
                        .map(|h| ((z as f64 + 0.5) * q / 8.0 - h).abs())
                        .min_by(f64::total_cmp)
                        .unwrap_or(f64::INFINITY);
                    let coverage = window(xy)
                        .filter(|n| {
                            bins.get(n)
                                .is_some_and(|b| b.range(z - 4..=z + 4).next().is_some())
                        })
                        .count();
                    (z, refs.is_empty(), std::cmp::Reverse(coverage), distance)
                })
                .min_by(|a, b| {
                    a.1.cmp(&b.1)
                        .then(a.2.cmp(&b.2))
                        .then(a.3.total_cmp(&b.3))
                        .then(a.0.cmp(&b.0))
                })
                .map(|value| (xy, value.0))
        })
        .collect()
}
#[derive(Debug)]
struct Group {
    seed: Bin,
    bins: BTreeSet<Bin>,
}
fn groups(
    xy: Xy,
    bins: &Bins,
    seeds: &Seeds,
    old: Option<&ColumnGroundRecords>,
    q: f64,
) -> Vec<Group> {
    let Some(&first) = seeds.get(&xy) else {
        return vec![];
    };
    if upright(xy, &mask(xy, 0, bins, seeds), bins) {
        return vec![];
    }
    let values = &bins[&xy];
    let mut remaining = values.clone();
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
            if high - low <= 32 && shape(xy, base, z - first, bins, seeds).joins() {
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
                    }) && (shape(xy, 0, z - first, bins, seeds).edge
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
fn legacy_shape(
    xy: Xy,
    candidate: Bin,
    central: Old<'_>,
    bins: &Bins,
    seeds: &Seeds,
    reference_heights: &BTreeMap<Xy, f64>,
    q: f64,
) -> Option<Shape> {
    let centre = central.height(xy, q)?;
    let delta = (candidate as f64 + 0.5) * q / 8.0 - centre;
    let mut refs = BTreeMap::new();
    for n in window(xy) {
        if !bins.contains_key(&n) {
            continue;
        }
        if n == xy {
            refs.insert(n, centre);
            continue;
        }
        if let Some(&height) = reference_heights.get(&n) {
            refs.insert(n, height);
        }
    }
    if refs.is_empty() {
        return None;
    }
    let lower: BTreeSet<_> = mask(xy, 0, bins, seeds)
        .into_iter()
        .filter(|n| refs.contains_key(n))
        .collect();
    let upper: BTreeSet<_> = refs
        .iter()
        .filter(|(n, h)| {
            bins[*n]
                .iter()
                .any(|z| (((*z as f64 + 0.5) * q / 8.0 - **h) - delta).abs() <= q / 4.0)
        })
        .map(|(n, _)| *n)
        .collect();
    Some(compare_masks(xy, &lower, &upper, bins))
}

/// 邻列参考选择只依赖本帧固定输入，所有中心列可以共享同一结果。
/// 中心列仍在legacy_shape中使用正在比较的具体旧记录覆盖。
fn legacy_reference_heights(
    bins: &Bins,
    seeds: &Seeds,
    old: &BTreeMap<Xy, ColumnGroundRecords>,
    q: f64,
) -> BTreeMap<Xy, f64> {
    let mut references = BTreeMap::new();
    for (&n, records) in old {
        if !bins.contains_key(&n) {
            continue;
        }
        let all: Vec<_> = old_records(records).collect();
        let associated: Vec<_> = all
            .iter()
            .copied()
            .filter(|r| {
                seeds
                    .get(&n)
                    .is_some_and(|z| r.contains(z.div_euclid(8) as i32))
            })
            .collect();
        let reference = if associated.len() == 1 {
            Some(associated[0])
        } else if associated.is_empty() && all.len() == 1 {
            Some(all[0])
        } else {
            None
        };
        if let Some(h) = reference.and_then(|r| r.height(n, q)) {
            references.insert(n, h);
        }
    }
    references
}

// Existing independent surfaces cannot be joined by a frame that sees only a
// middle return. Near-band independence needs an actual original slice edge;
// neither two record names nor a diagnostic sample is such evidence.
fn independent_old_pair(
    xy: Xy,
    a: Old<'_>,
    b: Old<'_>,
    old: &BTreeMap<Xy, ColumnGroundRecords>,
    bins: &Bins,
    seeds: &Seeds,
    historical: &mut Option<(Bins, Seeds)>,
    q: f64,
) -> bool {
    if a.height(xy, q)
        .zip(b.height(xy, q))
        .is_some_and(|(x, y)| (x - y).abs() > 4.0 * q)
    {
        return true;
    }
    let slice_edge = |left: &[Bin], right: &[Bin], bins: &Bins, seeds: &Seeds| {
        let Some(&first) = seeds.get(&xy) else {
            return false;
        };
        let masks: BTreeMap<_, _> = left
            .iter()
            .chain(right)
            .copied()
            .collect::<BTreeSet<_>>()
            .into_iter()
            .map(|z| (z, mask(xy, z - first, bins, seeds)))
            .collect();
        left.iter().any(|&x| {
            right
                .iter()
                .any(|&y| x != y && edge(&masks[&x.max(y)], &masks[&x.min(y)]))
        })
    };
    // Current evidence is usable only if both actual old member sets were seen.
    if let Some(current) = bins.get(&xy) {
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
        if slice_edge(&left, &right, bins, seeds) {
            return true;
        }
    }
    let (Old::Heightfield(a), Old::Heightfield(b)) = (a, b) else {
        return false;
    };
    let positive = |v: &HeightfieldSurface| {
        v.column
            .members()
            .flat_map(|(z, mass)| {
                mass.iter()
                    .enumerate()
                    .filter(|(_, m)| **m > 0.0)
                    .map(move |(i, _)| i64::from(z) * 8 + i as i64)
            })
            .collect::<Vec<_>>()
    };
    let left = positive(a);
    let right = positive(b);
    if left.is_empty() || right.is_empty() {
        return false;
    }
    let (prior_bins, prior_seeds) = historical.get_or_insert_with(|| {
        let prior_bins: Bins = old
            .iter()
            .map(|(&xy, column)| {
                (
                    xy,
                    column
                        .heightfields
                        .iter()
                        .flat_map(|v| positive(v))
                        .collect(),
                )
            })
            .collect();
        let prior_seeds = select_seeds(&prior_bins, old, q);
        (prior_bins, prior_seeds)
    });
    slice_edge(&left, &right, prior_bins, prior_seeds)
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
    let q = f64::from(writer.voxel_size_m());
    let touched: BTreeSet<_> = returns
        .iter()
        .map(|p| [p.key[0], p.key[1]])
        .chain(removed.iter().map(|k| [k[0], k[1]]))
        .collect();
    let writes = expand(&touched, 4);
    let reads = expand(&writes, 8);
    let old = writer.ground_columns(reads)?;
    let mut current: BTreeMap<Xy, Vec<&Return>> = BTreeMap::new();
    for p in returns {
        current.entry([p.key[0], p.key[1]]).or_default().push(p);
    }
    let mut bins: Bins = BTreeMap::new();
    for (&xy, column) in &old {
        if current.contains_key(&xy) {
            continue;
        }
        let values = bins.entry(xy).or_default();
        for field in &column.heightfields {
            for (z, mass) in field.column.members() {
                values.extend(
                    mass.iter()
                        .enumerate()
                        .filter(|(_, m)| **m > 0.)
                        .map(|(b, _)| i64::from(z) * 8 + b as i64),
                );
            }
        }
    }
    for (&xy, points) in &current {
        bins.insert(
            xy,
            points
                .iter()
                .filter(|p| !p.dynamic)
                .map(|p| p.bin)
                .collect(),
        );
    }
    let seeds = select_seeds(&bins, &old, q);
    let reference_heights = legacy_reference_heights(&bins, &seeds, &old, q);
    let mut updates = BTreeMap::new();
    let mut historical = None;
    let mut indices = Vec::new();
    for xy in writes {
        // Absence of a return is not counter-evidence. Original Writer::remove
        // already retracts removed occupancy. Halo participates in geometry only.
        let Some(points) = current.get(&xy) else {
            continue;
        };
        let grouping = groups(xy, &bins, &seeds, old.get(&xy), q);
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
        let mut compatible = |index: usize, seed: Bin| {
            *compatibility.entry((index, seed)).or_insert_with(|| {
                let record = olds[index];
                record
                    .height(xy, q)
                    .is_some_and(|h| ((seed as f64 + 0.5) * q / 8.0 - h).abs() <= 4.0 * q)
                    && legacy_shape(xy, seed, record, &bins, &seeds, &reference_heights, q)
                        .is_none_or(Shape::joins)
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
                .filter(|(index, _)| compatible(*index, group.seed))
                .map(|(_, record)| record)
                .collect();
            if candidates.iter().enumerate().any(|(i, &a)| {
                candidates[i + 1..].iter().any(|&b| {
                    independent_old_pair(xy, a, b, &old, &bins, &seeds, &mut historical, q)
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
            if let Some(frame) = HeightfieldColumn::from_counts(xy, writer.voxel_size_m(), counts)?
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
                .filter(|(_, (g, _, _))| compatible(i, g.seed))
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
            retain_samples(&mut samples, &source);
            next.heightfields.push(HeightfieldSurface {
                column: frame,
                samples,
            });
        }
        // This actual XY was classified, even when mixed/dynamic returns leave
        // no ground declaration. Preserve that distinction beside occupancy.
        next.classified_empty = next.legacy.is_empty() && next.heightfields.is_empty();
        if previous.is_none_or(|before| before != &next) {
            updates.insert(xy, next);
        }
    }
    if !updates.is_empty() {
        writer.replace_ground_columns(updates.iter().map(|(xy, value)| (*xy, value)))?;
    }
    indices.sort_unstable();
    indices.dedup();
    Ok(indices)
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
