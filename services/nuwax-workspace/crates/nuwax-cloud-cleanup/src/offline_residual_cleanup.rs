//! Completed-map cleanup. A fixed Euclidean graph is anchored by measured ground;
//! changing the percentage changes a physical gap tolerance, not graph topology.
//!
//! Components use petgraph's union-find (MIT/Apache-2.0). The neighbourhood follows
//! PCL's Euclidean cluster extraction, but a small component alone is NOT a reason
//! to delete it. Real support and coherent static structures are checked first.
use ahash::{AHashMap as HashMap, AHashSet as HashSet};
use nalgebra::{Matrix3, SymmetricEigen, Vector3};
use nuwax_map_chunks::{ArchiveReader, CachePolicy, GroundRecordRef, Key, Repository, Writer};
use petgraph::unionfind::UnionFind;
use serde::{Deserialize, Serialize};
use std::{fs::File, path::Path};

pub const CONNECT_DISTANCE_M: f64 = 0.12;
pub const SURFACE_DISTANCE_M: f64 = 0.03;

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct CleanupSettings {
    pub strength_percent: u8,
}
impl Default for CleanupSettings {
    fn default() -> Self {
        Self {
            strength_percent: 50,
        }
    }
}
impl CleanupSettings {
    pub fn gap_tolerance_m(self) -> Result<f64, String> {
        if self.strength_percent > 100 {
            return Err("清理力度必须为 0–100%".into());
        }
        // 0% is conservative, 100% is aggressive. Every integer step is real.
        Ok(0.30 - 0.25 * f64::from(self.strength_percent) / 100.0)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Support {
    MeasuredGround,
    ConnectedStructure,
    CoherentStructure,
    SurfaceContinuation,
    Unsupported,
    Unknown,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Component {
    pub points: usize,
    pub bounds_min: [f64; 3],
    pub bounds_max: [f64; 3],
    pub support: Support,
    pub gap_m: Option<f64>,
    pub selected: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CleanupPreview {
    pub settings: CleanupSettings,
    pub gap_tolerance_m: f64,
    pub source_points: usize,
    pub candidate_points: usize,
    pub measured_ground_points: usize,
    pub components: Vec<Component>,
    /// Exact original asset keys. Apply never recomputes the candidate selection.
    pub removed_keys: Vec<Key>,
}

#[derive(Clone, Debug)]
pub struct CompletedMap {
    pub voxel_size_m: f32,
    pub keys: Vec<Key>,
    pub hits: Vec<u16>,
    pub ground: HashSet<Key>,
    pub ground_samples: Vec<[f64; 3]>,
    measured: HashMap<Key, Vec<[f64; 3]>>,
}
/// Match the production fine asset's f64 * reciprocal(f32 quantum), including
/// boundaries. Never substitute literal 0.05 or f32 reciprocal here.
pub fn asset_key(p: [f64; 3], quantum: f32) -> Key {
    let inv = 1.0 / f64::from(quantum);
    p.map(|v| (v * inv).floor() as i32)
}
fn center(k: Key, q: f32) -> [f64; 3] {
    k.map(|v| (f64::from(v) + 0.5) * f64::from(q))
}
fn distance2(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|i| (a[i] - b[i]).powi(2)).sum()
}
fn grid(p: [f64; 3], q: f64) -> Key {
    p.map(|v| (v / q).floor() as i32)
}

impl CompletedMap {
    pub fn read(path: &Path) -> Result<Self, String> {
        let mut reader = ArchiveReader::open(File::open(path).map_err(|e| e.to_string())?)?;
        let q = reader.root().voxel_size_m();
        let mut map = Self {
            voxel_size_m: q,
            keys: Vec::new(),
            hits: Vec::new(),
            ground: HashSet::new(),
            ground_samples: Vec::new(),
            measured: HashMap::new(),
        };
        for (block_key, _, _) in reader.root().objects() {
            let bytes = reader.block_by_key(block_key)?;
            let block = nuwax_map_chunks::decode_block(&bytes)?;
            for (&local, evidence) in &block {
                map.keys
                    .push(nuwax_map_chunks::global_key(block_key, local)?);
                map.hits.push(evidence.hits);
            }
            for (_, record) in nuwax_map_chunks::decode_ground_records(&bytes)?.iter() {
                match record {
                    GroundRecordRef::HeightfieldV1(v) => {
                        for (local, mass) in &v.members {
                            if mass.iter().any(|v| *v > 0.0) {
                                map.ground
                                    .insert(nuwax_map_chunks::global_key(block_key, *local)?);
                            }
                        }
                        map.ground_samples.extend(v.samples.iter().map(|v| v.point));
                    }
                    GroundRecordRef::LegacyPlane(v) if v.normal[2] >= 0.5 => {
                        for local in &v.members {
                            map.ground
                                .insert(nuwax_map_chunks::global_key(block_key, *local)?);
                        }
                        map.ground_samples.extend(v.samples.iter().map(|v| v.point));
                    }
                    _ => {}
                }
            }
        }
        // Heightfield ownership records contain measured modes, not an upward
        // normal or ground permission. Qualify their actual samples as terrain;
        // never protect every member of a vertical human/wall mode as ground.
        let qualified = local_ground_indices(&map.ground_samples);
        let samples: Vec<_> = qualified
            .into_iter()
            .map(|i| map.ground_samples[i])
            .collect();
        let keys: HashSet<_> = samples.iter().map(|&p| asset_key(p, q)).collect();
        map.ground.retain(|k| keys.contains(k));
        map.ground_samples = samples;
        Ok(map)
    }

    /// Only use measured returns to fill an absent local ground classification.
    /// Existing ground always wins. The fit is local; it does not extrapolate an
    /// infinite floor across stairs, overhangs, unknown columns, or other storeys.
    pub fn add_ground_fallback(&mut self, measured: &[[f64; 3]]) {
        let fitted = local_ground_indices(measured);
        let present: HashSet<_> = self.keys.iter().copied().collect();
        let classified_xy: HashSet<_> = self.ground.iter().map(|k| [k[0], k[1]]).collect();
        let mut unique = HashSet::new();
        for &p in measured {
            let k = asset_key(p, self.voxel_size_m);
            if present.contains(&k) && unique.insert(p.map(f64::to_bits)) {
                self.measured.entry(k).or_default().push(p);
            }
        }
        for i in fitted {
            let p = measured[i];
            let k = asset_key(p, self.voxel_size_m);
            if present.contains(&k) && !classified_xy.contains(&[k[0], k[1]]) {
                self.ground.insert(k);
                self.ground_samples.push(p);
            }
        }
    }

    pub fn preview(
        &self,
        settings: CleanupSettings,
        cancelled: impl Fn() -> bool,
    ) -> Result<CleanupPreview, String> {
        let tolerance = settings.gap_tolerance_m()?;
        if self.keys.len() != self.hits.len() {
            return Err("点云统计与原始键不匹配".into());
        }
        let points: Vec<_> = self
            .keys
            .iter()
            .map(|&k| {
                self.measured
                    .get(&k)
                    .and_then(|v| v.first())
                    .copied()
                    .unwrap_or_else(|| center(k, self.voxel_size_m))
            })
            .collect();
        // Geometry comes from actual saved returns when available. Keep every
        // distinct return inside a voxel for surface/line fits; voxel centres
        // would hide thin edges. Counts and deletion still use exact asset keys.
        let mut samples = Vec::new();
        let mut samples_for_key = Vec::new();
        for (i, k) in self.keys.iter().enumerate() {
            let start = samples.len();
            if let Some(measured) = self.measured.get(k) {
                samples.extend(measured.iter().copied());
            } else {
                samples.push(points[i]);
            }
            samples_for_key.push((start..samples.len()).collect::<Vec<_>>());
        }
        let mut cells: HashMap<Key, Vec<usize>> = HashMap::new();
        let mut union = UnionFind::<usize>::new(points.len());
        for (i, &p) in points.iter().enumerate() {
            if i % 4096 == 0 && cancelled() {
                return Err("清理已取消，原图未改变".into());
            }
            let k = grid(p, CONNECT_DISTANCE_M);
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        if let Some(ids) = cells.get(&[k[0] + dx, k[1] + dy, k[2] + dz]) {
                            for &j in ids {
                                if distance2(p, points[j]) <= CONNECT_DISTANCE_M.powi(2) {
                                    union.union(i, j);
                                }
                            }
                        }
                    }
                }
            }
            cells.entry(k).or_default().push(i);
        }
        let mut groups: std::collections::BTreeMap<usize, Vec<usize>> =
            std::collections::BTreeMap::new();
        for i in 0..points.len() {
            groups.entry(union.find(i)).or_default().push(i);
        }
        let mut supported = HashSet::new();
        let mut grounded_roots = HashSet::new();
        for (&root, ids) in &groups {
            if ids.iter().any(|&i| self.ground.contains(&self.keys[i])) {
                supported.extend(ids.iter().copied());
                grounded_roots.insert(root);
            }
        }
        // A plane/line is protected only when observed repeatedly and it forms
        // a genuine extended surface. Compact head/torso blobs are not anchors.
        let mut coherent_roots = HashSet::new();
        for (&root, ids) in &groups {
            if !grounded_roots.contains(&root) && coherent_structure(ids, &points, &self.hits) {
                coherent_roots.insert(root);
                supported.extend(ids.iter().copied());
            }
        }
        let have_ground = !grounded_roots.is_empty();
        // Surface reachability crosses sampling/occlusion gaps only along a
        // verified 3cm local plane or line. Newly connected real surfaces can support
        // the next patch of the same wall, floor or table.
        let mut surface_index: HashMap<Key, Vec<usize>> = HashMap::new();
        let mut ground_index: HashMap<Key, Vec<usize>> = HashMap::new();
        for i in 0..points.len() {
            if self.ground.contains(&self.keys[i]) {
                for &j in &samples_for_key[i] {
                    ground_index
                        .entry(grid(samples[j], 0.75))
                        .or_default()
                        .push(j);
                }
            }
        }
        for &i in &supported {
            if self.hits[i] >= 5 || self.ground.contains(&self.keys[i]) {
                for &j in &samples_for_key[i] {
                    surface_index
                        .entry(grid(samples[j], 0.75))
                        .or_default()
                        .push(j);
                }
            }
        }
        let mut continuation_roots = HashSet::new();
        if have_ground {
            loop {
                let mut newly = Vec::new();
                for (&root, ids) in &groups {
                    if cancelled() {
                        return Err("清理已取消，原图未改变".into());
                    }
                    if grounded_roots.contains(&root)
                        || coherent_roots.contains(&root)
                        || continuation_roots.contains(&root)
                    {
                        continue;
                    }
                    if ids.iter().any(|&i| {
                        supported_surface(points[i], &samples, &ground_index)
                            || supported_surface(points[i], &samples, &surface_index)
                    }) {
                        newly.push(root);
                    }
                }
                if newly.is_empty() {
                    break;
                }
                for root in newly {
                    continuation_roots.insert(root);
                    for &i in &groups[&root] {
                        supported.insert(i);
                        if self.hits[i] >= 5 {
                            for &j in &samples_for_key[i] {
                                surface_index
                                    .entry(grid(samples[j], 0.75))
                                    .or_default()
                                    .push(j);
                            }
                        }
                    }
                }
            }
        }
        // Nearest support is a bounded spatial query, not N components × N points.
        let search = tolerance.max(CONNECT_DISTANCE_M) + f64::from(self.voxel_size_m);
        let mut support_index: HashMap<Key, Vec<usize>> = HashMap::new();
        for &i in &supported {
            support_index
                .entry(grid(points[i], search))
                .or_default()
                .push(i);
        }
        let mut components = Vec::new();
        let mut removed_keys = Vec::new();
        for (root, ids) in groups {
            if cancelled() {
                return Err("清理已取消，原图未改变".into());
            }
            let mut low = [f64::INFINITY; 3];
            let mut high = [f64::NEG_INFINITY; 3];
            for &i in &ids {
                for a in 0..3 {
                    low[a] = low[a].min(points[i][a]);
                    high[a] = high[a].max(points[i][a]);
                }
            }
            let (support, gap) = if grounded_roots.contains(&root) {
                let ground_only = ids.iter().all(|&i| self.ground.contains(&self.keys[i]));
                (
                    if ground_only {
                        Support::MeasuredGround
                    } else {
                        Support::ConnectedStructure
                    },
                    Some(0.0),
                )
            } else if coherent_roots.contains(&root) {
                (Support::CoherentStructure, Some(0.0))
            } else if !have_ground {
                (Support::Unknown, None)
            } else if continuation_roots.contains(&root) {
                (Support::SurfaceContinuation, None)
            } else {
                let mut d2 = f64::INFINITY;
                for &i in &ids {
                    let k = grid(points[i], search);
                    for dx in -1..=1 {
                        for dy in -1..=1 {
                            for dz in -1..=1 {
                                if let Some(js) =
                                    support_index.get(&[k[0] + dx, k[1] + dy, k[2] + dz])
                                {
                                    for &j in js {
                                        d2 = d2.min(distance2(points[i], points[j]));
                                    }
                                }
                            }
                        }
                    }
                }
                // Infinity means no support within the complete search radius,
                // not a fabricated distance. Keep the report's distance optional.
                (Support::Unsupported, d2.is_finite().then(|| d2.sqrt()))
            };
            let selected = support == Support::Unsupported && gap.is_none_or(|g| g > tolerance);
            if selected {
                removed_keys.extend(ids.iter().map(|&i| self.keys[i]));
            }
            components.push(Component {
                points: ids.len(),
                bounds_min: low,
                bounds_max: high,
                support,
                gap_m: gap,
                selected,
            });
        }
        removed_keys.sort_unstable();
        Ok(CleanupPreview {
            settings,
            gap_tolerance_m: tolerance,
            source_points: self.keys.len(),
            candidate_points: removed_keys.len(),
            measured_ground_points: self.ground.len(),
            components,
            removed_keys,
        })
    }
}

/// A bounded local plane fit against already supported returns. The candidate
/// never votes for its own support. This recognizes a sampled surface across an
/// occlusion gap; it does not fill unknown space or create ground observations.
fn supported_surface(p: [f64; 3], points: &[[f64; 3]], index: &HashMap<Key, Vec<usize>>) -> bool {
    const RADIUS: f64 = 0.75;
    let k = grid(p, RADIUS);
    let mut neighbours = Vec::new();
    for x in -1..=1 {
        for y in -1..=1 {
            for z in -1..=1 {
                if let Some(ids) = index.get(&[k[0] + x, k[1] + y, k[2] + z]) {
                    neighbours.extend(
                        ids.iter()
                            .copied()
                            .filter(|&i| distance2(p, points[i]) <= RADIUS * RADIUS),
                    );
                }
            }
        }
    }
    if neighbours.len() < 6 {
        return false;
    }
    neighbours.sort_by(|&a, &b| {
        points[a][0]
            .total_cmp(&points[b][0])
            .then(points[a][1].total_cmp(&points[b][1]))
            .then(points[a][2].total_cmp(&points[b][2]))
    });
    let sample: Vec<_> = neighbours
        .iter()
        .step_by(neighbours.len().div_ceil(12))
        .copied()
        .collect();
    for a in 0..sample.len() {
        for b in a + 1..sample.len() {
            let origin = Vector3::from(points[sample[a]]);
            let d = Vector3::from(points[sample[b]]) - origin;
            if d.norm() < 0.1 {
                continue;
            }
            let d = d.normalize();
            let v = Vector3::from(p) - origin;
            if (v - d * v.dot(&d)).norm() > SURFACE_DISTANCE_M {
                continue;
            }
            let mut along: Vec<_> = neighbours
                .iter()
                .filter_map(|&i| {
                    let v = Vector3::from(points[i]) - origin;
                    ((v - d * v.dot(&d)).norm() <= 0.01).then(|| v.dot(&d))
                })
                .collect();
            along.sort_by(f64::total_cmp);
            let Some(&lo) = along.first() else {
                continue;
            };
            let hi = *along.last().unwrap();
            let mut separated = 1;
            let mut last = lo;
            for &t in &along {
                if t - last >= 0.1 {
                    separated += 1;
                    last = t;
                }
            }
            // Three distinct, separated observations of the same thin line,
            // with the candidate between its observed ends, not an infinite
            // extension or a plane invented from a pole and a floor edge.
            if separated >= 3 && v.dot(&d) >= lo && v.dot(&d) <= hi {
                return true;
            }
        }
    }
    for a in 0..sample.len() {
        for b in a + 1..sample.len() {
            for c in b + 1..sample.len() {
                let origin = Vector3::from(points[sample[a]]);
                let n = (Vector3::from(points[sample[b]]) - origin)
                    .cross(&(Vector3::from(points[sample[c]]) - origin));
                let area = n.norm();
                if area < 0.012 {
                    continue;
                }
                let n = n / area;
                if (Vector3::from(p) - origin).dot(&n).abs() > SURFACE_DISTANCE_M {
                    continue;
                }
                for anchor in [sample[a], sample[b], sample[c]] {
                    let local: Vec<_> = neighbours
                        .iter()
                        .copied()
                        .filter(|&i| distance2(points[anchor], points[i]) <= 0.3_f64.powi(2))
                        .collect();
                    let fitted: Vec<_> = local
                        .iter()
                        .copied()
                        .filter(|&i| {
                            (Vector3::from(points[i]) - origin).dot(&n).abs() <= SURFACE_DISTANCE_M
                        })
                        .collect();
                    if fitted.len() < 6 || fitted.len() * 4 < local.len() * 3 {
                        continue;
                    }
                    let mean = fitted
                        .iter()
                        .fold(Vector3::zeros(), |s, &i| s + Vector3::from(points[i]))
                        / fitted.len() as f64;
                    let cov = fitted.iter().fold(Matrix3::zeros(), |s, &i| {
                        let d = Vector3::from(points[i]) - mean;
                        s + d * d.transpose()
                    }) / fitted.len() as f64;
                    let mut e = SymmetricEigen::new(cov).eigenvalues.as_slice().to_vec();
                    e.sort_by(f64::total_cmp);
                    // A real two-dimensional local patch. A pole and a floor
                    // line must not define a fictitious diagonal support plane.
                    if e[0] <= SURFACE_DISTANCE_M.powi(2)
                        && e[1] >= 0.1_f64.powi(2) / 12.
                        && e[2] >= 0.2_f64.powi(2) / 12.
                    {
                        return true;
                    }
                }
            }
        }
    }
    false
}

fn coherent_structure(ids: &[usize], points: &[[f64; 3]], hits: &[u16]) -> bool {
    if ids.len() < 8 || ids.iter().filter(|&&i| hits[i] >= 5).count() * 5 < ids.len() * 4 {
        return false;
    }
    let mean = ids
        .iter()
        .fold(Vector3::zeros(), |s, &i| s + Vector3::from(points[i]))
        / ids.len() as f64;
    let cov = ids.iter().fold(Matrix3::zeros(), |s, &i| {
        let d = Vector3::from(points[i]) - mean;
        s + d * d.transpose()
    }) / ids.len() as f64;
    let mut e = SymmetricEigen::new(cov).eigenvalues.as_slice().to_vec();
    e.sort_by(f64::total_cmp);
    // >= 0.8 m major extent, plus a true broad plane or a thin long line.
    // Repeated support alone cannot turn a small floating blob into a structure.
    e[2] >= 0.8_f64.powi(2) / 12.0
        && (e[0] <= SURFACE_DISTANCE_M.powi(2) && e[1] >= 0.4_f64.powi(2) / 12.0
            || e[1] <= 0.015_f64.powi(2) && e[2] >= 1.2_f64.powi(2) / 12.0)
}

/// Local lower-surface RANSAC from actual returns. Multiple levels are retained;
/// neighbour height is not copied into columns without measurements.
pub fn local_ground_indices(points: &[[f64; 3]]) -> Vec<usize> {
    let mut patches: HashMap<[i32; 2], Vec<usize>> = HashMap::new();
    for (i, p) in points.iter().enumerate() {
        if p.iter().all(|v| v.is_finite()) {
            patches
                .entry([(p[0] / 0.5).floor() as i32, (p[1] / 0.5).floor() as i32])
                .or_default()
                .push(i);
        }
    }
    let mut fits: HashMap<[i32; 2], (f64, Vec<usize>)> = HashMap::new();
    for (&patch, ids) in &patches {
        if ids.len() < 6 {
            continue;
        }
        let mut sorted = ids.clone();
        sorted.sort_by(|&a, &b| points[a][2].total_cmp(&points[b][2]));
        let low = points[sorted[0]][2];
        let seeds: Vec<_> = sorted
            .into_iter()
            .filter(|&i| points[i][2] <= low + 0.12)
            .collect();
        if seeds.len() < 6 {
            continue;
        }
        let stride = seeds.len().div_ceil(12).max(1);
        let sample: Vec<_> = seeds.iter().step_by(stride).copied().collect();
        let mut best: Vec<usize> = Vec::new();
        for a in 0..sample.len() {
            for b in a + 1..sample.len() {
                for c in b + 1..sample.len() {
                    let p = Vector3::from(points[sample[a]]);
                    let u = Vector3::from(points[sample[b]]) - p;
                    let v = Vector3::from(points[sample[c]]) - p;
                    let n = u.cross(&v);
                    let area = n.norm();
                    if area < 0.012 {
                        continue;
                    }
                    let n = n / area;
                    if n.z.abs() < 0.85 {
                        continue;
                    }
                    let fit: Vec<_> = ids
                        .iter()
                        .copied()
                        .filter(|&i| (Vector3::from(points[i]) - p).dot(&n).abs() <= 0.02)
                        .collect();
                    if fit.len() > best.len() {
                        best = fit;
                    }
                }
            }
        }
        if best.len() >= 6 {
            let height = best.iter().map(|&i| points[i][2]).sum::<f64>() / best.len() as f64;
            fits.insert(patch, (height, best));
        }
    }
    // A locally flat head or table top is not ground. Start at the observed
    // lowest terrain and follow neighbouring measured patches, allowing normal
    // stair rises. No height is fabricated in empty patches.
    let Some(low) = fits.values().map(|v| v.0).min_by(f64::total_cmp) else {
        return Vec::new();
    };
    let mut queue: std::collections::VecDeque<_> = fits
        .iter()
        .filter_map(|(&k, v)| (v.0 <= low + 0.12).then_some(k))
        .collect();
    let mut connected: HashSet<_> = queue.iter().copied().collect();
    while let Some(k) = queue.pop_front() {
        let h = fits[&k].0;
        for x in -1..=1 {
            for y in -1..=1 {
                let n = [k[0] + x, k[1] + y];
                if !connected.contains(&n) && fits.get(&n).is_some_and(|v| (v.0 - h).abs() <= 0.30)
                {
                    connected.insert(n);
                    queue.push_back(n);
                }
            }
        }
    }
    let mut result: Vec<_> = connected
        .into_iter()
        .flat_map(|k| fits.remove(&k).unwrap().1)
        .collect();
    result.sort_unstable();
    result.dedup();
    result
}

pub fn apply_fine(input: &Path, output: &Path, preview: &CleanupPreview) -> Result<usize, String> {
    apply_fine_with_recovery(input, output, preview, &HashSet::new())
}
pub fn apply_fine_with_recovery(
    input: &Path,
    output: &Path,
    preview: &CleanupPreview,
    recovered: &HashSet<Key>,
) -> Result<usize, String> {
    apply_fine_with_recovery_with_cache(
        input,
        output,
        preview,
        recovered,
        32 * 1024 * 1024,
        CachePolicy::Fixed,
    )
}

/// 生产拥有方显式传入已解析的缓存预算与策略，地图内容及预览选择保持一致。
pub fn apply_fine_with_recovery_with_cache(
    input: &Path,
    output: &Path,
    preview: &CleanupPreview,
    recovered: &HashSet<Key>,
    cache_bytes: usize,
    cache_policy: CachePolicy,
) -> Result<usize, String> {
    if input == output {
        return Err("清理必须另存新图".into());
    }
    let mut reader = ArchiveReader::open(File::open(input).map_err(|e| e.to_string())?)?;
    let repository = Repository::temporary()?;
    let root = reader.import(&repository)?;
    if root.points() as usize != preview.source_points {
        return Err("原图已变化，请重新预览".into());
    }
    let mut writer = Writer::new(repository, root.voxel_size_m(), cache_bytes)?;
    writer.set_cache_policy(cache_policy)?;
    writer.restore_archived(&root)?;
    writer.remove(preview.removed_keys.iter().copied())?;
    writer.seed_unknown(recovered.iter().copied())?;
    let sealed = writer.seal(
        root.domain().clone(),
        root.revision() + 1,
        root.source_timestamp_ns(),
    )?;
    let count = sealed.manifest().points() as usize;
    if count + preview.candidate_points != preview.source_points + recovered.len() {
        return Err("实际删除数量与预览不一致".into());
    }
    let (mut archive, _) = nuwax_map_chunks::archive_file(sealed.manifest(), &sealed.repository)?;
    let mut out = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output)
        .map_err(|e| e.to_string())?;
    std::io::copy(&mut archive, &mut out).map_err(|e| e.to_string())?;
    out.sync_all().map_err(|e| e.to_string())?;
    Ok(count)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn sample() -> CompletedMap {
        let mut map = CompletedMap {
            voxel_size_m: 0.05,
            keys: Vec::new(),
            hits: Vec::new(),
            ground: HashSet::new(),
            ground_samples: Vec::new(),
            measured: HashMap::new(),
        };
        for x in -10..=10 {
            for y in -10..=10 {
                let k = [x, y, 0];
                map.keys.push(k);
                map.hits.push(10);
                map.ground.insert(k);
            }
        }
        // A wall/leg is genuinely connected to the measured floor.
        for z in 1..=30 {
            map.keys.push([0, 0, z]);
            map.hits.push(10);
        }
        // A small unsupported ghost above the floor.
        for x in 5..=7 {
            for z in 15..=17 {
                map.keys.push([x, 5, z]);
                map.hits.push(2);
            }
        }
        map
    }
    #[test]
    fn height_modes_on_a_vertical_body_are_not_qualified_ground() {
        let mut measured = Vec::new();
        for x in 0..=100 {
            for y in -30..=30 {
                measured.push([x as f64 * 0.04, y as f64 * 0.04, 0.]);
            }
        }
        let ground_count = measured.len();
        for z in 0..=34 {
            for angle in 0..=40 {
                let theta = std::f64::consts::PI / 2. + angle as f64 * std::f64::consts::PI / 40.;
                measured.push([
                    3. + 0.25 * theta.cos(),
                    0.25 * theta.sin(),
                    0.12 + z as f64 * 0.05,
                ]);
            }
        }
        let qualified = local_ground_indices(&measured);
        assert!(!qualified.is_empty());
        assert!(
            qualified.iter().all(|&i| i < ground_count),
            "measured height modes are not a license to call a vertical human ground"
        );
    }
    #[test]
    fn support_not_component_size_controls_cleanup() {
        let m = sample();
        let p = m.preview(CleanupSettings::default(), || false).unwrap();
        assert_eq!(p.candidate_points, 9);
        assert!(p.removed_keys.iter().all(|k| k[0] >= 5));
        assert!(p
            .components
            .iter()
            .any(|c| c.support == Support::ConnectedStructure));
    }
    #[test]
    fn all_101_strengths_are_ordered_and_use_real_tolerances() {
        let m = sample();
        let mut previous = HashSet::new();
        let mut tolerance = 1.0;
        for s in 0..=100 {
            let p = m
                .preview(
                    CleanupSettings {
                        strength_percent: s,
                    },
                    || false,
                )
                .unwrap();
            let next: HashSet<_> = p.removed_keys.iter().copied().collect();
            assert!(previous.is_subset(&next));
            assert!(p.gap_tolerance_m < tolerance);
            tolerance = p.gap_tolerance_m;
            previous = next;
        }
    }
    #[test]
    fn absent_ground_is_unknown_not_blanket_delete() {
        let mut m = sample();
        m.ground.clear();
        let p = m.preview(CleanupSettings::default(), || false).unwrap();
        assert_eq!(p.candidate_points, 0);
        assert!(p.components.iter().any(|c| c.support == Support::Unknown));
    }
    #[test]
    fn cancellation_has_no_output() {
        assert!(sample()
            .preview(CleanupSettings::default(), || true)
            .is_err());
    }
    #[test]
    fn unsupported_gap_changes_at_intermediate_percentages() {
        let mut m = sample();
        m.keys.push([8, 8, 4]);
        m.hits.push(1);
        let low = m
            .preview(
                CleanupSettings {
                    strength_percent: 20,
                },
                || false,
            )
            .unwrap();
        let high = m
            .preview(
                CleanupSettings {
                    strength_percent: 80,
                },
                || false,
            )
            .unwrap();
        assert!(!low.removed_keys.contains(&[8, 8, 4]));
        assert!(high.removed_keys.contains(&[8, 8, 4]));
    }
    #[test]
    fn supported_plane_continues_across_occlusion_but_not_above_it() {
        let mut m = sample();
        m.keys.push([15, 0, 0]);
        m.hits.push(190);
        m.keys.push([15, 0, 5]);
        m.hits.push(190);
        let preview = m
            .preview(
                CleanupSettings {
                    strength_percent: 100,
                },
                || false,
            )
            .unwrap();
        assert!(!preview.removed_keys.contains(&[15, 0, 0]));
        assert!(preview.removed_keys.contains(&[15, 0, 5]));
        assert!(preview
            .components
            .iter()
            .any(|c| c.support == Support::SurfaceContinuation));
    }
    #[test]
    #[ignore = "requires unchanged native saved-map files"]
    fn replay_saved_static_geometry_support() {
        use std::io::Read;
        let folder = std::path::PathBuf::from(std::env::var("NEH_STATIC_SAVED_CASE").unwrap());
        let mut map = CompletedMap::read(&folder.join("original-fine.bin")).unwrap();
        let mut stream = File::open(folder.join("original-dense.bin")).unwrap();
        let mut magic = [0u8; 8];
        stream.read_exact(&mut magic).unwrap();
        assert_eq!(&magic, b"NUWAXDC1");
        let mut length = [0u8; 4];
        stream.read_exact(&mut length).unwrap();
        let mut meta = vec![0; u32::from_le_bytes(length) as usize];
        stream.read_exact(&mut meta).unwrap();
        let mut bytes = Vec::new();
        stream.read_to_end(&mut bytes).unwrap();
        let dense: Vec<_> = bytes
            .chunks_exact(12)
            .map(|p| {
                std::array::from_fn(|a| {
                    f32::from_le_bytes(p[a * 4..a * 4 + 4].try_into().unwrap()) as f64
                })
            })
            .collect();
        map.add_ground_fallback(&dense);
        let preview = map.preview(CleanupSettings::default(), || false).unwrap();
        std::fs::write(
            folder.join("support-replay.json"),
            serde_json::to_vec_pretty(&preview).unwrap(),
        )
        .unwrap();
        assert_eq!(
            preview.candidate_points, 0,
            "static cumulative geometry must remain; exact failed keys: {:?}",
            preview.removed_keys
        );
    }
    #[test]
    fn configured_cleanup_cache_preserves_selected_keys_and_recovery() {
        let folder = std::env::temp_dir().join(format!(
            "neh-cleanup-cache-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir_all(&folder).unwrap();
        let repository = Repository::temporary().unwrap();
        let mut writer = Writer::new(repository, 0.05, 4 * 1024 * 1024).unwrap();
        writer
            .seed_unknown([[0, 0, 0], [1, 0, 0], [2, 0, 0]])
            .unwrap();
        let sealed = writer
            .seal(
                nuwax_map_chunks::Domain {
                    session: "清理缓存验证".into(),
                    generation: 1,
                    frame_generation: 0,
                    producer_generation: 1,
                },
                1,
                100,
            )
            .unwrap();
        let (mut archive, _) =
            nuwax_map_chunks::archive_file(sealed.manifest(), &sealed.repository).unwrap();
        let input = folder.join("source.bin");
        std::io::copy(&mut archive, &mut File::create(&input).unwrap()).unwrap();
        let original = std::fs::read(&input).unwrap();
        let preview = CleanupPreview {
            settings: CleanupSettings::default(),
            gap_tolerance_m: 0.175,
            source_points: 3,
            candidate_points: 1,
            measured_ground_points: 0,
            components: vec![],
            removed_keys: vec![[1, 0, 0]],
        };
        let recovered = HashSet::from([[3, 0, 0]]);
        for (index, budget, policy) in [
            (0, 4, CachePolicy::Fixed),
            (1, 64, CachePolicy::Fixed),
            (
                2,
                4,
                CachePolicy::Auto {
                    max_bytes: 128 * 1024 * 1024,
                },
            ),
        ] {
            let output = folder.join(format!("cleaned-{index}.bin"));
            assert_eq!(
                apply_fine_with_recovery_with_cache(
                    &input,
                    &output,
                    &preview,
                    &recovered,
                    budget * 1024 * 1024,
                    policy
                )
                .unwrap(),
                3
            );
            let map = CompletedMap::read(&output).unwrap();
            assert_eq!(
                map.keys.into_iter().collect::<HashSet<_>>(),
                HashSet::from([[0, 0, 0], [2, 0, 0], [3, 0, 0]])
            );
        }
        let invalid = folder.join("invalid-policy.bin");
        assert!(apply_fine_with_recovery_with_cache(
            &input,
            &invalid,
            &preview,
            &recovered,
            8 * 1024 * 1024,
            CachePolicy::Auto {
                max_bytes: 4 * 1024 * 1024
            }
        )
        .is_err());
        assert!(!invalid.exists(), "无效缓存策略必须在写出制品之前报告");
        assert_eq!(std::fs::read(input).unwrap(), original);
        std::fs::remove_dir_all(folder).unwrap();
    }

    #[test]
    fn asset_key_uses_the_stored_f32_quantum() {
        let q = 0.05_f32;
        assert_eq!(asset_key([0.15, -0.15, 1.0], q), [2, -3, 19]);
    }
}
