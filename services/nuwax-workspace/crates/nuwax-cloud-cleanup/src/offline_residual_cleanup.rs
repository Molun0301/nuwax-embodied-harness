//! Completed-map cleanup. A fixed Euclidean graph is anchored by measured ground;
//! changing the percentage changes a physical gap tolerance, not graph topology.
//!
//! Components use petgraph's union-find (MIT/Apache-2.0). The neighbourhood follows
//! PCL's Euclidean cluster extraction, but a small component alone is NOT a reason
//! to delete it. Real support and coherent static structures are checked first.
use ahash::{AHashMap as HashMap, AHashSet as HashSet};
use nalgebra::{Matrix3, SymmetricEigen, Vector3};
use nuwax_map_chunks::{ArchiveReader, GroundRecordRef, Key, Writer, Repository};
use petgraph::unionfind::UnionFind;
use serde::{Deserialize, Serialize};
use std::{fs::File, path::Path};

pub const CONNECT_DISTANCE_M: f64 = 0.12;
pub const SURFACE_DISTANCE_M: f64 = 0.03;

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct CleanupSettings { pub strength_percent: u8 }
impl Default for CleanupSettings { fn default() -> Self { Self { strength_percent: 50 } } }
impl CleanupSettings {
    pub fn gap_tolerance_m(self) -> Result<f64, String> {
        if self.strength_percent > 100 { return Err("清理力度必须为 0–100%".into()); }
        // 0% is conservative, 100% is aggressive. Every integer step is real.
        Ok(0.30 - 0.25 * f64::from(self.strength_percent) / 100.0)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Support { MeasuredGround, ConnectedStructure, CoherentStructure, Unsupported, Unknown }
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
}
/// Match the production fine asset's f64 * reciprocal(f32 quantum), including
/// boundaries. Never substitute literal 0.05 or f32 reciprocal here.
pub fn asset_key(p: [f64; 3], quantum: f32) -> Key {
    let inv = 1.0 / f64::from(quantum);
    p.map(|v| (v * inv).floor() as i32)
}
fn center(k: Key, q: f32) -> [f64; 3] { k.map(|v| (f64::from(v) + 0.5) * f64::from(q)) }
fn distance2(a: [f64; 3], b: [f64; 3]) -> f64 { (0..3).map(|i| (a[i]-b[i]).powi(2)).sum() }
fn grid(p: [f64; 3], q: f64) -> Key { p.map(|v| (v / q).floor() as i32) }

impl CompletedMap {
    pub fn read(path: &Path) -> Result<Self, String> {
        let mut reader = ArchiveReader::open(File::open(path).map_err(|e|e.to_string())?)?;
        let q = reader.root().voxel_size_m();
        let mut map = Self { voxel_size_m: q, keys: Vec::new(), hits: Vec::new(), ground: HashSet::new(), ground_samples: Vec::new() };
        for (block_key, _, _) in reader.root().objects() {
            let bytes = reader.block_by_key(block_key)?;
            let block = nuwax_map_chunks::decode_block(&bytes)?;
            for (&local, evidence) in &block {
                map.keys.push(nuwax_map_chunks::global_key(block_key, local)?);
                map.hits.push(evidence.hits);
            }
            for (_, record) in nuwax_map_chunks::decode_ground_records(&bytes)?.iter() {
                match record {
                    GroundRecordRef::HeightfieldV1(v) => {
                        for (local, mass) in &v.members {
                            if mass.iter().any(|v| *v > 0.0) {
                                map.ground.insert(nuwax_map_chunks::global_key(block_key, *local)?);
                            }
                        }
                        map.ground_samples.extend(v.samples.iter().map(|v| v.point));
                    },
                    GroundRecordRef::LegacyPlane(v) if v.normal[2] >= 0.5 => {
                        for local in &v.members { map.ground.insert(nuwax_map_chunks::global_key(block_key, *local)?); }
                        map.ground_samples.extend(v.samples.iter().map(|v| v.point));
                    },
                    _ => {},
                }
            }
        }
        Ok(map)
    }

    /// Only use measured returns to fill an absent local ground classification.
    /// Existing ground always wins. The fit is local; it does not extrapolate an
    /// infinite floor across stairs, overhangs, unknown columns, or other storeys.
    pub fn add_ground_fallback(&mut self, measured: &[[f64; 3]]) {
        let fitted = local_ground_indices(measured);
        let present: HashSet<_> = self.keys.iter().copied().collect();
        let classified_xy: HashSet<_> = self.ground.iter().map(|k| [k[0],k[1]]).collect();
        for i in fitted {
            let p = measured[i]; let k = asset_key(p, self.voxel_size_m);
            if present.contains(&k) && !classified_xy.contains(&[k[0],k[1]]) {
                self.ground.insert(k); self.ground_samples.push(p);
            }
        }
    }

    pub fn preview(&self, settings: CleanupSettings, cancelled: impl Fn() -> bool) -> Result<CleanupPreview, String> {
        let tolerance = settings.gap_tolerance_m()?;
        if self.keys.len() != self.hits.len() { return Err("点云统计与原始键不匹配".into()); }
        let points: Vec<_> = self.keys.iter().map(|&k| center(k,self.voxel_size_m)).collect();
        let mut cells: HashMap<Key, Vec<usize>> = HashMap::new();
        let mut union = UnionFind::<usize>::new(points.len());
        for (i,&p) in points.iter().enumerate() {
            if i % 4096 == 0 && cancelled() { return Err("清理已取消，原图未改变".into()); }
            let k = grid(p,CONNECT_DISTANCE_M);
            for dx in -1..=1 { for dy in -1..=1 { for dz in -1..=1 {
                if let Some(ids) = cells.get(&[k[0]+dx,k[1]+dy,k[2]+dz]) {
                    for &j in ids { if distance2(p,points[j]) <= CONNECT_DISTANCE_M.powi(2) { union.union(i,j); } }
                }
            }}}
            cells.entry(k).or_default().push(i);
        }
        let mut groups: std::collections::BTreeMap<usize,Vec<usize>> = std::collections::BTreeMap::new();
        for i in 0..points.len() { groups.entry(union.find(i)).or_default().push(i); }
        let mut supported = HashSet::new();
        let mut grounded_roots = HashSet::new();
        for (&root,ids) in &groups {
            if ids.iter().any(|&i| self.ground.contains(&self.keys[i])) {
                supported.extend(ids.iter().copied()); grounded_roots.insert(root);
            }
        }
        // A plane/line is protected only when observed repeatedly and it forms
        // a genuine extended surface. Compact head/torso blobs are not anchors.
        let mut coherent_roots = HashSet::new();
        for (&root,ids) in &groups {
            if !grounded_roots.contains(&root) && coherent_structure(ids,&points,&self.hits) {
                coherent_roots.insert(root); supported.extend(ids.iter().copied());
            }
        }
        // Nearest support is a bounded spatial query, not N components × N points.
        let search = tolerance.max(CONNECT_DISTANCE_M) + f64::from(self.voxel_size_m);
        let mut support_index: HashMap<Key,Vec<usize>> = HashMap::new();
        for &i in &supported { support_index.entry(grid(points[i],search)).or_default().push(i); }
        let have_ground = !grounded_roots.is_empty();
        let mut components = Vec::new(); let mut removed_keys = Vec::new();
        for (root,ids) in groups {
            if cancelled() { return Err("清理已取消，原图未改变".into()); }
            let mut low = [f64::INFINITY;3]; let mut high = [f64::NEG_INFINITY;3];
            for &i in &ids { for a in 0..3 {low[a]=low[a].min(points[i][a]);high[a]=high[a].max(points[i][a]);} }
            let (support,gap) = if grounded_roots.contains(&root) {
                let ground_only = ids.iter().all(|&i| self.ground.contains(&self.keys[i]));
                (if ground_only {Support::MeasuredGround} else {Support::ConnectedStructure},Some(0.0))
            } else if coherent_roots.contains(&root) { (Support::CoherentStructure,Some(0.0)) }
            else if !have_ground { (Support::Unknown,None) }
            else {
                let mut d2 = f64::INFINITY;
                for &i in &ids {
                    let k=grid(points[i],search);
                    for dx in -1..=1 {for dy in -1..=1 {for dz in -1..=1 {
                        if let Some(js)=support_index.get(&[k[0]+dx,k[1]+dy,k[2]+dz]) {
                            for &j in js {d2=d2.min(distance2(points[i],points[j]));}
                        }
                    }}}
                }
                // Infinity means no support within the complete search radius,
                // not a fabricated distance. Keep the report's distance optional.
                (Support::Unsupported,d2.is_finite().then(||d2.sqrt()))
            };
            let selected = support == Support::Unsupported && gap.is_none_or(|g|g>tolerance);
            if selected { removed_keys.extend(ids.iter().map(|&i|self.keys[i])); }
            components.push(Component {points:ids.len(),bounds_min:low,bounds_max:high,support,gap_m:gap,selected});
        }
        removed_keys.sort_unstable();
        Ok(CleanupPreview {settings,gap_tolerance_m:tolerance,source_points:self.keys.len(),candidate_points:removed_keys.len(),
            measured_ground_points:self.ground.len(),components,removed_keys})
    }
}

fn coherent_structure(ids: &[usize], points: &[[f64;3]], hits: &[u16]) -> bool {
    if ids.len()<8 || ids.iter().filter(|&&i|hits[i]>=5).count()*5<ids.len()*4 { return false; }
    let mean=ids.iter().fold(Vector3::zeros(),|s,&i|s+Vector3::from(points[i]))/ids.len() as f64;
    let cov=ids.iter().fold(Matrix3::zeros(),|s,&i|{let d=Vector3::from(points[i])-mean;s+d*d.transpose()})/ids.len() as f64;
    let mut e=SymmetricEigen::new(cov).eigenvalues.as_slice().to_vec();e.sort_by(f64::total_cmp);
    // >= 0.8 m major extent, plus a true broad plane or a thin long line.
    // Repeated support alone cannot turn a small floating blob into a structure.
    e[2]>=0.8_f64.powi(2)/12.0 && (e[0]<=SURFACE_DISTANCE_M.powi(2) && e[1]>=0.4_f64.powi(2)/12.0
        || e[1]<=0.015_f64.powi(2) && e[2]>=1.2_f64.powi(2)/12.0)
}

/// Local lower-surface RANSAC from actual returns. Multiple levels are retained;
/// neighbour height is not copied into columns without measurements.
pub fn local_ground_indices(points: &[[f64;3]]) -> Vec<usize> {
    let mut patches: HashMap<[i32;2],Vec<usize>>=HashMap::new();
    for (i,p) in points.iter().enumerate() {if p.iter().all(|v|v.is_finite()) {
        patches.entry([(p[0]/0.5).floor() as i32,(p[1]/0.5).floor() as i32]).or_default().push(i);
    }}
    let mut result=Vec::new();
    for ids in patches.values() {
        if ids.len()<6 {continue;}
        let mut sorted=ids.clone();sorted.sort_by(|&a,&b|points[a][2].total_cmp(&points[b][2]));
        let low=points[sorted[0]][2];
        let seeds:Vec<_>=sorted.into_iter().filter(|&i|points[i][2]<=low+0.12).collect();
        if seeds.len()<6 {continue;}
        let stride=seeds.len().div_ceil(12).max(1);let sample:Vec<_>=seeds.iter().step_by(stride).copied().collect();
        let mut best:Vec<usize>=Vec::new();
        for a in 0..sample.len() {for b in a+1..sample.len() {for c in b+1..sample.len() {
            let p=Vector3::from(points[sample[a]]);let u=Vector3::from(points[sample[b]])-p;let v=Vector3::from(points[sample[c]])-p;
            let n=u.cross(&v);let area=n.norm();if area<0.012 {continue;} let n=n/area;
            if n.z.abs()<0.85 {continue;}
            let fit:Vec<_>=ids.iter().copied().filter(|&i|(Vector3::from(points[i])-p).dot(&n).abs()<=0.02).collect();
            if fit.len()>best.len(){best=fit;}
        }}}
        if best.len()>=6 {result.extend(best);}
    }
    result.sort_unstable();result.dedup();result
}

pub fn apply_fine(input: &Path, output: &Path, preview: &CleanupPreview) -> Result<usize,String> {
    if input==output {return Err("清理必须另存新图".into());}
    let mut reader=ArchiveReader::open(File::open(input).map_err(|e|e.to_string())?)?;
    let repository=Repository::temporary()?;
    let root=reader.import(&repository)?;
    if root.points() as usize!=preview.source_points {return Err("原图已变化，请重新预览".into());}
    let mut writer=Writer::new(repository,root.voxel_size_m(),32*1024*1024)?;writer.restore_archived(&root)?;
    writer.remove(preview.removed_keys.iter().copied())?;
    let sealed=writer.seal(root.domain().clone(),root.revision()+1,root.source_timestamp_ns())?;
    let count=sealed.manifest().points() as usize;
    if count+preview.candidate_points!=preview.source_points {return Err("实际删除数量与预览不一致".into());}
    let (mut archive,_)=nuwax_map_chunks::archive_file(sealed.manifest(),&sealed.repository)?;
    let mut out=std::fs::OpenOptions::new().write(true).create_new(true).open(output).map_err(|e|e.to_string())?;
    std::io::copy(&mut archive,&mut out).map_err(|e|e.to_string())?;out.sync_all().map_err(|e|e.to_string())?;
    Ok(count)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn sample() -> CompletedMap {
        let mut map=CompletedMap {voxel_size_m:0.05,keys:Vec::new(),hits:Vec::new(),ground:HashSet::new(),ground_samples:Vec::new()};
        for x in -10..=10 {for y in -10..=10 {let k=[x,y,0];map.keys.push(k);map.hits.push(10);map.ground.insert(k);}}
        // A wall/leg is genuinely connected to the measured floor.
        for z in 1..=30 {map.keys.push([0,0,z]);map.hits.push(10);}
        // A small unsupported ghost above the floor.
        for x in 5..=7 {for z in 15..=17 {map.keys.push([x,5,z]);map.hits.push(2);}}
        map
    }
    #[test] fn support_not_component_size_controls_cleanup() {
        let m=sample();let p=m.preview(CleanupSettings::default(),||false).unwrap();
        assert_eq!(p.candidate_points,9);assert!(p.removed_keys.iter().all(|k|k[0]>=5));
        assert!(p.components.iter().any(|c|c.support==Support::ConnectedStructure));
    }
    #[test] fn all_101_strengths_are_ordered_and_use_real_tolerances() {
        let m=sample();let mut previous=HashSet::new();let mut tolerance=1.0;
        for s in 0..=100 {let p=m.preview(CleanupSettings {strength_percent:s},||false).unwrap();
            let next:HashSet<_>=p.removed_keys.iter().copied().collect();assert!(previous.is_subset(&next));
            assert!(p.gap_tolerance_m<tolerance);tolerance=p.gap_tolerance_m;previous=next;}
    }
    #[test] fn absent_ground_is_unknown_not_blanket_delete() {
        let mut m=sample();m.ground.clear();let p=m.preview(CleanupSettings::default(),||false).unwrap();
        assert_eq!(p.candidate_points,0);assert!(p.components.iter().any(|c|c.support==Support::Unknown));
    }
    #[test] fn cancellation_has_no_output() {assert!(sample().preview(CleanupSettings::default(),||true).is_err());}
    #[test] fn unsupported_gap_changes_at_intermediate_percentages() {
        let mut m=sample();m.keys.push([8,8,4]);m.hits.push(1);
        let low=m.preview(CleanupSettings {strength_percent:20},||false).unwrap();
        let high=m.preview(CleanupSettings {strength_percent:80},||false).unwrap();
        assert!(!low.removed_keys.contains(&[8,8,4]));assert!(high.removed_keys.contains(&[8,8,4]));
    }
    #[test] fn asset_key_uses_the_stored_f32_quantum() {
        let q=0.05_f32;assert_eq!(asset_key([0.15,-0.15,1.0],q),[2,-3,19]);
    }
}
