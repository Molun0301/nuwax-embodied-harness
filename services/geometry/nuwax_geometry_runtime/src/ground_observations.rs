//! Capture-side physical-layer association. Only admitted original return indices
//! enter these bounded local fits; occupancy centres and replay resampling do not.
use nalgebra::{Matrix3, Point3, Vector3};
use nuwax_map_chunks::{
    block_key, global_key, local_key, Key, LayerObservation, LayerObservations, LayerSample, Writer,
    LayerSource, MAX_LAYERS_PER_BLOCK, MAX_LAYER_SAMPLES,
};
use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

const PATCH_SIDE_VOXELS: i32 = 4;
const MAX_PATCH_SAMPLES: usize = 192;
const MAX_PATCH_LAYERS: usize = 8;
const MIN_PLANE_INLIERS: usize = 6;
const HYPOTHESES: usize = 64;

#[derive(Clone)]
struct Return {
    point: [f64; 3],
    index: u32,
    key: Key,
}

/// Partition current admitted returns before forming moments. The existing owner
/// admits each scan once; source identity is retained with each selected return.
/// Missing/ambiguous fits leave legacy geometry in place and do not stop mapping.
pub(super) fn capture(
    writer: &mut Writer,
    points: &[Point3<f64>],
    sequence: u64,
    timestamp_ns: u64,
    source: Arc<LayerSource>,
    original_index: impl Fn(usize) -> Option<u32>,
    mut is_dynamic: impl FnMut(&Point3<f64>) -> bool,
) -> Result<Vec<u32>, String> {
    #[cfg(test)]
    let capture_started = std::time::Instant::now();
    let quantum = f64::from(writer.voxel_size_m());
    let residual = quantum * 0.2;
    let mut patches: BTreeMap<[i32; 2], Vec<Return>> = BTreeMap::new();
    let mut counts: BTreeMap<Key, usize> = BTreeMap::new();
    for (index, point) in points.iter().enumerate() {
        let Some(key) = super::voxel_key([point.x, point.y, point.z], quantum as f32) else {
            continue;
        };
        let key = [key.0, key.1, key.2];
        *counts.entry(key).or_default() += 1;
        if is_dynamic(point) {
            continue;
        }
        let index = original_index(index).ok_or("admitted original point index missing")?;
        patches
            .entry([
                key[0].div_euclid(PATCH_SIDE_VOXELS),
                key[1].div_euclid(PATCH_SIDE_VOXELS),
            ])
            .or_default()
            .push(Return {
                point: [point.x, point.y, point.z],
                index,
                key,
            });
    }
    // 先记录真实回波与旧平面冲突的具体成员，待本帧拟合后决定是否撤层。
    // 只有该格自身全部回波被同层候选解释时才可保留，邻格地面不能遮住当前障碍。
    let touched: BTreeSet<_> = counts.keys().map(|key| block_key(*key)).collect();
    let mut existing = BTreeMap::new();
    let mut claims = BTreeMap::new();
    for block in touched {
        let layers = writer.layer_observations(block)?;
        for (&anchor, layer) in &layers {
            for member in &layer.members {
                claims.insert((block, *member), anchor);
            }
        }
        existing.insert(block, layers);
    }
    let mut contradicted: BTreeMap<(Key, u16), BTreeSet<u16>> = BTreeMap::new();
    for point in points {
        let Some(key) = super::voxel_key([point.x, point.y, point.z], quantum as f32) else {
            continue;
        };
        let key = [key.0, key.1, key.2];
        let block = block_key(key);
        if let Some(anchor) = claims.get(&(block, local_key(key))) {
            let layer = &existing[&block][anchor];
            if distance(
                [point.x, point.y, point.z],
                layer.plane_origin,
                layer.normal,
            )
            .abs()
                > residual
                || is_dynamic(point)
            {
                contradicted.entry((block, *anchor)).or_default().insert(local_key(key));
            }
        }
    }
    // This is bounded by the admitted frame, not by accumulated map size.
    let mut updates: BTreeMap<Key, Vec<LayerObservation>> = BTreeMap::new();
    let mut current_ground = BTreeSet::new();
    for (patch, returns) in &patches {
        let mut classified = BTreeSet::new();
        // 分区仅用于有界检索，不能隔断同帧相邻真实回波的平面支撑。
        // 优先保留原小邻域结果；剩余成员再借一圈邻分区拟合，仍只分类中央原回波。
        for neighboring_support in [false, true] {
            if classified.len() == returns.len() {
                break;
            }
            let mut neighbors = Vec::new();
            if neighboring_support {
                for dx in -1..=1 {
                    for dy in -1..=1 {
                        if let Some(support) = patches.get(&[patch[0] + dx, patch[1] + dy]) {
                            neighbors.extend(support.iter().cloned());
                        }
                    }
                }
            }
            let fitting = if neighboring_support { neighbors.as_slice() } else { returns.as_slice() };
            // 只抽取真实原数组成员用于拟合，不插值或补造点；原数量/面积/残差门槛不变。
            let stride = fitting.len().div_ceil(MAX_PATCH_SAMPLES).max(1);
            let mut remaining: Vec<usize> = (0..fitting.len()).step_by(stride).collect();
            for _ in 0..MAX_PATCH_LAYERS {
                let Some((normal, origin, fit)) = fit_plane(fitting, &remaining, residual, quantum, *patch)
                else {
                    break;
                };
                let members: BTreeSet<usize> = fit.into_iter().collect();
                remaining.retain(|index| !members.contains(index));
                if normal[2] < 0.5 {
                    // 已解释为竖直表面的原回波继续占据，不能再借邻近地面拟成斜面。
                    classified.extend(returns.iter().filter(|point| {
                        distance(point.point, origin, normal).abs() <= residual
                    }).map(|point| point.index));
                    continue;
                }
                // Classification is evaluated against every actual return in the patch,
                // not only the fitting subset. Mixed floor/obstacle voxels stay occupied.
                let mut groups: BTreeMap<(Key, [i32; 2]), Vec<&Return>> = BTreeMap::new();
                let mut inlier_counts: BTreeMap<Key, usize> = BTreeMap::new();
                for point in returns.iter().filter(|p| {
                    !classified.contains(&p.index)
                        && distance(p.point, origin, normal).abs() <= residual
                }) {
                    *inlier_counts.entry(point.key).or_default() += 1;
                    groups
                        .entry((block_key(point.key), [point.key[0], point.key[1]]))
                        .or_default()
                        .push(point);
                }
                for points in groups.values() {
                    classified.extend(points.iter().map(|p| p.index));
                }
                for ((block, xy), points) in groups {
                    let safe: Vec<_> = points
                        .into_iter()
                        .filter(|p| inlier_counts[&p.key] == counts[&p.key])
                        .collect();
                    if safe.is_empty() {
                        continue;
                    }
                    current_ground.extend(safe.iter().map(|point| point.index));
                    let members: Vec<_> = safe
                        .iter()
                        .map(|p| local_key(p.key))
                        .collect::<BTreeSet<_>>()
                        .into_iter()
                        .collect();
                    let stride = safe.len().div_ceil(MAX_LAYER_SAMPLES).max(1);
                    let samples = safe
                        .into_iter()
                        .step_by(stride)
                        .take(MAX_LAYER_SAMPLES)
                        .map(|p| LayerSample {
                            point: p.point,
                            source: Arc::clone(&source),
                            point_index: p.index,
                            source_sequence: sequence,
                            source_timestamp_ns: timestamp_ns,
                        })
                        .collect();
                    let plane_xy = [
                        (f64::from(xy[0]) + 0.5) * quantum,
                        (f64::from(xy[1]) + 0.5) * quantum,
                    ];
                    let z = origin[2]
                        - (normal[0] * (plane_xy[0] - origin[0])
                            + normal[1] * (plane_xy[1] - origin[1]))
                            / normal[2];
                    updates.entry(block).or_default().push(LayerObservation {
                        members,
                        normal,
                        plane_origin: [plane_xy[0], plane_xy[1], z],
                        samples,
                    });
                }
            }
        }
    }
    let mut revoked_blocks = BTreeSet::new();
    for ((block, anchor), members) in contradicted {
        let old = &existing[&block][&anchor];
        // 候选成员已经通过本格全部真实点的一致性检查；还必须沿原高度/法向判据
        // 与该旧层关联。未分类、混格障碍、垂墙和动态点均不能借其它候选保层。
        let explained = updates.get(&block).is_some_and(|candidates| members.iter().all(|member|
            candidates.iter().any(|candidate| candidate.members.contains(member)
                && same_physical_layer(old, candidate, residual, block, quantum as f32))));
        if !explained {
            existing.get_mut(&block).unwrap().remove(&anchor);
            revoked_blocks.insert(block);
        }
    }
    #[cfg(test)]
    if std::thread::current().name().is_some_and(|name| name.contains("pgo_capture_cost")) {
        eprintln!("CAPTURE_FRAME touched={} patches={} eligible_patches={} fit_points={} old_layers={} candidates={} elapsed_before_replace_us={}",
            existing.len(), patches.len(), patches.values().filter(|p|p.len()>=MIN_PLANE_INLIERS).count(),
            patches.values().map(|p|p.len().div_ceil(p.len().div_ceil(MAX_PATCH_SAMPLES).max(1))).sum::<usize>(),
            existing.values().map(|l|l.len()).sum::<usize>(), updates.values().map(|l|l.len()).sum::<usize>(), capture_started.elapsed().as_micros());
    }
    for (key, mut layers) in existing {
        let candidates = updates.remove(&key).unwrap_or_default();
        // Unchanged observations need no dirty occupancy clone or full-block
        // validation. A revoked last layer must still write the empty result.
        if candidates.is_empty() && !revoked_blocks.contains(&key) { continue; }
        for candidate in candidates {
            associate(&mut layers, candidate, residual, key, quantum as f32);
        }
        writer.replace_layer_observations(key, layers)?;
    }
    Ok(current_ground.into_iter().collect())
}

fn distance(point: [f64; 3], origin: [f64; 3], normal: [f64; 3]) -> f64 {
    (0..3).map(|i| (point[i] - origin[i]) * normal[i]).sum()
}

/// 有界的多假设拟合先分离实际平面；竖直平面也需识别，交由捕获方保留占据。
/// 残差按平面垂距计算，斜坡不按高度分箱，邻区墙点也不混进地面斜拟合。
fn fit_plane(
    points: &[Return],
    available: &[usize],
    threshold: f64,
    quantum: f64,
    focus_patch: [i32; 2],
) -> Option<([f64; 3], [f64; 3], Vec<usize>)> {
    if available.len() < MIN_PLANE_INLIERS {
        return None;
    }
    let all_mean = available
        .iter()
        .map(|i| Vector3::from(points[*i].point))
        .sum::<Vector3<f64>>()
        / available.len() as f64;
    let all_scatter = available.iter().fold(Matrix3::zeros(), |sum, i| {
        let d = Vector3::from(points[*i].point) - all_mean;
        sum + d * d.transpose()
    });
    let all_eigen = all_scatter.symmetric_eigen();
    let mut axes = [0, 1, 2];
    axes.sort_by(|a, b| all_eigen.eigenvalues[*a].total_cmp(&all_eigen.eigenvalues[*b]));
    if all_eigen.eigenvalues[axes[1]] < quantum * quantum * 0.01 {
        return None;
    }
    let mut all_normal = all_eigen.eigenvectors.column(axes[0]).into_owned();
    if all_normal.z < 0.0 {
        all_normal = -all_normal;
    }
    if available
        .iter()
        .all(|i| distance(points[*i].point, all_mean.into(), all_normal.into()).abs() <= threshold)
    {
        return Some((all_normal.into(), all_mean.into(), available.to_vec()));
    }
    let mut best = Vec::new();
    let mut best_score = (0, 0);
    let mut best_plane: Option<([f64; 3], [f64; 3])> = None;
    let support_score = |indices: &[usize]| {
        let local = indices.iter().filter(|&&index| {
            let key = points[index].key;
            [key[0].div_euclid(PATCH_SIDE_VOXELS), key[1].div_euclid(PATCH_SIDE_VOXELS)] == focus_patch
        }).count();
        (local, indices.len())
    };
    let mut state = 0x9e37_79b9_u64;
    for _ in 0..HYPOTHESES {
        // Fixed PRNG is only hypothesis sampling, never an identity/content hash.
        let mut pick = || {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
            available[((state >> 32) as usize) % available.len()]
        };
        let a = Vector3::from(points[pick()].point);
        let b = Vector3::from(points[pick()].point);
        let c = Vector3::from(points[pick()].point);
        let mut normal = (b - a).cross(&(c - a));
        let norm = normal.norm();
        if norm < quantum * quantum * 0.01 {
            continue;
        }
        normal /= norm;
        if normal.z < 0.0 {
            normal = -normal;
        }
        let inliers: Vec<_> = available
            .iter()
            .copied()
            .filter(|i| distance(points[*i].point, a.into(), normal.into()).abs() <= threshold)
            .collect();
        // 邻域包含真实阶边时，优先解释中央分区自己的回波，再比较邻域支撑数。
        // 仅按邻域总数排序会把两级踏面各取一排，拟成并不存在的斜面。
        let score = support_score(&inliers);
        if inliers.len() >= MIN_PLANE_INLIERS && score > best_score {
            best = inliers;
            best_score = score;
            best_plane = Some((normal.into(), a.into()));
        }
    }
    if best.len() < MIN_PLANE_INLIERS {
        return None;
    }
    let mean = best
        .iter()
        .map(|i| Vector3::from(points[*i].point))
        .sum::<Vector3<f64>>()
        / best.len() as f64;
    let scatter = best.iter().fold(Matrix3::zeros(), |sum, i| {
        let d = Vector3::from(points[*i].point) - mean;
        sum + d * d.transpose()
    });
    let eigen = scatter.symmetric_eigen();
    let mut order = [0, 1, 2];
    order.sort_by(|a, b| eigen.eigenvalues[*a].total_cmp(&eigen.eigenvalues[*b]));
    // Reject line-like support: a row of points does not establish a local plane.
    if eigen.eigenvalues[order[1]] < quantum * quantum * 0.01 {
        return None;
    }
    let mut normal = eigen.eigenvectors.column(order[0]).into_owned();
    if normal.z < 0.0 {
        normal = -normal;
    }
    let inliers: Vec<_> = available
        .iter()
        .copied()
        .filter(|i| distance(points[*i].point, mean.into(), normal.into()).abs() <= threshold)
        .collect();
    let refined_is_better = inliers.len() >= MIN_PLANE_INLIERS && support_score(&inliers) >= best_score;
    #[cfg(test)]
    tests::record_fit_choice(points, focus_patch, &best, &inliers, refined_is_better);
    // 细化仍以原中央/总内点排序验收，不能以邻区点替换已解释的中央真实回波。
    // 原假设已通过同一残差、数量及上述二维面积检查，保留它不放宽任何门槛。
    if refined_is_better { Some((normal.into(), mean.into(), inliers)) }
    else { best_plane.map(|(normal, origin)| (normal, origin, best)) }
}

/// 捕获反证复核与实际关联共用原同层判据，不能形成两套不同的放行范围。
fn same_physical_layer(old: &LayerObservation, candidate: &LayerObservation, threshold: f64,
    block: Key, voxel_size: f32) -> bool {
    let Ok(center) = candidate.representative() else { return false; };
    let same_normal = old.normal.iter().zip(candidate.normal).map(|(a, b)| a * b).sum::<f64>() > 0.98;
    if old.members[0] % 1024 != candidate.members[0] % 1024 || !same_normal {
        return false;
    }
    let Some(height) = old.height_at([center[0], center[1]]) else { return false; };
    if (height - center[2]).abs() <= threshold { return true; }
    // 旧拟合输出可以落后于其保留的原样本。补充分支只让本帧安全平面解释全部
    // 真实旧样本，并逐一核对旧成员有同块同格的样本；不抬高度/垂距/法向门槛。
    !old.samples.is_empty()
        && old.samples.iter().all(|sample|
            distance(sample.point, center, candidate.normal).abs() <= threshold)
        && old.members.iter().all(|member| {
            let Ok(expected) = global_key(block, *member) else { return false; };
            old.samples.iter().any(|sample|
                super::voxel_key(sample.point, voxel_size).map(|key| [key.0, key.1, key.2]) == Some(expected))
        })
}

fn associate(layers: &mut LayerObservations, candidate: LayerObservation, threshold: f64,
    block: Key, voxel_size: f32) {
    let Ok(center) = candidate.representative() else {
        return;
    };
    let column = candidate.members[0] % 1024;
    let mut overlapping = Vec::new();
    let mut same = None;
    // Validated layer anchors are x + 32*y + 1024*z, z in 0..32.
    // Preserve original BTreeMap order within exactly the candidate's XY.
    let levels = layers.first_key_value().zip(layers.last_key_value())
        .map(|((&first, _), (&last, _))| (first / 1024, last / 1024));
    let mut visit = |key, old: &LayerObservation| {
        if old.members.iter().any(|m| candidate.members.contains(m)) {
            overlapping.push(key);
        }
        if same.is_none() && same_physical_layer(old, &candidate, threshold, block, voxel_size) {
            same = Some(key);
        }
    };
    if let Some((first, last)) = levels {
        // A sparse, tall block may contain fewer records than candidate Z keys.
        // Choose the smaller enumeration; both retain the same ascending order.
        if layers.len() < usize::from(last - first + 1) {
            for (&key, old) in layers.iter().filter(|(key, _)| **key % 1024 == column) {
                visit(key, old);
            }
        } else {
            for z in first..=last {
                let key = column + z * 1024;
                if let Some(old) = layers.get(&key) { visit(key, old); }
            }
        }
    }
    // Ambiguous claims are withdrawn rather than merging unrelated physical layers.
    for key in &overlapping {
        if Some(*key) != same {
            layers.remove(key);
        }
    }
    let mut result = candidate;
    if let Some(key) = same {
        let old = layers.remove(&key).unwrap();
        // Retain the prior fitted output only inside half the fitting residual
        // bound (0.1 source voxel). Compare to the old fixed output, not the last
        // input: persistent motion accumulates and eventually crosses this bound.
        // Normal drift is measured as height at one patch radius, also in metres.
        let normal_shift = (old.normal[0] / old.normal[2] - result.normal[0] / result.normal[2])
            .abs()
            + (old.normal[1] / old.normal[2] - result.normal[1] / result.normal[2]).abs();
        let stability = threshold * 0.5;
        if (old.plane_origin[2] - result.plane_origin[2]).abs() <= stability
            && normal_shift * (threshold / 0.2) * f64::from(PATCH_SIDE_VOXELS) <= stability
        {
            result.plane_origin = old.plane_origin;
            result.normal = old.normal;
        }
        let current_source = Arc::clone(&result.samples[0].source);
        let mut samples = old.samples;
        for sample in result.samples {
            if !samples.iter().any(|old| {
                old.source == sample.source
                    && old.source_sequence == sample.source_sequence
                    && old.source_timestamp_ns == sample.source_timestamp_ns
                    && old.point_index == sample.point_index
            }) {
                samples.push(sample);
            }
        }
        // Retained inliers are rechecked against the newly fitted plane. Old ground
        // is not expired by a clock; only new geometric evidence changes this patch.
        samples.retain(|p| distance(p.point, center, result.normal).abs() <= threshold);
        retain_bounded_sources(&mut samples, &current_source);
        result.samples = samples;
        result.members.extend(old.members);
        result.members.sort_unstable();
        result.members.dedup();
    }
    if !result.samples.is_empty()
        && result.members.len() <= nuwax_map_chunks::MAX_LAYER_MEMBERS
        && (layers.len() < MAX_LAYERS_PER_BLOCK || layers.contains_key(&result.members[0]))
    {
        layers.insert(result.members[0], result);
    }
}


/// 各真实来源内部按采集顺序取新样本，再按来源轮流保留；不同设备时钟不可比大小。
/// 输入最多是旧层与本帧各 32 个返回；只选原数组下标，不复制点或增加历史表。
fn retain_bounded_sources(samples: &mut Vec<LayerSample>, current: &LayerSource) {
    samples.sort_by(|a, b| (&a.source, a.source_timestamp_ns, a.source_sequence, a.point_index)
        .cmp(&(&b.source, b.source_timestamp_ns, b.source_sequence, b.point_index)));
    if samples.len() <= MAX_LAYER_SAMPLES { return; }
    let mut groups = Vec::new();
    let mut start = 0;
    while start < samples.len() {
        let end = start + samples[start..].iter().take_while(|sample| sample.source == samples[start].source).count();
        groups.push((start, end));
        start = end;
    }
    // 来源数本身超过样本预算时，当前真实观测仍有一个位置；其余按稳定来源顺序选择。
    if let Some(index) = groups.iter().position(|(start, _)| samples[*start].source.as_ref() == current) {
        groups.rotate_left(index);
    }
    let mut selected = vec![false; samples.len()];
    let mut count = 0;
    while count < MAX_LAYER_SAMPLES {
        for (start, end) in &mut groups {
            if *end > *start {
                *end -= 1;
                selected[*end] = true;
                count += 1;
                if count == MAX_LAYER_SAMPLES { break; }
            }
        }
    }
    let mut index = 0;
    samples.retain(|_| { let keep = selected[index]; index += 1; keep });
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_map_chunks::{Domain, Repository};
    std::thread_local! {
        static FIT_CHOICES: std::cell::RefCell<Option<([i32; 2], Vec<serde_json::Value>)>> = const {
            std::cell::RefCell::new(None)
        };
    }
    pub(super) fn record_fit_choice(points: &[Return], focus: [i32; 2], selected: &[usize], refined: &[usize], accepted: bool) {
        FIT_CHOICES.with(|trace| {
            let mut trace = trace.borrow_mut();
            let Some((requested, rows)) = trace.as_mut() else { return; };
            if *requested != focus { return; }
            let local = |indices: &[usize]| indices.iter().filter(|&&index| {
                let key = points[index].key;
                [key[0].div_euclid(PATCH_SIDE_VOXELS), key[1].div_euclid(PATCH_SIDE_VOXELS)] == focus
            }).map(|&index| points[index].index).collect::<Vec<_>>();
            rows.push(serde_json::json!({"selected_indices":selected.iter().map(|i|points[*i].index).collect::<Vec<_>>(),
                "selected_central_indices":local(selected),"refined_indices":refined.iter().map(|i|points[*i].index).collect::<Vec<_>>(),
                "refined_central_indices":local(refined),"accepted_refinement":accepted}));
        });
    }
    fn associate_full_block_reference(layers: &mut LayerObservations, candidate: LayerObservation, threshold: f64,
        block: Key, voxel_size: f32) {
        let Ok(center) = candidate.representative() else {
            return;
        };
        let overlapping: Vec<_> = layers
            .iter()
            .filter(|(_, old)| old.members.iter().any(|m| candidate.members.contains(m)))
            .map(|(key, _)| *key)
            .collect();
        let same = layers
            .iter()
            .find(|(_, old)| same_physical_layer(old, &candidate, threshold, block, voxel_size))
            .map(|(key, _)| *key);
        // Ambiguous claims are withdrawn rather than merging unrelated physical layers.
        for key in &overlapping {
            if Some(*key) != same {
                layers.remove(key);
            }
        }
        let mut result = candidate;
        if let Some(key) = same {
            let old = layers.remove(&key).unwrap();
            // Retain the prior fitted output only inside half the fitting residual
            // bound (0.1 source voxel). Compare to the old fixed output, not the last
            // input: persistent motion accumulates and eventually crosses this bound.
            // Normal drift is measured as height at one patch radius, also in metres.
            let normal_shift = (old.normal[0] / old.normal[2] - result.normal[0] / result.normal[2])
                .abs()
                + (old.normal[1] / old.normal[2] - result.normal[1] / result.normal[2]).abs();
            let stability = threshold * 0.5;
            if (old.plane_origin[2] - result.plane_origin[2]).abs() <= stability
                && normal_shift * (threshold / 0.2) * f64::from(PATCH_SIDE_VOXELS) <= stability
            {
                result.plane_origin = old.plane_origin;
                result.normal = old.normal;
            }
            let current_source = Arc::clone(&result.samples[0].source);
            let mut samples = old.samples;
            for sample in result.samples {
                if !samples.iter().any(|old| {
                    old.source == sample.source
                        && old.source_sequence == sample.source_sequence
                        && old.source_timestamp_ns == sample.source_timestamp_ns
                        && old.point_index == sample.point_index
                }) {
                    samples.push(sample);
                }
            }
            // Retained inliers are rechecked against the newly fitted plane. Old ground
            // is not expired by a clock; only new geometric evidence changes this patch.
            samples.retain(|p| distance(p.point, center, result.normal).abs() <= threshold);
            retain_bounded_sources(&mut samples, &current_source);
            result.samples = samples;
            result.members.extend(old.members);
            result.members.sort_unstable();
            result.members.dedup();
        }
        if !result.samples.is_empty()
            && result.members.len() <= nuwax_map_chunks::MAX_LAYER_MEMBERS
            && (layers.len() < MAX_LAYERS_PER_BLOCK || layers.contains_key(&result.members[0]))
        {
            layers.insert(result.members[0], result);
        }
    }


    fn new_writer() -> Writer {
        Writer::new(Repository::temporary().unwrap(), 0.05, 4 * 1024 * 1024).unwrap()
    }

    #[test]
    fn same_column_association_preserves_full_block_results_and_source_samples() {
        let quantum = 0.05_f32;
        let threshold = f64::from(quantum) * 0.2;
        for (columns, scans_existing) in [(1_u16, true), (8, true), (96, false)] {
            let mut actual = LayerObservations::new();
            for z in [0_u16, 4, 31] { for column in 0..columns {
                let anchor = column + 1024 * z;
                let point = [(f64::from(column % 32) + 0.5) * f64::from(quantum),
                    (f64::from(column / 32) + 0.5) * f64::from(quantum),
                    (f64::from(z) + 0.5) * f64::from(quantum)];
                actual.insert(anchor, LayerObservation {
                    members: vec![anchor], plane_origin: point, normal: [0.0, 0.0, 1.0],
                    samples: vec![LayerSample { point, source: Arc::new(LayerSource::Legacy),
                        point_index: u32::from(anchor), source_sequence: 1, source_timestamp_ns: 100 }],
                });
            }}
            let first = actual.first_key_value().unwrap().0 / 1024;
            let last = actual.last_key_value().unwrap().0 / 1024;
            assert_eq!(actual.len() < usize::from(last - first + 1), scans_existing,
                "exercise both existing-record and bounded-key enumeration");
            let mut expected = actual.clone();
            let originals: Vec<_> = actual.values().cloned().collect();
            for round in 0..4_u64 { for (index, original) in originals.iter().enumerate() {
                let mut candidate = original.clone();
                let shift = [0.004, -0.004, 0.006, -0.006][round as usize];
                candidate.plane_origin[2] += shift;
                if round == 2 && index % 7 == 0 { candidate.normal = [0.6, 0.0, 0.8]; }
                candidate.samples[0].point = candidate.plane_origin;
                candidate.samples[0].source_sequence = round + 2;
                candidate.samples[0].source_timestamp_ns = 200 + round;
                candidate.samples[0].source = Arc::new(if index % 2 == 0 {
                    LayerSource::Lidar { backend: 1, runtime_generation: 1, sensor_frame_id: "lidar".into() }
                } else {
                    LayerSource::Auxiliary { sensor_id: "aux".into(), clock_id: "clock".into(),
                        runtime_generation: 1, session_generation: 1 }
                });
                associate_full_block_reference(&mut expected, candidate.clone(), threshold, [0; 3], quantum);
                associate(&mut actual, candidate, threshold, [0; 3], quantum);
                assert_eq!(actual, expected, "columns={columns} round={round} index={index}");
            }}
        }
    }
    fn ingest(writer: &mut Writer, points: &[Point3<f64>], stamp: u64) {
        let keys = points
            .iter()
            .map(|p| super::super::voxel_key([p.x, p.y, p.z], 0.05).unwrap())
            .map(|k| [k.0, k.1, k.2]);
        writer.observe(keys, stamp).unwrap();
        capture(writer, points, stamp, stamp, Arc::new(LayerSource::Legacy), |index| u32::try_from(index).ok(), |_| false).unwrap();
    }
    fn grid(z: impl Fn(f64, f64) -> f64) -> Vec<Point3<f64>> {
        (0..8)
            .flat_map(|x| (0..8).map(move |y| (0.008 + x as f64 * 0.022, 0.008 + y as f64 * 0.022)))
            .map(|(x, y)| Point3::new(x, y, z(x, y)))
            .collect()
    }
    fn domain() -> Domain {
        Domain {
            session: "layers".into(),
            generation: 1,
            frame_generation: 1,
            producer_generation: 1,
        }
    }
    fn sparse_patch_grid(height: impl Fn(f64, f64) -> f64) -> Vec<Point3<f64>> {
        (0..4).flat_map(|x| (0..4).map(move |y| (0.025 + x as f64 * 0.11, 0.025 + y as f64 * 0.11)))
            .map(|(x, y)| Point3::new(x, y, height(x, y))).collect()
    }

    fn residual_conflict_fixture() -> (Writer, Vec<Point3<f64>>) {
        let mut writer = new_writer();
        let low = [0, 0, 0];
        let high = [0, 0, 1];
        writer.observe([low, high], 1).unwrap();
        let source = Arc::new(LayerSource::Legacy);
        let old = LayerObservation {
            members: vec![local_key(low), local_key(high)],
            plane_origin: [0.025, 0.025, 0.045],
            normal: [0.0, 0.0, 1.0],
            samples: [([0.020, 0.020, 0.044], 0), ([0.030, 0.030, 0.051], 1)]
                .into_iter().map(|(point, point_index)| LayerSample {
                    point, source: Arc::clone(&source), source_sequence: 1,
                    source_timestamp_ns: 1, point_index,
                }).collect(),
        };
        writer.replace_layer_observations([0, 0, 0], [(local_key(low), old)].into()).unwrap();
        let points = (0..3).flat_map(|x| (0..3).map(move |y| Point3::new(
            0.025 + f64::from(x) * 0.06, 0.025 + f64::from(y) * 0.06,
            if x == 0 && y == 0 { 0.0565 } else { 0.0505 },
        ))).collect();
        (writer, points)
    }

    #[test]
    fn same_frame_compatible_fit_retains_unobserved_member_after_residual_conflict() {
        let (mut writer, points) = residual_conflict_fixture();
        // 单点偏离旧面11.5mm；本帧真实二维邻域仍解释为原10mm关联范围内的同一层。
        assert!(points[0].z - 0.045 > 0.01);
        ingest(&mut writer, &points, 2);
        let layers = writer.layer_observations([0, 0, 0]).unwrap();
        let layer = layers.values().find(|layer| layer.members.contains(&local_key([0, 0, 1])))
            .expect("本帧真实上格仍应被拟合");
        assert!(layer.members.contains(&local_key([0, 0, 0])),
            "同帧同层拟合解释了冲突时，未删除的旧成员不能先于关联被整层撤销");
        assert!(layer.samples.iter().any(|sample| sample.source_sequence == 1));
        assert!(layer.samples.iter().any(|sample| sample.source_sequence == 2));
        assert_eq!(writer.region([0, 0, 0], [0, 0, 0], 1).unwrap().len(), 1);
    }

    #[test]
    fn neighboring_ground_cannot_protect_unclassified_or_dynamic_member_conflict() {
        for dynamic in [false, true] {
            let (mut writer, mut points) = residual_conflict_fixture();
            if !dynamic {
                // 与待解释地面同一实际占据格内加入低障碍；邻格地面不能为它取得豁免。
                points.push(Point3::new(0.025, 0.025, 0.090));
            }
            let keys = points.iter().map(|point| {
                let key = super::super::voxel_key([point.x, point.y, point.z], 0.05).unwrap();
                [key.0, key.1, key.2]
            }).collect::<Vec<_>>();
            writer.observe(keys, 2).unwrap();
            let ground = capture(&mut writer, &points, 2, 2, Arc::new(LayerSource::Legacy),
                |index| Some(index as u32), |point| dynamic && point == &points[0]).unwrap();
            assert!(ground.iter().any(|index| *index != 0), "其它真实地面仍有足够同帧支撑");
            assert!(!ground.contains(&0), "受冲突格不能借邻格地面取得当前索引");
            let layers = writer.layer_observations([0, 0, 0]).unwrap();
            assert!(layers.values().all(|layer| !layer.members.contains(&local_key([0, 0, 0]))
                && !layer.members.contains(&local_key([0, 0, 1]))), "未解释或动态冲突仍须撤销旧层");
            assert_eq!(writer.region([0, 0, 1], [0, 0, 1], 1).unwrap().len(), 1);
        }
    }

    fn stale_fit_association_fixture() -> (LayerObservation, LayerObservation) {
        // 完整住宅粗糙面实录：旧第10/23/26帧与第41帧新拟合，全部保留原始索引。
        let source = Arc::new(LayerSource::Lidar {
            backend: 2, runtime_generation: 1, sensor_frame_id: "go2_imu_base_link".into(),
        });
        let old = LayerObservation {
            members: vec![local_key([-7, -2, -8])],
            plane_origin: [-0.3250000048428774, -0.07500000111758709, -0.36259337107331147],
            normal: [-0.016630129166888608, -0.008126092321413033, 0.9998286880398444],
            samples: [
                ([-0.34740057672360486, -0.08485589392934524, -0.36067890630831445], 10, 1792229850048540831, 723),
                ([-0.33860610369310923, -0.08864349913713766, -0.35629224398017423], 23, 1792229851348540783, 672),
                ([-0.34272246419851005, -0.06355862867777072, -0.3524160550780994], 26, 1792229851648540974, 528),
            ].into_iter().map(|(point, source_sequence, source_timestamp_ns, point_index)| LayerSample {
                point, source: Arc::clone(&source), source_sequence, source_timestamp_ns, point_index,
            }).collect(),
        };
        let current = LayerObservation {
            members: vec![local_key([-7, -2, -7])],
            plane_origin: [-0.3250000048428774, -0.07500000111758709, -0.3524112363379468],
            normal: [-0.02914604546786826, -0.019061645668827995, 0.999393396865309],
            samples: vec![LayerSample {
                point: [-0.34888872650451347, -0.06929202407318469, -0.3494896926943463],
                source, source_sequence: 41, source_timestamp_ns: 1792229853148540974, point_index: 98,
            }],
        };
        (old, current)
    }

    #[test]
    fn original_samples_reconcile_stale_fit_without_widening_height_tolerance() {
        let (old, current) = stale_fit_association_fixture();
        let residual = f64::from(0.05_f32) * 0.2;
        assert!((old.plane_origin[2] - current.plane_origin[2]).abs() > residual);
        assert!(old.samples.iter().all(|sample|
            distance(sample.point, current.plane_origin, current.normal).abs() <= residual));
        let source_identities = old.samples.iter().chain(&current.samples)
            .map(|sample| (sample.source_sequence, sample.point_index, sample.point)).collect::<Vec<_>>();
        let mut layers = [(old.members[0], old)].into();
        associate(&mut layers, current, residual, [-1, -1, -1], 0.05);
        assert_eq!(layers.len(), 1, "旧拟合输出过时但全部真实样本同层时，不能制造两条独立地面");
        let layer = layers.values().next().unwrap();
        assert_eq!(layer.members, vec![local_key([-7, -2, -8]), local_key([-7, -2, -7])]);
        assert_eq!(layer.samples.iter().map(|sample|
            (sample.source_sequence, sample.point_index, sample.point)).collect::<Vec<_>>(), source_identities);
    }

    #[test]
    fn sample_supported_association_requires_each_member_and_original_normal_gate() {
        for contradiction in ["missing_member", "foreign_block", "different_normal", "different_floor"] {
            let (mut old, mut current) = stale_fit_association_fixture();
            let residual = f64::from(0.05_f32) * 0.2;
            match contradiction {
                "missing_member" => {
                    old.members.insert(0, local_key([-7, -2, -9]));
                }
                "foreign_block" => {
                    // 局部格编号相同且仍在新面残差内，但原样本来自另一全局块，不能充数。
                    let block_side = f64::from(0.05_f32) * 32.0;
                    for sample in &mut old.samples {
                        sample.point[0] += 2.0 * block_side;
                        sample.point[1] -= 3.0 * block_side;
                    }
                    assert!(old.samples.iter().all(|sample|
                        distance(sample.point, current.plane_origin, current.normal).abs() <= residual));
                }
                "different_normal" => current.normal = [0.25, 0.0, (1.0_f64 - 0.25 * 0.25).sqrt()],
                "different_floor" => current.plane_origin[2] += 0.2,
                _ => unreachable!(),
            }
            let mut layers = [(old.members[0], old)].into();
            associate(&mut layers, current, residual, [-1, -1, -1], 0.05);
            assert_eq!(layers.len(), 2, "补充分支不能越过证据边界：{contradiction}");
        }
    }

    #[test]
    fn normal_height_match_keeps_original_bounded_sample_semantics() {
        let (mut old, mut current) = stale_fit_association_fixture();
        old.members.insert(0, local_key([-7, -2, -9]));
        current.plane_origin[2] = old.plane_origin[2] + 0.005;
        let mut layers = [(old.members[0], old)].into();
        associate(&mut layers, current, f64::from(0.05_f32) * 0.2, [-1, -1, -1], 0.05);
        assert_eq!(layers.len(), 1, "原高度/法向匹配分支不能附加补充分支的逐成员样本门槛");
        assert!(layers.values().next().unwrap().members.contains(&local_key([-7, -2, -9])));
    }

    #[test]
    fn complete_flat_block_keeps_every_classified_column_persistent() {
        let mut writer = new_writer();
        let q = f64::from(writer.voxel_size_m());
        let points: Vec<_> = (0..32).flat_map(|x| (0..32).map(move |y|
            Point3::new((f64::from(x) + 0.5) * q, (f64::from(y) + 0.5) * q, 0.01)))
            .collect();
        let keys: Vec<_> = points.iter().map(|point| {
            let k = super::super::voxel_key([point.x, point.y, point.z], q as f32).unwrap();
            [k.0, k.1, k.2]
        }).collect();
        writer.observe(keys.iter().copied(), 10).unwrap();
        let ground = capture(&mut writer, &points, 1, 10, Arc::new(LayerSource::Legacy),
            |index| Some(index as u32), |_| false).unwrap();
        assert_eq!(ground.len(), 1024);
        let layers = writer.layer_observations([0, 0, 0]).unwrap();
        let members: BTreeSet<_> = layers.values().flat_map(|layer| layer.members.iter().copied()).collect();
        assert_eq!(members, keys.iter().map(|key| local_key(*key)).collect(),
            "同帧已正确分类的完整平地必须全部持久化，不能只保存前256列");
        let saved = writer.snapshot(domain(), 1, 10).unwrap();
        assert_eq!(saved.blocks.len(), 1);
    }

    #[test]
    fn sparse_same_frame_neighbors_establish_ground_without_fabricating_members() {
        let mut writer = new_writer();
        let points = sparse_patch_grid(|x, y| 0.01 + 0.1 * x + 0.05 * y);
        let keys: Vec<_> = points.iter().map(|point| {
            let key = super::super::voxel_key([point.x, point.y, point.z], 0.05).unwrap();
            [key.0, key.1, key.2]
        }).collect();
        writer.observe(keys.iter().copied(), 10).unwrap();
        let ground = capture(&mut writer, &points, 7, 10, Arc::new(LayerSource::Legacy),
            |index| Some(1000 + index as u32), |_| false).unwrap();
        assert_eq!(ground, (1000..1016).collect::<Vec<_>>(), "每分区仅4点，仍应由同帧真实二维邻域建立平面");
        let layers = writer.layer_observations([0, 0, 0]).unwrap();
        let claimed: BTreeSet<_> = layers.values().flat_map(|layer| layer.members.iter().copied()).collect();
        assert_eq!(claimed, keys.iter().map(|key| local_key(*key)).collect());
        for layer in layers.values() {
            assert!((layer.plane_origin[2] - (0.01 + 0.1 * layer.plane_origin[0] + 0.05 * layer.plane_origin[1])).abs() < 1e-8);
            for sample in &layer.samples {
                assert_eq!(sample.source_sequence, 7);
                assert_eq!(sample.source_timestamp_ns, 10);
                let original = points[(sample.point_index - 1000) as usize];
                assert_eq!(sample.point, [original.x, original.y, original.z]);
            }
        }
    }

    #[test]
    fn neighboring_support_retains_mixed_voxel_obstacle_and_vertical_wall() {
        let mut writer = new_writer();
        let mut points = sparse_patch_grid(|_, _| 0.01);
        let obstacle = points.len();
        points.push(Point3::new(0.025, 0.025, 0.045));
        let wall_start = points.len();
        for y in 0..4 {
            for z in 0..4 {
                points.push(Point3::new(0.51, 0.025 + y as f64 * 0.11, 0.12 + z as f64 * 0.11));
            }
        }
        let keys: Vec<_> = points.iter().map(|point| {
            let key = super::super::voxel_key([point.x, point.y, point.z], 0.05).unwrap();
            [key.0, key.1, key.2]
        }).collect();
        writer.observe(keys.iter().copied(), 10).unwrap();
        let ground = capture(&mut writer, &points, 1, 10, Arc::new(LayerSource::Legacy),
            |index| Some(index as u32), |_| false).unwrap();
        assert!(ground.contains(&1), "邻域普通地面仍可分类");
        assert!(!ground.contains(&0) && !ground.contains(&(obstacle as u32)), "同格新增低障碍必须连同含混地面成员保留占据");
        assert!(ground.iter().all(|index| (*index as usize) < wall_start), "邻域支持不能将竖直箱面变成地面");
        let layers = writer.layer_observations([0, 0, 0]).unwrap();
        assert!(layers.values().all(|layer| !layer.members.contains(&local_key(keys[obstacle]))));
        assert_eq!(writer.region(keys[obstacle], keys[obstacle], 1).unwrap().len(), 1);
    }

    #[test]
    fn sparse_step_neighbors_keep_actual_treads_instead_of_cross_step_plane() {
        let mut writer = new_writer();
        let points = sparse_patch_grid(|x, _| if x < 0.2 { 0.01 } else { 0.16 });
        ingest(&mut writer, &points, 1);
        let layers = writer.layer_observations([0, 0, 0]).unwrap();
        assert_eq!(layers.len(), points.len());
        for layer in layers.values() {
            let expected = if layer.plane_origin[0] < 0.2 { 0.01 } else { 0.16 };
            assert!((layer.plane_origin[2] - expected).abs() < 1e-8, "稀疏真实台阶不能被相邻分区拟成斜面：{layer:?}");
            assert!(layer.normal[0].abs() < 1e-8 && layer.normal[1].abs() < 1e-8);
        }
    }

    #[test]
    #[ignore = "需显式提供保留源序号和原索引的真实局部帧"]
    fn replay_recorded_ground_patch_with_original_indices() {
        #[derive(serde::Deserialize)]
        struct Frame {
            sequence: u64,
            timestamp_ns: u64,
            points: Vec<[f64; 3]>,
            indices: Vec<u32>,
        }
        let path = std::env::var("NUWAX_GROUND_REPLAY").expect("真实局部帧路径");
        let target: Key = serde_json::from_str(
            &std::env::var("NUWAX_GROUND_REPLAY_TARGET").unwrap_or("[-7,-2,-7]".into()),
        ).unwrap();
        let frames = std::fs::read_to_string(path).unwrap();
        let mut writer = new_writer();
        let mut report = Vec::new();
        let mut missed = 0;
        let mut observed = 0;
        for line in frames.lines() {
            let frame: Frame = serde_json::from_str(line).unwrap();
            assert_eq!(frame.points.len(), frame.indices.len());
            let points: Vec<_> = frame.points.iter().copied().map(Point3::from).collect();
            let keys: Vec<_> = frame.points.iter().map(|p| {
                let key = super::super::voxel_key(*p, 0.05).unwrap();
                [key.0, key.1, key.2]
            }).collect();
            writer.observe(keys.iter().copied(), frame.timestamp_ns).unwrap();
            let ground = capture(&mut writer, &points, frame.sequence, frame.timestamp_ns,
                Arc::new(LayerSource::Legacy), |index| Some(frame.indices[index]), |_| false).unwrap();
            let target_indices: Vec<_> = keys.iter().zip(&frame.indices)
                .filter_map(|(key, index)| (*key == target).then_some(*index)).collect();
            observed += target_indices.len();
            let rejected: Vec<_> = target_indices.iter().filter(|index| !ground.contains(index)).copied().collect();
            missed += rejected.len();
            let layers = writer.layer_observations(block_key(target)).unwrap();
            report.push(serde_json::json!({"sequence":frame.sequence,"timestamp_ns":frame.timestamp_ns,
                "points":points.len(),"classified_points":ground.len(),"target_indices":target_indices,
                "unclassified_target_indices":rejected,"persisted_target_layer":layers.values()
                    .find(|layer|layer.members.contains(&local_key(target)))
                    .map(|layer|serde_json::json!({"origin":layer.plane_origin,"normal":layer.normal,"samples":layer.samples.len()}))}));
        }
        if let Ok(path) = std::env::var("NUWAX_GROUND_REPLAY_REPORT") {
            std::fs::write(path, serde_json::to_vec_pretty(&report).unwrap()).unwrap();
        }
        eprintln!("真实局部帧重放：{}帧，拒绝格未分类原始回波{missed}", report.len());
        assert!(observed > 0, "输入必须实际包含目标回波，不能把空对照当通过");
        assert_eq!(missed, 0, "已由独立物理表面核实的拒绝格回波应具有当前源的地面分类");
    }

    #[test]
    fn real_sparse_ground_refinement_keeps_the_selected_central_support() {
        // 第175帧完整27点邻域，直接保留真实原坐标及索引；目标586已独立匹配物理地面。
        let original: [([f64; 3], u32); 27] = [
            ([-0.46524500846862793, -0.03422210365533829, -0.3421630561351776], 214),
            ([-0.5980445742607117, -0.05577140301465988, -0.3405984044075012], 334),
            ([-0.4471633732318878, -0.1649593561887741, -0.33730635046958923], 594),
            ([-0.5699533224105835, -0.16625407338142395, -0.344806432723999], 704),
            ([-0.43143904209136963, -0.10384058952331543, -0.34493863582611084], 785),
            ([-0.5077713131904602, 0.12125318497419357, -0.3474777936935425], 387),
            ([-0.5099345445632935, 0.06262246519327164, -0.34294411540031433], 535),
            ([-0.44276514649391174, 0.07780898362398148, -0.34646400809288025], 617),
            ([-0.4480379521846771, 0.008225071243941784, -0.34369125962257385], 738),
            ([-0.5340133905410767, 0.1637427657842636, -0.3476913571357727], 900),
            ([-0.43640467524528503, 0.31405171751976013, -0.34286385774612427], 44),
            ([-0.5636026859283447, 0.29486316442489624, -0.3522794842720032], 350),
            ([-0.4705148935317993, 0.31106361746788025, -0.3466835618019104], 557),
            ([-0.4149859547615051, 0.23787519335746765, -0.34204554557800293], 795),
            ([-0.38893425464630127, -0.140560582280159, -0.3446272611618042], 66),
            ([-0.3381047248840332, -0.1735590100288391, -0.33688923716545105], 97),
            ([-0.32019251585006714, -0.13507291674613953, -0.3381752073764801], 148),
            ([-0.33374395966529846, -0.15351256728172302, -0.33628302812576294], 651),
            ([-0.3437967896461487, 0.036629095673561096, -0.33915600180625916], 101),
            ([-0.35961535573005676, 0.10907970368862152, -0.3384304344654083], 472),
            ([-0.2995506227016449, 0.13993710279464722, -0.3295917809009552], 586),
            ([-0.354240745306015, 0.021305572241544724, -0.3353867530822754], 838),
            ([-0.20120486617088318, 0.32915523648262024, -0.33858370780944824], 150),
            ([-0.3002198338508606, 0.25639572739601135, -0.3336906135082245], 209),
            ([-0.0007987688295543194, 0.39117708802223206, -0.34497812390327454], 171),
            ([-0.02416551485657692, 0.38935571908950806, -0.345954567193985], 673),
            ([-0.1313919723033905, 0.31196221709251404, -0.3462202847003937], 768),
        ];
        let points: Vec<_> = original.into_iter().map(|(point, index)| {
            let key = super::super::voxel_key(point, 0.05).unwrap();
            Return { point, index, key: [key.0, key.1, key.2] }
        }).collect();
        let available: Vec<_> = (0..points.len()).collect();
        let quantum = f64::from(0.05_f32);
        let (_, _, inliers) = fit_plane(&points, &available, quantum * 0.2, quantum, [-2, 0]).unwrap();
        let retained: BTreeSet<_> = inliers.iter().map(|index| points[*index].index).collect();
        assert!([101, 472, 586, 838].iter().all(|index| retained.contains(index)),
            "细化不能用邻区点替换原最佳候选已经解释的中央真实证据");
    }

    #[test]
    #[ignore = "只读诊断：需完整真实单帧及指定原索引，不改变生产拟合结果"]
    fn diagnose_recorded_same_frame_plane_choices() {
        #[derive(serde::Deserialize)]
        struct Frame {
            sequence: u64, timestamp_ns: u64, points: Vec<[f64; 3]>,
            indices: Vec<u32>, source: LayerSource,
        }
        let file = std::env::var("NUWAX_GROUND_REPLAY").unwrap();
        let text = std::fs::read_to_string(file).unwrap();
        let frame: Frame = serde_json::from_str(text.lines().next().unwrap()).unwrap();
        let target: u32 = std::env::var("NUWAX_GROUND_FOCUS_INDEX").unwrap().parse().unwrap();
        let quantum = f64::from(0.05_f32);
        let mut patches: BTreeMap<[i32; 2], Vec<Return>> = BTreeMap::new();
        let mut focus = None;
        for (&point, &index) in frame.points.iter().zip(&frame.indices) {
            let key = super::super::voxel_key(point, 0.05).unwrap();
            let patch = [key.0.div_euclid(PATCH_SIDE_VOXELS), key.1.div_euclid(PATCH_SIDE_VOXELS)];
            if index == target { focus = Some(patch); }
            patches.entry(patch).or_default().push(Return { point, index, key: [key.0, key.1, key.2] });
        }
        let focus = focus.expect("原索引必须真实存在");
        FIT_CHOICES.with(|trace| *trace.borrow_mut() = Some((focus, Vec::new())));
        let central = &patches[&focus];
        let mut classified = BTreeSet::new();
        let mut trials = Vec::new();
        for neighboring in [false, true] {
            let mut fitting = central.clone();
            if neighboring {
                fitting.clear();
                for dx in -1..=1 { for dy in -1..=1 {
                    if let Some(points) = patches.get(&[focus[0] + dx, focus[1] + dy]) {
                        fitting.extend(points.iter().cloned());
                    }
                }}
            }
            let stride = fitting.len().div_ceil(MAX_PATCH_SAMPLES).max(1);
            let mut remaining: Vec<_> = (0..fitting.len()).step_by(stride).collect();
            for layer in 0..MAX_PATCH_LAYERS {
                let before: Vec<_> = remaining.iter().map(|index| fitting[*index].index).collect();
                let Some((normal, origin, inliers)) = fit_plane(&fitting, &remaining, quantum * 0.2, quantum, focus) else {
                    trials.push(serde_json::json!({"neighboring":neighboring,"layer":layer,
                        "remaining_indices":before,"fit":null}));
                    break;
                };
                let inlier_set: BTreeSet<_> = inliers.iter().copied().collect();
                remaining.retain(|index| !inlier_set.contains(index));
                let central_distances: Vec<_> = central.iter().map(|point| serde_json::json!({
                    "index":point.index,"point":point.point,"key":point.key,
                    "distance_m":distance(point.point, origin, normal),
                    "previously_classified":classified.contains(&point.index),
                })).collect();
                trials.push(serde_json::json!({"neighboring":neighboring,"layer":layer,"normal":normal,
                    "origin":origin,"remaining_indices":before,"inlier_indices":inliers.iter()
                        .map(|index|fitting[*index].index).collect::<Vec<_>>(),"central":central_distances}));
                classified.extend(central.iter().filter(|point|
                    distance(point.point, origin, normal).abs() <= quantum * 0.2).map(|point|point.index));
            }
        }
        let points: Vec<_> = frame.points.iter().copied().map(Point3::from).collect();
        let mut writer = new_writer();
        writer.observe(frame.points.iter().map(|point| {
            let key = super::super::voxel_key(*point, 0.05).unwrap(); [key.0, key.1, key.2]
        }), frame.timestamp_ns).unwrap();
        let ground = capture(&mut writer, &points, frame.sequence, frame.timestamp_ns,
            Arc::new(frame.source), |index| Some(frame.indices[index]), |_| false).unwrap();
        let fit_choices = FIT_CHOICES.with(|trace| trace.borrow_mut().take().unwrap().1);
        let report = serde_json::json!({"sequence":frame.sequence,"target_index":target,"focus_patch":focus,
            "actual_capture_target_ground":ground.contains(&target),"trials":trials,"fit_choices":fit_choices});
        std::fs::write(std::env::var("NUWAX_GROUND_REPLAY_REPORT").unwrap(), serde_json::to_vec_pretty(&report).unwrap()).unwrap();
    }
    #[test]
    fn separate_floors_and_step_treads_never_form_a_column_average() {
        let mut writer = new_writer();
        let mut points = grid(|_, _| 0.01);
        points.extend(grid(|_, _| 0.31));
        ingest(&mut writer, &points, 1);
        let layers = writer.layer_observations([0, 0, 0]).unwrap();
        assert!(!layers.is_empty());
        let same_xy: Vec<_> = layers
            .values()
            .filter(|l| l.members[0] % 1024 == 0)
            .collect();
        assert_eq!(same_xy.len(), 2);
        for layer in same_xy {
            let z = layer.plane_origin[2];
            assert!((z - 0.01).abs() < 1e-8 || (z - 0.31).abs() < 1e-8);
        }
        let mut steps = new_writer();
        let points = grid(|x, _| if x < 0.095 { 0.01 } else { 0.16 });
        ingest(&mut steps, &points, 1);
        for layer in steps.layer_observations([0, 0, 0]).unwrap().values() {
            let z = layer.plane_origin[2];
            assert!(
                (z - 0.01).abs() < 1e-8 || (z - 0.16).abs() < 1e-8,
                "invented ramp at {z}"
            );
        }
    }
    #[test]
    fn slope_is_detrended_and_wall_does_not_become_ground() {
        let mut writer = new_writer();
        let points = grid(|x, y| 0.01 + 0.2 * x + 0.1 * y);
        ingest(&mut writer, &points, 1);
        let layers = writer.layer_observations([0, 0, 0]).unwrap();
        assert!(layers.len() >= 8);
        for layer in layers.values() {
            assert!(
                (layer.plane_origin[2]
                    - (0.01 + 0.2 * layer.plane_origin[0] + 0.1 * layer.plane_origin[1]))
                    .abs()
                    < 1e-8
            );
        }
        let mut wall = new_writer();
        let points = (0..8)
            .flat_map(|y| {
                (0..8)
                    .map(move |z| Point3::new(0.08, 0.01 + y as f64 * 0.02, 0.01 + z as f64 * 0.04))
            })
            .collect::<Vec<_>>();
        ingest(&mut wall, &points, 1);
        assert!(wall.layer_observations([0, 0, 0]).unwrap().is_empty());
    }
    #[test]
    fn outliers_stay_occupied_and_mixed_voxel_is_not_a_ground_member() {
        let mut writer = new_writer();
        let mut points = grid(|_, _| 0.01);
        points.push(Point3::new(0.008, 0.008, 0.045));
        points.push(Point3::new(0.08, 0.08, 0.4));
        ingest(&mut writer, &points, 1);
        let layers = writer.layer_observations([0, 0, 0]).unwrap();
        assert!(!layers.is_empty());
        assert!(layers.values().all(|l| !l.members.contains(&0)));
        assert!(layers
            .values()
            .all(|l| (l.plane_origin[2] - 0.01).abs() < 1e-8));
        let key = super::super::voxel_key([0.08, 0.08, 0.4], 0.05).unwrap();
        assert_eq!(
            writer
                .region([key.0, key.1, key.2], [key.0, key.1, key.2], 1)
                .unwrap()
                .len(),
            1,
            "non-ground return must remain collision occupancy"
        );
    }
    #[test]
    fn retained_returns_are_bounded_and_pure_refresh_keeps_geometry() {
        let mut writer = new_writer();
        let points = grid(|_, _| 0.01);
        ingest(&mut writer, &points, 1);
        let first = writer.snapshot(domain(), 1, 1).unwrap();
        for stamp in 2..=15 {
            ingest(&mut writer, &points, stamp);
        }
        let last = writer.snapshot(domain(), 2, 15).unwrap();
        assert_eq!(last.geometry_revision, first.geometry_revision);
        for layer in writer.layer_observations([0, 0, 0]).unwrap().values() {
            assert!(layer.samples.len() <= MAX_LAYER_SAMPLES);
            assert_eq!(
                layer.statistics().unwrap().moments().count() as usize,
                layer.samples.len()
            );
        }
    }
    #[test]
    fn noisy_ground_and_sparse_upper_returns_keep_separate_representatives() {
        let mut writer = new_writer();
        let quantum = f64::from(writer.voxel_size_m());
        for frame in 1..=4 {
            let mut points: Vec<_> = grid(|x,y| {
                // 以源体素的十分之一为峰峰噪声；帧间扰动独立于高处回波。
                let parity = ((x/0.022).round() as i32 + (y/0.022).round() as i32 + frame) % 2;
                0.01 + if parity == 0 {quantum*0.05} else {-quantum*0.05}
            });
            let echoes: Vec<_> = points.iter().step_by(8).map(|p|
                Point3::new(p.x,p.y,p.z+quantum*4.0)).collect();
            points.extend(echoes.iter().copied());
            ingest(&mut writer,&points,frame as u64);
            let layers = writer.layer_observations([0,0,0]).unwrap();
            let floor: Vec<_> = layers.values().filter(|layer|layer.plane_origin[2]<quantum).collect();
            assert!(!floor.is_empty(),"有二维面积支撑的抖动地面应保留分层观测");
            for layer in floor {
                assert!((layer.plane_origin[2]-0.01).abs()<=quantum*0.1,
                    "地面拟合不能向上方回波的混合均值漂移");
                assert!(layer.samples.iter().all(|sample|sample.point[2]<quantum));
            }
            // 未归入地面层的上方回波仍是真实占据；拟合不删除物理几何。
            for echo in echoes {
                let key = super::super::voxel_key([echo.x,echo.y,echo.z],quantum as f32).unwrap();
                let key = [key.0,key.1,key.2];
                assert_eq!(writer.region(key,key,1).unwrap().len(),1);
            }
        }
    }

    #[test]
    fn bounded_plane_stabilization_eventually_tracks_both_height_directions() {
        let mut writer = new_writer();
        ingest(&mut writer, &grid(|_, _| 0.01), 1);
        let initial = writer.layer_observations([0, 0, 0]).unwrap()[&0].plane_origin[2];
        for (stamp, z) in [(2, 0.012), (3, 0.014)] {
            ingest(&mut writer, &grid(|_, _| z), stamp);
            assert_eq!(
                writer.layer_observations([0, 0, 0]).unwrap()[&0].plane_origin[2],
                initial
            );
        }
        ingest(&mut writer, &grid(|_, _| 0.018), 4);
        assert!(
            (writer.layer_observations([0, 0, 0]).unwrap()[&0].plane_origin[2] - 0.018).abs()
                < 1e-8
        );
        ingest(&mut writer, &grid(|_, _| 0.006), 5);
        assert!(
            (writer.layer_observations([0, 0, 0]).unwrap()[&0].plane_origin[2] - 0.006).abs()
                < 1e-8
        );
    }
    #[test]
    fn persistent_ground_does_not_expire_but_one_conflicting_return_withdraws_its_mask() {
        let mut writer = new_writer();
        ingest(&mut writer, &grid(|_, _| 0.01), 1);
        let old = writer.layer_observations([0, 0, 0]).unwrap();
        assert!(old.contains_key(&0));
        ingest(&mut writer, &[Point3::new(10.0, 10.0, 1.0)], 1_000_000_000);
        assert_eq!(
            writer.layer_observations([0, 0, 0]).unwrap(),
            old,
            "absence of new ground is not expiration"
        );
        ingest(
            &mut writer,
            &[Point3::new(0.008, 0.008, 0.045)],
            1_000_000_001,
        );
        let current = writer.layer_observations([0, 0, 0]).unwrap();
        assert!(!current.contains_key(&0));
        assert_eq!(
            current.len() + 1,
            old.len(),
            "single-frame obstacle affects only its ground member"
        );
        assert_eq!(writer.region([0, 0, 0], [0, 0, 0], 1).unwrap().len(), 1);
    }
    #[test]
    fn sparse_unclassified_scan_does_not_make_an_unchanged_block_dirty() {
        let mut writer = new_writer();
        writer.observe([[0, 0, 0]], 1).unwrap();
        writer.snapshot(domain(), 1, 1).unwrap();
        writer.region([0, 0, 0], [0, 0, 0], 1).unwrap();
        let before = writer.resident_bytes();
        assert!(capture(&mut writer, &[Point3::new(0.008, 0.008, 0.01)], 2, 2, Arc::new(LayerSource::Legacy), |index| u32::try_from(index).ok(), |_|false).unwrap().is_empty());
        assert!(!writer.has_unpublished_changes());
        assert_eq!(writer.resident_bytes(), before, "no duplicate dirty occupancy for unchanged empty layers");
    }

    #[test]
    fn revoking_the_last_layer_writes_empty_layers_and_changes_geometry() {
        let mut writer = new_writer();
        ingest(&mut writer, &grid(|_, _|0.01), 1);
        let only = writer.layer_observations([0, 0, 0]).unwrap().remove(&0).unwrap();
        writer.replace_layer_observations([0, 0, 0], [(0, only)].into()).unwrap();
        let before = writer.snapshot(domain(), 1, 1).unwrap();
        ingest(&mut writer, &[Point3::new(0.008, 0.008, 0.045)], 2);
        assert!(writer.layer_observations([0, 0, 0]).unwrap().is_empty());
        let after = writer.snapshot(domain(), 2, 2).unwrap();
        assert!(after.geometry_revision > before.geometry_revision);
        assert_eq!(writer.region([0, 0, 0], [0, 0, 0], 1).unwrap().len(), 1);
    }

    // Stage-only: excludes capture, Writer persistence, publish, planner and controller.
    #[test]
    #[ignore = "manual serial release exact-output association crossover benchmark"]
    fn benchmark_same_column_association_crossover() {
        use std::{hint::black_box,time::Instant};
        let q=0.05_f32;let threshold=f64::from(q)*0.2;
        for columns in [1_u16,4,32,1024] {for zs in [&[0_u16][..],&[0,31][..],&[0,1,2,3][..]] {
            let mut initial=LayerObservations::new();
            for &z in zs {for column in 0..columns {
                let anchor=column+1024*z;
                let point=[(f64::from(column%32)+0.5)*f64::from(q),
                    (f64::from(column/32)+0.5)*f64::from(q),(f64::from(z)+0.5)*f64::from(q)];
                initial.insert(anchor,LayerObservation{members:vec![anchor],normal:[0.,0.,1.],plane_origin:point,
                    samples:vec![LayerSample{source:Arc::new(LayerSource::Legacy),point,
                        point_index:u32::from(anchor),source_sequence:1,source_timestamp_ns:1}]});
            }}
            let rows:Vec<_>=initial.values().collect();
            let candidates:Vec<_>=(0..1024).map(|i| {
                let mut value=(*rows[(i*127)%rows.len()]).clone();
                value.samples[0].source_sequence=2;value.samples[0].source_timestamp_ns=2;value
            }).collect();
            let mut expected=initial.clone();
            for candidate in candidates.iter().cloned() {
                associate_full_block_reference(&mut expected,candidate,threshold,[0;3],q);
            }
            for pair in 0..7 {for fast in if pair%2==0 {[false,true]} else {[true,false]} {
                let mut actual=initial.clone();let fresh=candidates.clone();let start=Instant::now();
                for candidate in fresh {
                    if fast {associate(&mut actual,black_box(candidate),threshold,[0;3],q);}
                    else {associate_full_block_reference(&mut actual,black_box(candidate),threshold,[0;3],q);}
                }
                let elapsed=start.elapsed().as_nanos();
                assert_eq!(actual,expected,"complete exact layer/sample equality");
                eprintln!("ASSOCIATION_NATIVE_AB columns={columns} zs={zs:?} updates=1024 pair={pair} fast={fast} ns={elapsed}");
            }}
        }}
    }

}
