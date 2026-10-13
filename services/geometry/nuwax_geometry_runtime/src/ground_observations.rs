//! Capture-side physical-layer association. Only admitted original return indices
//! enter these bounded local fits; occupancy centres and replay resampling do not.
use nalgebra::{Matrix3, Point3, Vector3};
use nuwax_map_chunks::{
    block_key, local_key, Key, LayerObservation, LayerObservations, LayerSample, Writer,
    MAX_LAYERS_PER_BLOCK, MAX_LAYER_SAMPLES,
};
use std::collections::{BTreeMap, BTreeSet};

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
    first_index: usize,
    sequence: u64,
    timestamp_ns: u64,
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
        let index =
            u32::try_from(first_index + index).map_err(|_| "admitted point index overflow")?;
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
    // Retire only claims contradicted by an actual return in a claimed member.
    // A recent obstacle sharing a fine voxel with old ground must not be hidden.
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
    let mut contradicted = BTreeSet::new();
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
                contradicted.insert((block, *anchor));
            }
        }
    }
    let revoked_blocks: BTreeSet<_> = contradicted.iter().map(|(block, _)| *block).collect();
    for (block, anchor) in contradicted {
        existing.get_mut(&block).unwrap().remove(&anchor);
    }
    // This is bounded by the admitted frame, not by accumulated map size.
    let mut updates: BTreeMap<Key, Vec<LayerObservation>> = BTreeMap::new();
    let mut current_ground = BTreeSet::new();
    for returns in patches.values_mut() {
        // Evenly decimate indices only for fitting; never generate interpolated returns.
        let stride = returns.len().div_ceil(MAX_PATCH_SAMPLES).max(1);
        let mut classified = BTreeSet::new();
        let mut remaining: Vec<usize> = (0..returns.len()).step_by(stride).collect();
        for _ in 0..MAX_PATCH_LAYERS {
            let Some((normal, origin, fit)) = fit_plane(returns, &remaining, residual, quantum)
            else {
                break;
            };
            let members: BTreeSet<usize> = fit.into_iter().collect();
            remaining.retain(|index| !members.contains(index));
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
            associate(&mut layers, candidate, residual);
        }
        writer.replace_layer_observations(key, layers)?;
    }
    Ok(current_ground.into_iter().collect())
}

fn distance(point: [f64; 3], origin: [f64; 3], normal: [f64; 3]) -> f64 {
    (0..3).map(|i| (point[i] - origin[i]) * normal[i]).sum()
}

/// A bounded deterministic multi-hypothesis fit separates planes before computing
/// statistics. Vertical surfaces cannot seed an upward ground patch. Residuals are
/// perpendicular to the plane, so a sloping surface is not split into height bins.
fn fit_plane(
    points: &[Return],
    available: &[usize],
    threshold: f64,
    quantum: f64,
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
        return (all_normal.z >= 0.5)
            .then(|| (all_normal.into(), all_mean.into(), available.to_vec()));
    }
    let mut best = Vec::new();
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
        if normal.z < 0.5 {
            continue;
        }
        let inliers: Vec<_> = available
            .iter()
            .copied()
            .filter(|i| distance(points[*i].point, a.into(), normal.into()).abs() <= threshold)
            .collect();
        if inliers.len() > best.len() {
            best = inliers;
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
    if normal.z < 0.5 {
        return None;
    }
    let inliers: Vec<_> = available
        .iter()
        .copied()
        .filter(|i| distance(points[*i].point, mean.into(), normal.into()).abs() <= threshold)
        .collect();
    (inliers.len() >= MIN_PLANE_INLIERS).then_some((normal.into(), mean.into(), inliers))
}

fn associate(layers: &mut LayerObservations, candidate: LayerObservation, threshold: f64) {
    let anchor = candidate.members[0];
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
        .find(|(_, old)| {
            old.members[0] % 1024 == anchor % 1024
                && old
                    .height_at([center[0], center[1]])
                    .is_some_and(|z| (z - center[2]).abs() <= threshold)
                && old
                    .normal
                    .iter()
                    .zip(candidate.normal)
                    .map(|(a, b)| a * b)
                    .sum::<f64>()
                    > 0.98
        })
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
        let mut samples = old.samples;
        for sample in result.samples {
            if !samples.iter().any(|old| {
                old.source_sequence == sample.source_sequence
                    && old.source_timestamp_ns == sample.source_timestamp_ns
                    && old.point_index == sample.point_index
            }) {
                samples.push(sample);
            }
        }
        // Retained inliers are rechecked against the newly fitted plane. Old ground
        // is not expired by a clock; only new geometric evidence changes this patch.
        samples.retain(|p| distance(p.point, center, result.normal).abs() <= threshold);
        samples.sort_by_key(|s| (s.source_timestamp_ns, s.source_sequence, s.point_index));
        if samples.len() > MAX_LAYER_SAMPLES {
            samples.drain(..samples.len() - MAX_LAYER_SAMPLES);
        }
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

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_map_chunks::{Domain, Repository};
    fn new_writer() -> Writer {
        Writer::new(Repository::temporary().unwrap(), 0.05, 4 * 1024 * 1024).unwrap()
    }
    fn ingest(writer: &mut Writer, points: &[Point3<f64>], stamp: u64) {
        let keys = points
            .iter()
            .map(|p| super::super::voxel_key([p.x, p.y, p.z], 0.05).unwrap())
            .map(|k| [k.0, k.1, k.2]);
        writer.observe(keys, stamp).unwrap();
        capture(writer, points, 0, stamp, stamp, |_| false).unwrap();
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
        assert!(capture(&mut writer, &[Point3::new(0.008, 0.008, 0.01)], 0, 2, 2, |_|false).unwrap().is_empty());
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

}
