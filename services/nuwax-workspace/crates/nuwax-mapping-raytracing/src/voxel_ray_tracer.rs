// Copyright 2026 Nuwax Team
// SPDX-License-Identifier: Apache-2.0

use ahash::{AHashMap, AHashSet};
use arrayvec::ArrayVec;
use nalgebra::{Matrix3, Vector3};
use rayon::prelude::*;
use thiserror::Error;
use validator::{Validate, ValidationError};

pub type VoxelKey = (i32, i32, i32);
pub type VoxelHealth = i32;

/// Smallest supported voxel edge. Finer grids make otherwise small rays
/// unreasonably expensive and exceed the precision useful to this `f32` map.
pub const MIN_VOXEL_SIZE_METERS: f32 = 0.001;
/// Deterministic upper bound for one 3-D DDA traversal.
pub const MAX_STEPS_PER_RAY: u64 = 200_000;
/// Deterministic upper bound for all DDA traversals in one JSON request.
pub const MAX_REQUEST_RAY_STEPS: u64 = 50_000_000;

#[derive(Validate)]
#[validate(schema(function = "validate_config_limits"))]
// 注：原 `dimos_module::native_config` 过程宏在 1:1 源码中存在，它为 `dimos-module` 框架
// 提供运行时反射支持。本 crate 作为 nuwax standalone 库不需要该反射元数据，
// 因此移除该属性（功能等价于"无反射元数据"）。阶段 0 完成兼容 shim 后可重新引入。
pub struct Config {
    #[validate(range(min = 0.001))]
    pub voxel_size: f32,
    #[validate(range(min = 0.0))]
    pub max_range: f32,
    #[validate(range(min = 1))]
    pub ray_subsample: u32,
    #[validate(range(min = 0.0))]
    pub shadow_depth: f32,
    #[validate(range(min = 0.0))]
    pub grace_depth: f32,
    pub min_health: i32,
    #[validate(range(min = 1))]
    pub max_health: i32,
    /// Spare a miss when abs of ray dot normal is below this. Higher clears only
    /// on direct hits, lower clears on slight grazes too.
    #[validate(range(min = 0.0, max = 1.0))]
    pub graze_cos: f32,
    /// Only spare a voxel whose neighborhood was hit within this many frames.
    /// Large disables it.
    pub recency_window: u32,
    /// Publish the accumulated local map and region bounds every Nth frame. Zero disables them.
    #[validate(range(min = 0))]
    pub emit_every: u32,
    /// Publish the global map every Nth frame. Zero disables it.
    #[validate(range(min = 0))]
    pub global_emit_every: u32,
    /// Size the local region to this percentile of batch point distances, so a
    /// stray far hit cannot inflate it.
    #[validate(range(min = 0.0, max = 100.0))]
    pub region_percentile: f32,
}

fn validate_config_limits(cfg: &Config) -> Result<(), ValidationError> {
    if cfg.min_health >= cfg.max_health {
        return Err(ValidationError::new("min_health_lt_max_health"));
    }
    if cfg.max_range > 0.0
        && conservative_ray_step_bound(
            f64::from(cfg.max_range),
            f64::from(cfg.shadow_depth),
            f64::from(cfg.voxel_size),
        )
        .is_none_or(|steps| steps > MAX_STEPS_PER_RAY)
    {
        return Err(ValidationError::new("ray_step_budget"));
    }
    Ok(())
}

/// Conservative number of voxel crossings for a ray of `range_m`, including
/// its shadow extension. The `sqrt(3)` factor bounds the L1 displacement after
/// any rigid rotation, while DDA advances at most one axis per iteration.
pub fn conservative_ray_step_bound(
    range_m: f64,
    shadow_depth: f64,
    voxel_size: f64,
) -> Option<u64> {
    if !range_m.is_finite()
        || !shadow_depth.is_finite()
        || !voxel_size.is_finite()
        || range_m < 0.0
        || shadow_depth < 0.0
        || voxel_size < f64::from(MIN_VOXEL_SIZE_METERS)
    {
        return None;
    }
    let path_m = range_m + shadow_depth;
    let steps = (path_m * 3.0_f64.sqrt() / voxel_size).ceil() + 3.0;
    if !steps.is_finite() || steps > u64::MAX as f64 {
        None
    } else {
        Some(steps as u64)
    }
}

#[derive(Default)]
pub struct VoxelMap {
    pub voxels: AHashMap<VoxelKey, Voxel>,
    frame: u32,
}

impl VoxelMap {
    pub fn healthy_count(&self) -> usize {
        self.voxels.values().filter(|c| c.health > 0).count()
    }

    /// Add a return to its voxel's accumulated moments.
    fn accumulate(&mut self, point: (f32, f32, f32), voxel_size: f32) {
        let key = world_to_voxel(point.0, point.1, point.2, 1.0 / voxel_size);
        let center = Vector3::new(
            (key.0 as f32 + 0.5) * voxel_size,
            (key.1 as f32 + 0.5) * voxel_size,
            (key.2 as f32 + 0.5) * voxel_size,
        );
        self.voxels
            .entry(key)
            .or_default()
            .observe(Vector3::new(point.0, point.1, point.2) - center);
    }

    #[cfg(test)]
    fn set(&mut self, key: VoxelKey, health: VoxelHealth) {
        self.voxels.insert(key, Voxel::with_health(health));
    }

    #[cfg(test)]
    fn health(&self, key: VoxelKey) -> Option<VoxelHealth> {
        self.voxels.get(&key).map(|c| c.health)
    }

    /// Fit every occupied voxel's normal from its pooled neighborhood.
    #[cfg(test)]
    fn recompute_all_normals(&mut self, voxel_size: f32) {
        let updates: Vec<(VoxelKey, Option<Vector3<f32>>)> = self
            .voxels
            .keys()
            .copied()
            .map(|k| (k, pooled_normal_and_recency(&self.voxels, k, voxel_size).0))
            .collect();
        for (k, n) in updates {
            self.voxels.get_mut(&k).unwrap().normal = n;
        }
    }
}

const NORMAL_MIN_POINTS: u32 = 3;
const NORMAL_NEIGHBOR_RADIUS: i32 = 1;
const NEIGHBORHOOD_CAP: usize = (2 * NORMAL_NEIGHBOR_RADIUS as usize + 1).pow(3);
const NORMAL_REWEIGHT_ITERS: u32 = 3;
/// Neighbor weight falloff with plane distance, as a fraction of voxel size.
const NORMAL_PLANE_SIGMA_FRAC: f32 = 0.5;
/// Fraction of points that must survive the IRLS to count as a real plane.
const NORMAL_MIN_SUPPORT: f32 = 0.5;

/// Occupancy health, accumulated point moments about the voxel center, and the
/// normal fit from the voxel's neighborhood.
#[derive(Clone)]
pub struct Voxel {
    pub health: VoxelHealth,
    num_pts: u32,
    sum: Vector3<f32>,
    m2: Matrix3<f32>,
    normal: Option<Vector3<f32>>,
    last_hit: u32,
    // Most recent frame any voxel in this one's neighborhood was hit.
    recency: u32,
}

impl Default for Voxel {
    fn default() -> Self {
        Self {
            health: 0,
            num_pts: 0,
            sum: Vector3::zeros(),
            m2: Matrix3::zeros(),
            normal: None,
            last_hit: 0,
            recency: 0,
        }
    }
}

impl Voxel {
    pub fn with_health(health: VoxelHealth) -> Self {
        Self {
            health,
            ..Default::default()
        }
    }

    /// Fold a centered point into the running moments.
    fn observe(&mut self, q: Vector3<f32>) {
        self.num_pts += 1;
        self.sum += q;
        self.m2 += q * q.transpose();
    }

    #[cfg(test)]
    fn planar_normal(&self) -> Option<Vector3<f32>> {
        self.normal
    }

    /// Fit a normal from this voxel's own points alone, ignoring neighbors.
    #[cfg(test)]
    fn self_normal(&self) -> Option<Vector3<f32>> {
        if self.num_pts < NORMAL_MIN_POINTS {
            return None;
        }
        let n = self.num_pts as f32;
        let mean = self.sum / n;
        fit_normal(self.m2 / n - mean * mean.transpose())
    }
}

/// The surface normal of a covariance, or None unless it is clearly planar.
fn fit_normal(cov: Matrix3<f32>) -> Option<Vector3<f32>> {
    let eig = cov.symmetric_eigen();
    let mut idx = [0usize, 1, 2];
    idx.sort_by(|&a, &b| eig.eigenvalues[a].total_cmp(&eig.eigenvalues[b]));
    let e2 = eig.eigenvalues[idx[2]].max(0.0);
    if e2 < 1e-12 {
        return None;
    }
    let l0 = eig.eigenvalues[idx[0]].max(0.0).sqrt();
    let l1 = eig.eigenvalues[idx[1]].max(0.0).sqrt();
    let l2 = e2.sqrt();
    let linearity = (l2 - l1) / l2;
    let planarity = (l1 - l0) / l2;
    let scattering = l0 / l2;
    if planarity < linearity || planarity < scattering {
        return None;
    }
    Some(eig.eigenvectors.column(idx[0]).into_owned())
}

/// Moments of one neighbor voxel: count, sum, sum of outer products, centroid.
struct Neighbor {
    n: f32,
    s: Vector3<f32>,
    t: Matrix3<f32>,
    centroid: Vector3<f32>,
}

/// Find voxel's normal and the most recent frame any voxel in its
/// neighborhood was hit, from one scan of the neighborhood.
fn pooled_normal_and_recency(
    voxels: &AHashMap<VoxelKey, Voxel>,
    key: VoxelKey,
    voxel_size: f32,
) -> (Option<Vector3<f32>>, u32) {
    let r = NORMAL_NEIGHBOR_RADIUS;
    let mut nbs: ArrayVec<Neighbor, NEIGHBORHOOD_CAP> = ArrayVec::new();
    let mut n_raw: u32 = 0;
    let mut recency = 0;
    for dx in -r..=r {
        for dy in -r..=r {
            for dz in -r..=r {
                let nk = (key.0 + dx, key.1 + dy, key.2 + dz);
                let Some(v) = voxels.get(&nk) else {
                    continue;
                };
                recency = recency.max(v.last_hit);
                if v.num_pts == 0 {
                    continue;
                }
                let ni = v.num_pts as f32;
                // Shift this voxel's center-relative moments to the target center.
                let d = Vector3::new(dx as f32, dy as f32, dz as f32) * voxel_size;
                let s = v.sum + d * ni;
                let t =
                    v.m2 + v.sum * d.transpose() + d * v.sum.transpose() + d * d.transpose() * ni;
                n_raw += v.num_pts;
                nbs.push(Neighbor {
                    n: ni,
                    s,
                    t,
                    centroid: s / ni,
                });
            }
        }
    }
    if n_raw < NORMAL_MIN_POINTS {
        return (None, recency);
    }

    let sigma = NORMAL_PLANE_SIGMA_FRAC * voxel_size;
    let two_sig2 = 2.0 * sigma * sigma;
    let mut weights = [1.0_f32; NEIGHBORHOOD_CAP];
    let mut cov = Matrix3::zeros();
    for _ in 0..NORMAL_REWEIGHT_ITERS {
        let (mut wn, mut s, mut t) = (0.0_f32, Vector3::zeros(), Matrix3::zeros());
        for (nb, &w) in nbs.iter().zip(&weights) {
            wn += w * nb.n;
            s += nb.s * w;
            t += nb.t * w;
        }
        if wn < 1e-6 {
            break;
        }
        let mean = s / wn;
        cov = t / wn - mean * mean.transpose();
        let eig = cov.symmetric_eigen();
        let smallest = eig
            .eigenvalues
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.total_cmp(b.1))
            .map(|(i, _)| i)
            .unwrap();
        let normal = eig.eigenvectors.column(smallest).into_owned();
        for (nb, w) in nbs.iter().zip(&mut weights) {
            let dist = normal.dot(&(nb.centroid - mean)).abs();
            *w = (-(dist * dist) / two_sig2).exp();
        }
    }
    // Reject the plane if too many points had to be discarded to fit it.
    let kept: f32 = nbs.iter().zip(&weights).map(|(nb, &w)| w * nb.n).sum();
    if kept < NORMAL_MIN_SUPPORT * n_raw as f32 {
        return (None, recency);
    }
    (fit_normal(cov), recency)
}

/// Refit the cached normal and neighborhood recency of every voxel whose
/// neighborhood changed this frame.
fn refresh_voxels(
    map: &mut VoxelMap,
    hits: &AHashSet<VoxelKey>,
    removed: &[VoxelKey],
    voxel_size: f32,
) {
    let r = NORMAL_NEIGHBOR_RADIUS;
    let mut dirty: AHashSet<VoxelKey> = AHashSet::new();
    for &c in hits.iter().chain(removed.iter()) {
        for dx in -r..=r {
            for dy in -r..=r {
                for dz in -r..=r {
                    dirty.insert((c.0 + dx, c.1 + dy, c.2 + dz));
                }
            }
        }
    }
    let updates: Vec<(VoxelKey, Option<Vector3<f32>>, u32)> = dirty
        .par_iter()
        .filter(|k| map.voxels.contains_key(k))
        .map(|&k| {
            let (normal, recency) = pooled_normal_and_recency(&map.voxels, k, voxel_size);
            (k, normal, recency)
        })
        .collect();
    for (k, n, rec) in updates {
        if let Some(c) = map.voxels.get_mut(&k) {
            c.normal = n;
            c.recency = rec;
        }
    }
}

/// Spare a clearing miss only when a grazing ray skims a recently hit planar
/// surface. Stale voxels or those with no normal are left to the health checks.
fn should_spare(
    c: &Voxel,
    ray_unit: Vector3<f32>,
    graze_cos: f32,
    frame: u32,
    recency_window: u32,
) -> bool {
    match c.normal {
        Some(n) => {
            frame.saturating_sub(c.recency) <= recency_window && ray_unit.dot(&n).abs() < graze_cos
        }
        None => false,
    }
}

pub struct LocalBounds {
    pub origin_x: f32,
    pub origin_y: f32,
    pub r_xy_max_sq: f32,
    pub z_min: f32,
    pub z_max: f32,
}

impl LocalBounds {
    pub fn contains(&self, x: f32, y: f32, z: f32) -> bool {
        if z < self.z_min || z > self.z_max {
            return false;
        }
        let dx = x - self.origin_x;
        let dy = y - self.origin_y;
        dx * dx + dy * dy <= self.r_xy_max_sq
    }
}

/// A cylinder (cx, cy, radius, z_min, z_max) on the mean origin, sized to a
/// percentile of the point distances so a stray far hit cannot inflate it.
/// Points must be finite. An empty batch yields a zero-radius region.
pub fn batch_local_bounds(
    points: &[(f32, f32, f32)],
    origins: &[(f32, f32, f32)],
    percentile_pct: f32,
    margin: f32,
) -> (f32, f32, f32, f32, f32) {
    let n = origins.len().max(1) as f64;
    let cx = (origins.iter().map(|o| o.0 as f64).sum::<f64>() / n) as f32;
    let cy = (origins.iter().map(|o| o.1 as f64).sum::<f64>() / n) as f32;
    if points.is_empty() {
        let cz = (origins.iter().map(|o| o.2 as f64).sum::<f64>() / n) as f32;
        return (cx, cy, 0.0, cz, cz);
    }

    let mut dist: Vec<f32> = points.iter().map(|p| (p.0 - cx).hypot(p.1 - cy)).collect();
    let mut zs: Vec<f32> = points.iter().map(|p| p.2).collect();
    let radius = percentile(&mut dist, percentile_pct) + margin;
    let z_min = percentile(&mut zs, 100.0 - percentile_pct) - margin;
    let z_max = percentile(&mut zs, percentile_pct) + margin;
    (cx, cy, radius, z_min, z_max)
}

fn percentile(values: &mut [f32], p: f32) -> f32 {
    let n = values.len();
    if n == 1 {
        return values[0];
    }
    let rank = (p as f64 / 100.0).clamp(0.0, 1.0) * (n - 1) as f64;
    let lo = rank.floor() as usize;
    let frac = (rank - lo as f64) as f32;
    let (_, &mut v_lo, rest) = values.select_nth_unstable_by(lo, |a, b| a.total_cmp(b));
    if frac == 0.0 || rest.is_empty() {
        return v_lo;
    }
    let v_hi = rest.iter().copied().fold(f32::INFINITY, f32::min);
    v_lo + frac * (v_hi - v_lo)
}

pub fn iter_global_points(
    map: &VoxelMap,
    voxel_size: f32,
) -> impl Iterator<Item = (f32, f32, f32)> + '_ {
    let half = voxel_size * 0.5;
    map.voxels
        .iter()
        .filter(|(_, c)| c.health > 0)
        .map(move |(&(kx, ky, kz), _)| {
            (
                kx as f32 * voxel_size + half,
                ky as f32 * voxel_size + half,
                kz as f32 * voxel_size + half,
            )
        })
}

/// Healthy voxel centers paired with their surface normal, the zero vector where
/// there is no plane.
pub fn iter_global_normals(
    map: &VoxelMap,
    voxel_size: f32,
) -> impl Iterator<Item = ((f32, f32, f32), [f32; 3])> + '_ {
    let half = voxel_size * 0.5;
    map.voxels
        .iter()
        .filter(|(_, c)| c.health > 0)
        .map(move |(&(kx, ky, kz), c)| {
            let pos = (
                kx as f32 * voxel_size + half,
                ky as f32 * voxel_size + half,
                kz as f32 * voxel_size + half,
            );
            let normal = c.normal.map_or([0.0; 3], |n| [n[0], n[1], n[2]]);
            (pos, normal)
        })
}

fn live_voxels(points: &[(f32, f32, f32)], voxel_size: f32) -> AHashSet<VoxelKey> {
    let inv = 1.0_f32 / voxel_size;
    let mut out: AHashSet<VoxelKey> = AHashSet::with_capacity(points.len());
    for &(x, y, z) in points {
        out.insert(world_to_voxel(x, y, z, inv));
    }
    out
}

#[derive(Debug, Error, PartialEq)]
pub enum UpdateMapError {
    #[error("invalid ray-tracing configuration: {0}")]
    InvalidConfig(String),
    #[error("ray origin contains a non-finite value")]
    NonFiniteOrigin,
    #[error("point at index {index} contains a non-finite value")]
    NonFinitePoint { index: usize },
    #[error("ray origin is outside the supported voxel coordinate range")]
    OriginVoxelRange,
    #[error(
        "point at index {index} or its shadow traversal is outside the supported voxel coordinate range"
    )]
    RayVoxelRange { index: usize },
    #[error(
        "point at index {index} requires {steps} steps, exceeding the {MAX_STEPS_PER_RAY}-step per-ray limit"
    )]
    RayStepBudget { index: usize, steps: u64 },
}

/// Backward-compatible infallible update entry point.
///
/// Invalid inputs panic instead of being partially applied or silently
/// truncated. New callers should use [`try_update_map`] to handle rejection.
pub fn update_map(
    map: &mut VoxelMap,
    origin: (f32, f32, f32),
    points: &[(f32, f32, f32)],
    cfg: &Config,
) -> AHashSet<VoxelKey> {
    try_update_map(map, origin, points, cfg).expect("invalid input passed to update_map")
}

/// Validate and apply one map update without partially mutating `map` on error.
pub fn try_update_map(
    map: &mut VoxelMap,
    origin: (f32, f32, f32),
    points: &[(f32, f32, f32)],
    cfg: &Config,
) -> Result<AHashSet<VoxelKey>, UpdateMapError> {
    validate_update_inputs(origin, points, cfg)?;
    Ok(update_map_prevalidated(map, origin, points, cfg))
}

/// Apply an update whose configuration, work budget, and coordinate range were
/// already validated. Multi-runtime comparisons use this only after every
/// variant accepts the frame.
pub(crate) fn update_map_prevalidated(
    map: &mut VoxelMap,
    origin: (f32, f32, f32),
    points: &[(f32, f32, f32)],
    cfg: &Config,
) -> AHashSet<VoxelKey> {
    let inv = 1.0_f32 / cfg.voxel_size;
    let max_range_sq = if cfg.max_range > 0.0 {
        cfg.max_range * cfg.max_range
    } else {
        f32::INFINITY
    };

    map.frame += 1;
    let frame = map.frame;
    let hits = live_voxels(points, cfg.voxel_size);

    let origin_voxel = world_to_voxel(origin.0, origin.1, origin.2, inv);
    let step = cfg.ray_subsample as usize;
    let voxels = &map.voxels;
    let misses: AHashSet<VoxelKey> = points
        .par_iter()
        .enumerate()
        .fold(AHashSet::new, |mut misses, (i, &p)| {
            if i % step != 0 {
                return misses;
            }
            let dx = p.0 - origin.0;
            let dy = p.1 - origin.1;
            let dz = p.2 - origin.2;
            if dx * dx + dy * dy + dz * dz > max_range_sq {
                return misses;
            }
            let endpoint = world_to_voxel(p.0, p.1, p.2, inv);
            find_misses_along_ray(
                &mut misses,
                voxels,
                origin,
                p,
                cfg.voxel_size,
                cfg.shadow_depth,
                cfg.grace_depth,
                cfg.graze_cos,
                frame,
                cfg.recency_window,
                origin_voxel,
                endpoint,
            );
            misses
        })
        .reduce(AHashSet::new, |mut a, mut b| {
            if a.len() < b.len() {
                std::mem::swap(&mut a, &mut b);
            }
            a.extend(b);
            a
        });

    for v in &hits {
        let c = map.voxels.entry(*v).or_insert_with(|| Voxel {
            health: cfg.min_health,
            ..Default::default()
        });
        c.health = (c.health + 1).min(cfg.max_health);
        c.last_hit = frame;
    }

    for &p in points {
        map.accumulate(p, cfg.voxel_size);
    }

    let mut removed: Vec<VoxelKey> = Vec::new();
    for v in misses.difference(&hits) {
        if let Some(c) = map.voxels.get_mut(v) {
            c.health -= 1;
            if c.health <= cfg.min_health {
                map.voxels.remove(v);
                removed.push(*v);
            }
        }
    }

    refresh_voxels(map, &hits, &removed, cfg.voxel_size);

    hits
}

pub(crate) fn validate_update_inputs(
    origin: (f32, f32, f32),
    points: &[(f32, f32, f32)],
    cfg: &Config,
) -> Result<(), UpdateMapError> {
    cfg.validate()
        .map_err(|error| UpdateMapError::InvalidConfig(error.to_string()))?;
    let config_values = [
        cfg.voxel_size,
        cfg.max_range,
        cfg.shadow_depth,
        cfg.grace_depth,
        cfg.graze_cos,
        cfg.region_percentile,
    ];
    if !config_values.into_iter().all(f32::is_finite) {
        return Err(UpdateMapError::InvalidConfig(
            "all floating-point fields must be finite".to_owned(),
        ));
    }
    if ![origin.0, origin.1, origin.2]
        .into_iter()
        .all(f32::is_finite)
    {
        return Err(UpdateMapError::NonFiniteOrigin);
    }
    let inv = 1.0_f32 / cfg.voxel_size;
    if !voxel_point_is_safe(origin, inv) {
        return Err(UpdateMapError::OriginVoxelRange);
    }

    for (index, point) in points.iter().copied().enumerate() {
        if ![point.0, point.1, point.2].into_iter().all(f32::is_finite) {
            return Err(UpdateMapError::NonFinitePoint { index });
        }
        if !voxel_point_is_safe(point, inv) {
            return Err(UpdateMapError::RayVoxelRange { index });
        }
        if index % cfg.ray_subsample as usize != 0 {
            continue;
        }
        let range = f64::from(point.0 - origin.0)
            .hypot(f64::from(point.1 - origin.1))
            .hypot(f64::from(point.2 - origin.2));
        if cfg.max_range > 0.0 && range > f64::from(cfg.max_range) {
            continue;
        }
        if !shadow_traversal_is_safe(origin, point, cfg) {
            return Err(UpdateMapError::RayVoxelRange { index });
        }
        let steps = conservative_ray_step_bound(
            range,
            f64::from(cfg.shadow_depth),
            f64::from(cfg.voxel_size),
        )
        .ok_or_else(|| {
            UpdateMapError::InvalidConfig("ray work estimate is not finite".to_owned())
        })?;
        if steps > MAX_STEPS_PER_RAY {
            return Err(UpdateMapError::RayStepBudget { index, steps });
        }
    }
    Ok(())
}

const VOXEL_INDEX_GUARD_CELLS: f64 = 2.0;

fn voxel_point_is_safe(point: (f32, f32, f32), inv: f32) -> bool {
    let min = f64::from(i32::MIN) + VOXEL_INDEX_GUARD_CELLS;
    let max = f64::from(i32::MAX) - VOXEL_INDEX_GUARD_CELLS;
    [point.0, point.1, point.2].into_iter().all(|value| {
        let scaled = (value * inv).floor();
        scaled.is_finite() && (min..=max).contains(&f64::from(scaled))
    })
}

fn shadow_traversal_is_safe(origin: (f32, f32, f32), point: (f32, f32, f32), cfg: &Config) -> bool {
    let delta = [
        f64::from(point.0) - f64::from(origin.0),
        f64::from(point.1) - f64::from(origin.1),
        f64::from(point.2) - f64::from(origin.2),
    ];
    let range = delta[0].hypot(delta[1]).hypot(delta[2]);
    if range <= f64::EPSILON {
        return true;
    }

    let guard_distance = f64::from(cfg.shadow_depth) + 2.0 * f64::from(cfg.voxel_size);
    let point = [f64::from(point.0), f64::from(point.1), f64::from(point.2)];
    let min = f64::from(i32::MIN) + VOXEL_INDEX_GUARD_CELLS;
    let max = f64::from(i32::MAX) - VOXEL_INDEX_GUARD_CELLS;
    point.into_iter().zip(delta).all(|(value, direction)| {
        let extended = value + direction / range * guard_distance;
        let index = (extended / f64::from(cfg.voxel_size)).floor();
        index.is_finite() && (min..=max).contains(&index)
    })
}

#[inline]
fn world_to_voxel(x: f32, y: f32, z: f32, inv: f32) -> VoxelKey {
    (
        (x * inv).floor() as i32,
        (y * inv).floor() as i32,
        (z * inv).floor() as i32,
    )
}

/// Amanatides and Woo 3d DDA. Records in-map voxels along the ray between the
/// origin and the end of the shadow region. Voxels within the grace region of
/// the endpoint are spared from being marked as misses.
#[allow(clippy::too_many_arguments)]
fn find_misses_along_ray(
    misses: &mut AHashSet<VoxelKey>,
    map_voxels: &AHashMap<VoxelKey, Voxel>,
    origin: (f32, f32, f32),
    end: (f32, f32, f32),
    voxel_size: f32,
    shadow_depth: f32,
    grace_depth: f32,
    graze_cos: f32,
    frame: u32,
    recency_window: u32,
    origin_voxel: VoxelKey,
    endpoint: VoxelKey,
) {
    if origin_voxel == endpoint {
        return;
    }

    let (ox, oy, oz) = origin;
    let dx = end.0 - ox;
    let dy = end.1 - oy;
    let dz = end.2 - oz;

    let (mut x, mut y, mut z) = origin_voxel;

    let step_x = dx.signum() as i32;
    let step_y = dy.signum() as i32;
    let step_z = dz.signum() as i32;

    let t_max_init = |p: f32, d: f32, vox: i32, step: i32| -> f32 {
        if step == 0 {
            return f32::INFINITY;
        }
        let next_boundary = if step > 0 {
            (vox + 1) as f32 * voxel_size
        } else {
            vox as f32 * voxel_size
        };
        (next_boundary - p) / d
    };

    let mut tx = t_max_init(ox, dx, x, step_x);
    let mut ty = t_max_init(oy, dy, y, step_y);
    let mut tz = t_max_init(oz, dz, z, step_z);

    let dt_x = if step_x == 0 {
        f32::INFINITY
    } else {
        voxel_size / dx.abs()
    };
    let dt_y = if step_y == 0 {
        f32::INFINITY
    } else {
        voxel_size / dy.abs()
    };
    let dt_z = if step_z == 0 {
        f32::INFINITY
    } else {
        voxel_size / dz.abs()
    };

    let half = voxel_size * 0.5;
    let endpoint_center = (
        endpoint.0 as f32 * voxel_size + half,
        endpoint.1 as f32 * voxel_size + half,
        endpoint.2 as f32 * voxel_size + half,
    );
    let shadow_sq = shadow_depth.powi(2);
    let grace_sq = grace_depth.powi(2);

    let ray_len = (dx * dx + dy * dy + dz * dz).sqrt();
    let t_max = 1.0 + shadow_depth / ray_len.max(f32::EPSILON);
    let ray_unit = Vector3::new(dx, dy, dz) / ray_len.max(f32::EPSILON);

    let mut past_endpoint = false;
    for _ in 0..MAX_STEPS_PER_RAY {
        let t_enter = tx.min(ty).min(tz);
        if t_enter > t_max {
            return;
        }
        if t_enter >= 1.0 {
            past_endpoint = true;
        }

        if tx < ty {
            if tx < tz {
                x += step_x;
                tx += dt_x;
            } else {
                z += step_z;
                tz += dt_z;
            }
        } else if ty < tz {
            y += step_y;
            ty += dt_y;
        } else {
            z += step_z;
            tz += dt_z;
        }

        if (x, y, z) == endpoint {
            past_endpoint = true;
            continue;
        }

        let cx = x as f32 * voxel_size + half;
        let cy = y as f32 * voxel_size + half;
        let cz = z as f32 * voxel_size + half;
        let ddx = cx - endpoint_center.0;
        let ddy = cy - endpoint_center.1;
        let ddz = cz - endpoint_center.2;
        let dist_sq = ddx * ddx + ddy * ddy + ddz * ddz;

        if past_endpoint {
            // Past the endpoint, keep going until we leave the shadow region.
            if dist_sq > shadow_sq {
                return;
            }
        } else if dist_sq < grace_sq {
            // Too close to the endpoint to safely mark a miss, we might be clipping another voxel's ray.
            continue;
        }

        if let Some(c) = map_voxels.get(&(x, y, z)) {
            if !should_spare(c, ray_unit, graze_cos, frame, recency_window) {
                misses.insert((x, y, z));
            }
        }
    }
    // Reaching the defensive cap preserves all unvisited voxels. Validated
    // configurations cannot reach this branch, but direct library misuse must
    // still fail closed instead of spinning indefinitely.
}

#[cfg(test)]
mod tests {
    use super::*;

    fn basic_config() -> Config {
        Config {
            voxel_size: 1.0,
            max_range: 100.0,
            ray_subsample: 1,
            shadow_depth: 2.0,
            grace_depth: 0.0,
            min_health: 0,
            max_health: 1,
            graze_cos: 0.5,
            recency_window: 60,
            emit_every: 1,
            global_emit_every: 1,
            region_percentile: 95.0,
        }
    }

    #[test]
    fn config_rejects_unbounded_voxel_and_ray_settings() {
        let mut cfg = basic_config();
        cfg.voxel_size = MIN_VOXEL_SIZE_METERS / 10.0;
        assert!(cfg.validate().is_err());

        let mut cfg = basic_config();
        cfg.voxel_size = MIN_VOXEL_SIZE_METERS;
        cfg.max_range = 1_000.0;
        assert!(cfg.validate().is_err());

        let mut cfg = basic_config();
        cfg.max_range = 0.0;
        assert!(cfg.validate().is_ok());
    }

    #[test]
    fn conservative_step_bound_covers_axis_and_diagonal_rays() {
        let axis = conservative_ray_step_bound(50.0, 0.2, 0.1).unwrap();
        assert_eq!(axis, 873);
        assert!(axis > ((50.0_f32 + 0.2) / 0.1).ceil() as u64);

        let at_limit = conservative_ray_step_bound(
            (MAX_STEPS_PER_RAY as f64 - 11.0) * f64::from(MIN_VOXEL_SIZE_METERS) / 3.0_f64.sqrt(),
            0.0,
            f64::from(MIN_VOXEL_SIZE_METERS),
        )
        .unwrap();
        assert!(at_limit <= MAX_STEPS_PER_RAY);
    }

    #[test]
    fn find_misses_along_ray_hits_correct_voxels() {
        let voxel_size = 1.0;
        let shadow_depth = 2.0;
        let origin = (0.5, 0.5, 0.5);
        let end = (5.5, 0.5, 0.5);
        let inv = 1.0 / voxel_size;
        let origin_voxel = world_to_voxel(origin.0, origin.1, origin.2, inv);
        let endpoint = world_to_voxel(end.0, end.1, end.2, inv);

        let expected: AHashSet<VoxelKey> = [
            (1, 0, 0),
            (2, 0, 0),
            (3, 0, 0),
            (4, 0, 0),
            (6, 0, 0),
            (7, 0, 0),
        ]
        .into_iter()
        .collect();
        let mut map_voxels: AHashMap<VoxelKey, Voxel> = AHashMap::new();
        for v in &expected {
            map_voxels.insert(*v, Voxel::with_health(1));
        }

        let mut misses: AHashSet<VoxelKey> = AHashSet::new();
        find_misses_along_ray(
            &mut misses,
            &map_voxels,
            origin,
            end,
            voxel_size,
            shadow_depth,
            0.0,
            0.5,
            1,
            60,
            origin_voxel,
            endpoint,
        );

        assert_eq!(misses, expected);
    }

    #[test]
    fn batch_bounds_ignore_far_outlier() {
        let origins = [(1.0, 1.0, 0.5), (3.0, 1.0, 0.5)];
        let mut points: Vec<(f32, f32, f32)> = (0..99)
            .map(|i| {
                let a = i as f32 / 99.0 * std::f32::consts::TAU;
                (2.0 + a.cos(), 1.0 + a.sin(), (i % 10) as f32 * 0.1)
            })
            .collect();
        points.push((60.0, 1.0, 30.0));
        let (cx, cy, radius, z_min, z_max) = batch_local_bounds(&points, &origins, 95.0, 0.3);
        assert_eq!(cx, 2.0);
        assert_eq!(cy, 1.0);
        assert!(radius < 2.0, "outlier inflated radius to {radius}");
        assert!(z_max < 2.0, "outlier inflated z_max to {z_max}");
        assert!((-0.5..=0.0).contains(&z_min), "z_min out of range: {z_min}");
    }

    #[test]
    fn batch_bounds_empty_points_zero_radius() {
        let origins = [(1.0, 2.0, 3.0)];
        let (cx, cy, radius, z_min, z_max) = batch_local_bounds(&[], &origins, 95.0, 0.3);
        assert_eq!((cx, cy, radius), (1.0, 2.0, 0.0));
        assert_eq!(z_min, 3.0);
        assert_eq!(z_max, 3.0);
    }

    #[test]
    fn hits_insert_voxels() {
        let cfg = basic_config();
        let mut map = VoxelMap::default();
        update_map(
            &mut map,
            (0.0, 0.0, 0.0),
            &[(5.5, 0.5, 0.5), (0.5, 5.5, 0.5)],
            &cfg,
        );
        assert_eq!(map.health((5, 0, 0)), Some(1));
        assert_eq!(map.health((0, 5, 0)), Some(1));
        assert_eq!(map.voxels.len(), 2);
    }

    #[test]
    fn voxels_on_ray_are_removed() {
        let cfg = basic_config();
        let mut map = VoxelMap::default();
        map.set((3, 0, 0), 1);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        // The voxel on the ray should be cleared.
        assert!(!map.voxels.contains_key(&(3, 0, 0)));
        assert_eq!(map.health((5, 0, 0)), Some(1));
    }

    #[test]
    fn voxels_not_on_ray_survive() {
        let cfg = basic_config();
        let mut map = VoxelMap::default();
        map.set((3, 5, 0), 1);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        assert_eq!(map.health((3, 5, 0)), Some(1));
        assert_eq!(map.health((5, 0, 0)), Some(1));
    }

    #[test]
    fn voxels_within_shadow_region_are_removed() {
        let cfg = basic_config();
        let mut map = VoxelMap::default();
        map.set((6, 0, 0), 1);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        // The voxel inside the shadow region should be cleared.
        assert!(!map.voxels.contains_key(&(6, 0, 0)));
        assert_eq!(map.health((5, 0, 0)), Some(1));
    }

    #[test]
    fn voxels_beyond_shadow_region_survive() {
        let cfg = basic_config();
        let mut map = VoxelMap::default();
        map.set((8, 0, 0), 1);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        assert_eq!(map.health((8, 0, 0)), Some(1));
        assert_eq!(map.health((5, 0, 0)), Some(1));
    }

    #[test]
    fn hit_caught_by_other_ray_is_not_removed() {
        let cfg = basic_config();
        let mut map = VoxelMap::default();
        update_map(
            &mut map,
            (0.0, 0.0, 0.0),
            &[(3.5, 0.5, 0.5), (5.5, 0.5, 0.5)],
            &cfg,
        );
        assert_eq!(map.health((3, 0, 0)), Some(1));
        assert_eq!(map.health((5, 0, 0)), Some(1));
    }

    #[test]
    fn point_beyond_max_range_does_not_clear() {
        let cfg = Config {
            max_range: 3.0,
            ..basic_config()
        };
        let mut map = VoxelMap::default();
        map.set((3, 0, 0), 1);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        assert_eq!(map.health((3, 0, 0)), Some(1));
    }

    #[test]
    fn two_hits_needed_when_min_health_is_negative() {
        let cfg = Config {
            min_health: -1,
            ..basic_config()
        };
        let mut map = VoxelMap::default();
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        assert_eq!(map.health((5, 0, 0)), Some(0));

        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        assert_eq!(map.health((5, 0, 0)), Some(1));
    }

    /// A grazing ray along a floor must not clip floor voxels near its hit.
    #[test]
    fn ground_clipping_single_ray() {
        let voxel_size = 0.1_f32;
        let lidar_height = 1.0_f32;
        let cfg = Config {
            voxel_size,
            max_range: 50.0,
            ray_subsample: 1,
            shadow_depth: 0.2,
            grace_depth: 0.2,
            min_health: 0,
            max_health: 1,
            graze_cos: 0.5,
            recency_window: 60,
            emit_every: 1,
            global_emit_every: 1,
            region_percentile: 95.0,
        };
        // Build the floor over a y band so it is a 2d plane, not a wire.
        let max_x = 25.0_f32;
        let y_half = 0.3_f32;
        let ds = voxel_size / 3.0;
        let nx = (max_x / ds).ceil() as i32;
        let ny = (2.0 * y_half / ds).ceil() as i32;
        let floor_z = voxel_size * 0.5;
        let floor_points: Vec<(f32, f32, f32)> = (0..=nx)
            .flat_map(|i| (0..=ny).map(move |j| (i as f32 * ds, -y_half + j as f32 * ds, floor_z)))
            .collect();

        let ranges: Vec<f32> = (1..=20).map(|i| i as f32).collect();
        let mut table = format!(
            "voxel_size={voxel_size} lidar_height={lidar_height} grace={} shadow={}\n\
             range_m  ground_voxels_in_row  clipped  clipped_pct\n",
            cfg.grace_depth, cfg.shadow_depth
        );
        let mut total_clipped = 0usize;
        for &range in &ranges {
            let (mut map, _) = build_surface(&floor_points, voxel_size, cfg.max_health);
            // The ray walks the y=0, z=0 row, so only that row is ever at risk.
            let center_row: Vec<VoxelKey> = map
                .voxels
                .keys()
                .copied()
                .filter(|k| k.1 == 0 && k.2 == 0)
                .collect();
            let n_before = center_row.len();

            let origin = (0.0_f32, 0.0_f32, lidar_height);
            let points = vec![(range, 0.0_f32, 0.0_f32)];
            update_map(&mut map, origin, &points, &cfg);

            let n_after_ground = center_row
                .iter()
                .filter(|k| map.voxels.contains_key(k))
                .count();
            let clipped = n_before - n_after_ground;
            let pct = 100.0 * clipped as f32 / n_before as f32;
            table.push_str(&format!(
                "{range:>6.1}  {n_before:>20}  {clipped:>7}  {pct:>10.1}\n"
            ));
            total_clipped += clipped;
        }
        assert!(
            total_clipped == 0,
            "planar grace regressed, ground voxels clipped:\n{table}"
        );
    }

    /// Sample axis-aligned segments across a y band so each patch is a 2d surface.
    fn sample_segments(
        segments: &[(bool, f32, f32, f32)],
        voxel_size: f32,
    ) -> Vec<(f32, f32, f32)> {
        let ds = voxel_size / 6.0;
        // Sample the full step width so treads keep two in-plane directions.
        let width = 3.0 * voxel_size;
        let ny = 19;
        let mut pts = Vec::new();
        for &(vertical, fixed, lo, hi) in segments {
            let n = ((hi - lo) / ds).round().max(1.0) as i32;
            for i in 0..=n {
                let t = lo + (hi - lo) * (i as f32 / n as f32);
                for j in 0..ny {
                    let yy = width * (j as f32 / (ny - 1) as f32);
                    pts.push(if vertical {
                        (fixed, yy, t)
                    } else {
                        (t, yy, fixed)
                    });
                }
            }
        }
        pts
    }

    /// Build a map by accumulating sampled returns and marking each touched
    /// voxel occupied. Returns the map and the sorted unique voxel keys.
    fn build_surface(
        lidar: &[(f32, f32, f32)],
        voxel_size: f32,
        health: VoxelHealth,
    ) -> (VoxelMap, Vec<VoxelKey>) {
        let inv = 1.0 / voxel_size;
        let mut map = VoxelMap::default();
        for &p in lidar {
            map.accumulate(p, voxel_size);
        }
        let mut keys: Vec<VoxelKey> = lidar
            .iter()
            .map(|&(x, y, z)| world_to_voxel(x, y, z, inv))
            .collect();
        keys.sort();
        keys.dedup();
        for &k in &keys {
            map.voxels.get_mut(&k).unwrap().health = health;
        }
        map.recompute_all_normals(voxel_size);
        (map, keys)
    }

    /// Nearest forward intersection (t > 0) of a ray with the segments, as an
    /// x-z point.
    fn nearest_hit(
        origin: (f32, f32, f32),
        d: (f32, f32),
        segments: &[(bool, f32, f32, f32)],
    ) -> Option<(f32, f32)> {
        let mut best: Option<(f32, (f32, f32))> = None;
        for &(vertical, fixed, lo, hi) in segments {
            let hit = if vertical {
                if d.0.abs() < 1e-9 {
                    continue;
                }
                let t = (fixed - origin.0) / d.0;
                let z = origin.2 + t * d.1;
                (t > 1e-4 && z >= lo && z <= hi).then_some((t, (fixed, z)))
            } else {
                if d.1.abs() < 1e-9 {
                    continue;
                }
                let t = (fixed - origin.2) / d.1;
                let x = origin.0 + t * d.0;
                (t > 1e-4 && x >= lo && x <= hi).then_some((t, (x, fixed)))
            };
            if let Some(cand) = hit {
                if best.is_none_or(|b| cand.0 < b.0) {
                    best = Some(cand);
                }
            }
        }
        best.map(|(_, p)| p)
    }

    /// A ray fan from the foot of a staircase grazes lower steps en route to
    /// upper ones. The grazing gate must leave every planar surface voxel intact.
    #[test]
    fn stair_clipping_ray_fan() {
        let voxel_size = 0.1_f32;
        let half = voxel_size * 0.5;
        let cfg = Config {
            voxel_size,
            max_range: 50.0,
            ray_subsample: 1,
            shadow_depth: 0.2,
            grace_depth: 0.2,
            min_health: 0,
            max_health: 1,
            graze_cos: 0.5,
            recency_window: 60,
            emit_every: 1,
            global_emit_every: 1,
            region_percentile: 95.0,
        };

        // Staircase
        const N: i32 = 5;
        let run = 3.0 * voxel_size;
        let rise = 2.0 * voxel_size;
        let first_riser_x = 3.0 * voxel_size + half;
        let base_z = half;
        let mut segments: Vec<(bool, f32, f32, f32)> = Vec::new();
        for k in 1..=N {
            let rx = first_riser_x + (k - 1) as f32 * run;
            let zb = base_z + (k - 1) as f32 * rise;
            let zt = base_z + k as f32 * rise;
            segments.push((true, rx, zb, zt));
            segments.push((false, zt, rx, rx + run));
        }

        let lidar = sample_segments(&segments, voxel_size);
        let (mut map, all_stairs) = build_surface(&lidar, voxel_size, cfg.max_health);

        // Voxels with a normal must be spared. Only edge voxels with no plane may clear.
        let planar: Vec<VoxelKey> = all_stairs
            .iter()
            .copied()
            .filter(|k| map.voxels.get(k).and_then(Voxel::planar_normal).is_some())
            .collect();

        let origin = (half, half, base_z + 0.23);

        // A ray fan sweeping up the staircase.
        const N_RAYS: usize = 6;
        let (lo_deg, hi_deg) = (0.0_f32, 27.0_f32);
        let mut hits: Vec<(f32, f32, f32)> = Vec::new();
        for i in 0..N_RAYS {
            let frac = i as f32 / (N_RAYS - 1) as f32;
            let theta = (lo_deg + (hi_deg - lo_deg) * frac).to_radians();
            if let Some((hx, hz)) = nearest_hit(origin, (theta.cos(), theta.sin()), &segments) {
                hits.push((hx, half, hz));
            }
        }

        update_map(&mut map, origin, &hits, &cfg);

        let cleared_planar: Vec<VoxelKey> = planar
            .iter()
            .copied()
            .filter(|v| !map.voxels.contains_key(v))
            .collect();
        assert!(
            cleared_planar.is_empty(),
            "grazing rays eroded {} planar surface voxel(s): {cleared_planar:?}",
            cleared_planar.len()
        );
    }

    /// A flat landing floor with a far wall, scanned by a downward ray fan. The
    /// grazing gate must not erode the floor.
    #[test]
    fn landing_floor_ray_fan() {
        let voxel_size = 0.1_f32;
        let half = voxel_size * 0.5;
        let cfg = Config {
            voxel_size,
            max_range: 50.0,
            ray_subsample: 1,
            shadow_depth: 0.2,
            grace_depth: 0.2,
            min_health: 0,
            max_health: 1,
            graze_cos: 0.5,
            recency_window: 60,
            emit_every: 1,
            global_emit_every: 1,
            region_percentile: 95.0,
        };

        // Flat floor from the sensor out to a vertical wall.
        let floor_z = half;
        let x_wall = 25.0 * voxel_size + half;
        let segments = vec![
            (false, floor_z, half, x_wall),         // floor
            (true, x_wall, floor_z, floor_z + 1.0), // wall
        ];

        let lidar = sample_segments(&segments, voxel_size);
        let (mut map, all_surf) = build_surface(&lidar, voxel_size, cfg.max_health);

        // Sensor above the floor, so grazing rays skim it on the way to the wall.
        const SENSOR_HEIGHT: f32 = 0.3;
        let origin = (half, half, floor_z + SENSOR_HEIGHT);

        let floor: Vec<VoxelKey> = all_surf.iter().copied().filter(|k| k.2 == 0).collect();

        const N_RAYS: usize = 16;
        let (lo_deg, hi_deg) = (-35.0_f32, 18.0_f32);
        let mut hits: Vec<(f32, f32, f32)> = Vec::new();
        for i in 0..N_RAYS {
            let frac = i as f32 / (N_RAYS - 1) as f32;
            let theta = (lo_deg + (hi_deg - lo_deg) * frac).to_radians();
            if let Some((hx, hz)) = nearest_hit(origin, (theta.cos(), theta.sin()), &segments) {
                hits.push((hx, half, hz));
            }
        }

        update_map(&mut map, origin, &hits, &cfg);

        let cleared: Vec<VoxelKey> = floor
            .iter()
            .copied()
            .filter(|v| !map.voxels.contains_key(v))
            .collect();
        assert!(
            cleared.is_empty(),
            "ray fan cleared {} floor voxel(s): {cleared:?}",
            cleared.len()
        );
    }

    /// A landing seen edge-on from just below must survive the grazing rays.
    #[test]
    fn landing_grazed_from_below() {
        let voxel_size = 0.1_f32;
        let half = voxel_size * 0.5;
        let cfg = |graze_cos| Config {
            voxel_size,
            max_range: 50.0,
            ray_subsample: 1,
            shadow_depth: 0.2,
            grace_depth: 0.2,
            min_health: 0,
            max_health: 1,
            graze_cos,
            recency_window: 60,
            emit_every: 1,
            global_emit_every: 1,
            region_percentile: 95.0,
        };

        // Staircase topped by a flat landing and a back wall.
        const N: i32 = 5;
        let run = 3.0 * voxel_size;
        let rise = 2.0 * voxel_size;
        let first_riser_x = 3.0 * voxel_size + half;
        let base_z = half;
        let mut segments: Vec<(bool, f32, f32, f32)> = Vec::new();
        for k in 1..=N {
            let rx = first_riser_x + (k - 1) as f32 * run;
            let zb = base_z + (k - 1) as f32 * rise;
            let zt = base_z + k as f32 * rise;
            segments.push((true, rx, zb, zt));
            if k < N {
                segments.push((false, zt, rx, rx + run));
            }
        }
        let z_top = base_z + N as f32 * rise;
        let landing_x0 = first_riser_x + (N - 1) as f32 * run;
        segments.push((false, z_top, landing_x0, landing_x0 + 1.0));
        segments.push((true, landing_x0 + 1.0, z_top, z_top + 1.0));

        let lidar = sample_segments(&segments, voxel_size);
        let landing_row = (z_top / voxel_size).floor() as i32;

        let step_below_x = first_riser_x + (N - 2) as f32 * run + run * 0.5;
        let origin = (step_below_x, half, z_top - rise + 0.3);
        const N_RAYS: usize = 16;
        let (lo_deg, hi_deg) = (-38.0_f32, -2.0_f32);
        let mut hits: Vec<(f32, f32, f32)> = Vec::new();
        for i in 0..N_RAYS {
            let frac = i as f32 / (N_RAYS - 1) as f32;
            let theta = (lo_deg + (hi_deg - lo_deg) * frac).to_radians();
            if let Some((hx, hz)) = nearest_hit(origin, (theta.cos(), theta.sin()), &segments) {
                hits.push((hx, half, hz));
            }
        }

        let (mut map, surf) = build_surface(&lidar, voxel_size, 1);
        update_map(&mut map, origin, &hits, &cfg(0.7));

        let cleared: Vec<VoxelKey> = surf
            .iter()
            .copied()
            .filter(|k| k.2 == landing_row && !map.voxels.contains_key(k))
            .collect();
        assert!(
            cleared.is_empty(),
            "landing must survive when the robot can see over it, cleared {cleared:?}"
        );
    }

    #[test]
    fn two_misses_needed_when_max_health_is_two() {
        let cfg = Config {
            max_health: 2,
            ..basic_config()
        };
        let mut map = VoxelMap::default();
        update_map(&mut map, (0.0, 0.0, 0.0), &[(3.5, 0.5, 0.5)], &cfg);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(3.5, 0.5, 0.5)], &cfg);
        assert_eq!(map.health((3, 0, 0)), Some(2));

        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        assert_eq!(map.health((3, 0, 0)), Some(1));

        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        assert!(!map.voxels.contains_key(&(3, 0, 0)));
    }

    #[test]
    fn planar_patch_yields_vertical_normal() {
        let mut v = Voxel::default();
        for i in 0..8 {
            for j in 0..8 {
                let x = 0.09 * (i as f32 / 7.0 - 0.5);
                let y = 0.09 * (j as f32 / 7.0 - 0.5);
                v.observe(Vector3::new(x, y, 0.0));
            }
        }
        let n = v
            .self_normal()
            .expect("a flat 2d patch must yield a normal");
        assert!(n[2].abs() > 0.99, "expected ~vertical normal, got {n:?}");
    }

    #[test]
    fn line_like_patch_has_no_normal() {
        // A scan-line is not planar, so it gets no normal.
        let mut v = Voxel::default();
        for j in 0..20 {
            let y = 0.08 * (j as f32 / 19.0 - 0.5);
            let z = 0.003 * ((j % 3) - 1) as f32;
            v.observe(Vector3::new(0.0, y, z));
        }
        assert!(
            v.self_normal().is_none(),
            "a scan-line has no trustworthy normal"
        );
    }

    /// A grazing ray spares a fresh floor but clears it once stale.
    #[test]
    fn stale_planar_voxel_loses_its_spare() {
        let voxel_size = 0.1_f32;
        let y_half = 0.3_f32;
        let ds = voxel_size / 3.0;
        let nx = (20.0 / ds).ceil() as i32;
        let ny = (2.0 * y_half / ds).ceil() as i32;
        let floor_z = voxel_size * 0.5;
        let floor: Vec<(f32, f32, f32)> = (0..=nx)
            .flat_map(|i| (0..=ny).map(move |j| (i as f32 * ds, -y_half + j as f32 * ds, floor_z)))
            .collect();
        let origin = (0.0_f32, 0.0_f32, 0.35_f32);
        let ray = vec![(8.0_f32, 0.0, 0.0)];

        let clipped = |recency_window| {
            let cfg = Config {
                voxel_size,
                max_range: 50.0,
                ray_subsample: 1,
                shadow_depth: 0.2,
                grace_depth: 0.2,
                min_health: 0,
                max_health: 1,
                graze_cos: 0.5,
                recency_window,
                emit_every: 1,
                global_emit_every: 1,
                region_percentile: 95.0,
            };
            let (mut map, _) = build_surface(&floor, voxel_size, cfg.max_health);
            let row: Vec<VoxelKey> = map
                .voxels
                .keys()
                .copied()
                .filter(|k| k.1 == 0 && k.2 == 0)
                .collect();
            update_map(&mut map, origin, &ray, &cfg);
            row.iter().filter(|k| !map.voxels.contains_key(k)).count()
        };

        assert_eq!(clipped(60), 0, "a fresh floor keeps its grazing spare");
        assert!(clipped(0) > 0, "a stale floor loses its spare and clips");
    }
}
