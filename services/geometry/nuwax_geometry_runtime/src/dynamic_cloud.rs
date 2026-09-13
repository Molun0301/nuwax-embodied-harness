//! 用后续实测射线的空闲证据清除三维地图残影；不按时间淘汰静态结构。

use ahash::{AHashMap as HashMap, AHashSet as HashSet};

use nalgebra::Point3;

use super::voxel_index::{coarse_key, VoxelIndex};
use super::{voxel_center_f64, voxel_key, VoxelKey};

const CLEAR_OBSERVATIONS: u8 = 3;
const EVIDENCE_WINDOW_NS: u64 = 2_000_000_000;
const MAX_RAYS: usize = 2048;
const MAX_RANGE_M: f64 = 30.0;
const MIN_RANGE_M: f64 = 0.5;
const SURFACE_MARGIN_M: f64 = 0.2;
const MAX_RAY_STEPS: usize = 2048;

#[derive(Default)]
pub(super) struct DynamicCloudFilter {
    misses: HashMap<VoxelKey, (u8, u64)>,
    last_timestamp_ns: u64,
    sampling_phase: usize,
    removed_keys: Vec<VoxelKey>,
}

impl DynamicCloudFilter {
    /// 当前扫描实际移除的体素，供确定性快照同步撤销；下一次扫描清空。
    pub(super) fn removed_keys(&self) -> &[VoxelKey] {
        &self.removed_keys
    }

    /// 只在原会话接纳成功后调用；同一帧的多束射线只能形成一次反证。
    #[cfg(test)]
    pub(super) fn clear_observed_free_space(
        &mut self,
        voxels: &mut VoxelIndex,
        voxel_size: f32,
        origin: [f64; 3],
        timestamp_ns: u64,
        points: &[Point3<f64>],
    ) -> usize {
        let hits = points
            .iter()
            .filter_map(|p| voxel_key([p.x, p.y, p.z], voxel_size))
            .collect();
        self.clear_observed_free_space_with_hits(
            voxels,
            voxel_size,
            origin,
            timestamp_ns,
            points,
            &hits,
        )
    }

    /// 验证后的整帧命中集由累加器共享，射线仍按原始点序和采样相位推进。
    pub(super) fn clear_observed_free_space_with_hits(
        &mut self,
        voxels: &mut VoxelIndex,
        voxel_size: f32,
        origin: [f64; 3],
        timestamp_ns: u64,
        points: &[Point3<f64>],
        hits: &HashSet<VoxelKey>,
    ) -> usize {
        self.removed_keys.clear();
        if timestamp_ns == 0
            || timestamp_ns <= self.last_timestamp_ns
            || points.is_empty()
            || voxel_key(origin, voxel_size).is_none()
        {
            return 0;
        }
        self.last_timestamp_ns = timestamp_ns;
        // 当前命中及相邻体素优先；恢复观测立即撤销此前反证。
        self.misses.retain(|key, (_, last)| {
            timestamp_ns.saturating_sub(*last) <= EVIDENCE_WINDOW_NS
                && !has_nearby_hit(hits, *key)
                && voxels.contains(key)
        });
        let stride = points.len().div_ceil(MAX_RAYS);
        let phase = self.sampling_phase % stride;
        self.sampling_phase = self.sampling_phase.wrapping_add(1);
        // 全部历史体素都被当前回波或其邻格保护时，不可能产生删除反证。
        // 先证明这一条件，可省去整帧在空空间里的重复射线遍历；采样相位仍正常推进。
        if voxels.iter().all(|key| has_nearby_hit(hits, *key)) {
            return 0;
        }
        let coarse_hits = voxels.coarse_enabled().then(|| {
            hits.iter()
                .map(|key| coarse_key(*key))
                .collect::<HashSet<_>>()
        });
        let mut free = HashSet::new();
        for point in points.iter().skip(phase).step_by(stride) {
            trace_free_voxels(
                voxels,
                hits,
                voxel_size,
                origin,
                point,
                coarse_hits.as_ref(),
                &mut free,
            );
        }
        let mut removed = 0;
        for key in free {
            let evidence = self.misses.entry(key).or_insert((0, timestamp_ns));
            evidence.0 = evidence.0.saturating_add(1);
            evidence.1 = timestamp_ns;
            if evidence.0 >= CLEAR_OBSERVATIONS {
                if voxels.remove(&key) {
                    removed += 1;
                    self.removed_keys.push(key);
                }
                self.misses.remove(&key);
            }
        }
        removed
    }
}

fn has_nearby_hit(hits: &HashSet<VoxelKey>, key: VoxelKey) -> bool {
    if hits.contains(&key) {
        return true;
    }
    (-1..=1).any(|x| {
        (-1..=1).any(|y| {
            (-1..=1).any(|z| {
                hits.contains(&(
                    key.0.saturating_add(x),
                    key.1.saturating_add(y),
                    key.2.saturating_add(z),
                ))
            })
        })
    })
}

fn trace_free_voxels(
    voxels: &VoxelIndex,
    hits: &HashSet<VoxelKey>,
    voxel_size: f32,
    origin: [f64; 3],
    point: &Point3<f64>,
    coarse_hits: Option<&HashSet<VoxelKey>>,
    free: &mut HashSet<VoxelKey>,
) {
    let delta = [
        point.x - origin[0],
        point.y - origin[1],
        point.z - origin[2],
    ];
    let range = delta[0].hypot(delta[1]).hypot(delta[2]);
    if !range.is_finite() || range <= MIN_RANGE_M + SURFACE_MARGIN_M {
        return;
    }
    let size = f64::from(voxel_size);
    let end = (range - SURFACE_MARGIN_M.max(size * 2.0)).min(MAX_RANGE_M);
    if end <= MIN_RANGE_M {
        return;
    }
    let direction = delta.map(|value| value / range);
    let Some(start) = voxel_key(origin, voxel_size) else {
        return;
    };
    let mut key = [start.0, start.1, start.2];
    let mut step = [0_i32; 3];
    let mut next = [f64::INFINITY; 3];
    let mut interval = [f64::INFINITY; 3];
    for axis in 0..3 {
        if direction[axis] != 0.0 {
            step[axis] = if direction[axis] > 0.0 { 1 } else { -1 };
            let boundary = (f64::from(key[axis]) + if step[axis] > 0 { 1.0 } else { 0.0 }) * size;
            next[axis] = ((boundary - origin[axis]) / direction[axis]).max(0.0);
            interval[axis] = size / direction[axis].abs();
        }
    }
    let mut coarse_cache = None;
    for _ in 0..MAX_RAY_STEPS {
        let cell = (key[0], key[1], key[2]);
        // 只跳过集合查询；原浮点步进、边界平局、距离和总步数完全保留。
        let occupied_block = if let Some(coarse_hits) = coarse_hits {
            let block = coarse_key(cell);
            match coarse_cache {
                Some((cached, occupied)) if cached == block => occupied,
                _ => {
                    let occupied = voxels.coarse_contains(&block) || coarse_hits.contains(&block);
                    coarse_cache = Some((block, occupied));
                    occupied
                }
            }
        } else {
            true
        };
        if occupied_block {
            // 整帧全部回波参与遮挡判断，不能把抽样遗漏的近回波当成空闲。
            if hits.contains(&cell) {
                break;
            }
            if voxels.contains(&cell) && !free.contains(&cell) {
                let center = voxel_center_f64(cell, voxel_size);
                let offset = std::array::from_fn::<_, 3, _>(|i| center[i] - origin[i]);
                let along = (0..3).map(|i| offset[i] * direction[i]).sum::<f64>();
                let perpendicular_sq = (0..3)
                    .map(|i| (offset[i] - along * direction[i]).powi(2))
                    .sum::<f64>();
                // 仅穿过体素内接球的射线算反证，擦边和回波附近保留。
                if along >= MIN_RANGE_M
                    && along + size <= end
                    && perpendicular_sq <= (size * 0.45).powi(2)
                    && !has_nearby_hit(hits, cell)
                {
                    free.insert(cell);
                }
            }
        }
        let axis = if next[0] <= next[1] && next[0] <= next[2] {
            0
        } else if next[1] <= next[2] {
            1
        } else {
            2
        };
        if next[axis] >= end {
            break;
        }
        let Some(advanced) = key[axis].checked_add(step[axis]) else {
            break;
        };
        key[axis] = advanced;
        next[axis] += interval[axis];
    }
}

#[cfg(test)]
fn trace_free_voxels_reference(
    voxels: &VoxelIndex,
    hits: &HashSet<VoxelKey>,
    voxel_size: f32,
    origin: [f64; 3],
    point: &Point3<f64>,
    free: &mut HashSet<VoxelKey>,
) {
    let delta = [
        point.x - origin[0],
        point.y - origin[1],
        point.z - origin[2],
    ];
    let range = delta[0].hypot(delta[1]).hypot(delta[2]);
    if !range.is_finite() || range <= MIN_RANGE_M + SURFACE_MARGIN_M {
        return;
    }
    let size = f64::from(voxel_size);
    let end = (range - SURFACE_MARGIN_M.max(size * 2.0)).min(MAX_RANGE_M);
    if end <= MIN_RANGE_M {
        return;
    }
    let direction = delta.map(|value| value / range);
    let Some(start) = voxel_key(origin, voxel_size) else {
        return;
    };
    let mut key = [start.0, start.1, start.2];
    let mut step = [0_i32; 3];
    let mut next = [f64::INFINITY; 3];
    let mut interval = [f64::INFINITY; 3];
    for axis in 0..3 {
        if direction[axis] != 0.0 {
            step[axis] = if direction[axis] > 0.0 { 1 } else { -1 };
            let boundary = (f64::from(key[axis]) + if step[axis] > 0 { 1.0 } else { 0.0 }) * size;
            next[axis] = ((boundary - origin[axis]) / direction[axis]).max(0.0);
            interval[axis] = size / direction[axis].abs();
        }
    }
    for _ in 0..MAX_RAY_STEPS {
        let cell = (key[0], key[1], key[2]);
        // 整帧全部回波参与遮挡判断，不能把抽样遗漏的近回波当成空闲。
        if hits.contains(&cell) {
            break;
        }
        if voxels.contains(&cell) && !free.contains(&cell) {
            let center = voxel_center_f64(cell, voxel_size);
            let offset = std::array::from_fn::<_, 3, _>(|i| center[i] - origin[i]);
            let along = (0..3).map(|i| offset[i] * direction[i]).sum::<f64>();
            let perpendicular_sq = (0..3)
                .map(|i| (offset[i] - along * direction[i]).powi(2))
                .sum::<f64>();
            // 仅穿过体素内接球的射线算反证，擦边和回波附近保留。
            if along >= MIN_RANGE_M
                && along + size <= end
                && perpendicular_sq <= (size * 0.45).powi(2)
                && !has_nearby_hit(hits, cell)
            {
                free.insert(cell);
            }
        }
        let axis = if next[0] <= next[1] && next[0] <= next[2] {
            0
        } else if next[1] <= next[2] {
            1
        } else {
            2
        };
        if next[axis] >= end {
            break;
        }
        let Some(advanced) = key[axis].checked_add(step[axis]) else {
            break;
        };
        key[axis] = advanced;
        next[axis] += interval[axis];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coarse_empty_blocks_match_original_ray_walk_exactly() {
        let mut seed = 7_u64;
        let mut draw = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            ((seed >> 32) % 161) as i32 - 80
        };
        let mut total_free = 0;
        for _ in 0..20 {
            let keys: Vec<_> = (0..2000).map(|_| (draw(), draw(), draw())).collect();
            let mut voxels: VoxelIndex = keys.iter().copied().collect();
            voxels.extend((10..60).map(|x| (x, 0, 0)));
            let mut hits: HashSet<_> = keys.iter().take(400).copied().collect();
            hits.insert((80, 0, 0));
            let coarse_hits: HashSet<_> = hits.iter().map(|key| coarse_key(*key)).collect();
            let mut points: Vec<_> = keys
                .iter()
                .take(32)
                .map(|key| Point3::from(voxel_center_f64(*key, 0.05)))
                .collect();
            points.extend([
                Point3::new(4.025, 0.025, 0.025),
                Point3::new(-50.0, -50.0, -50.0),
            ]);
            for origin in [[0.025; 3], [0.0; 3], [-0.125, 0.1, -0.2], [10.0, -8.0, 2.0]] {
                for point in &points {
                    let mut expected = HashSet::new();
                    trace_free_voxels_reference(&voxels, &hits, 0.05, origin, point, &mut expected);
                    let mut actual = HashSet::new();
                    trace_free_voxels(
                        &voxels,
                        &hits,
                        0.05,
                        origin,
                        point,
                        Some(&coarse_hits),
                        &mut actual,
                    );
                    assert_eq!(actual, expected, "粗块跳过改变了原射线证据");
                    let mut fallback = HashSet::new();
                    trace_free_voxels(&voxels, &hits, 0.05, origin, point, None, &mut fallback);
                    assert_eq!(fallback, expected);
                    total_free += actual.len();
                }
            }
        }
        assert!(total_free > 100, "夹具必须真实产生反证，不能只验证空集合");
    }

    #[test]
    fn dynamic_cloud_requires_distinct_observations_and_preserves_occlusion() {
        let mut filter = DynamicCloudFilter::default();
        let mut map = VoxelIndex::from([(40, 0, 0), (120, 0, 0), (40, 40, 0)]);
        let origin = [0.025; 3];
        let clear = vec![Point3::new(4.025, 0.025, 0.025); 20];
        for stamp in [1, 1, 0, 2] {
            assert_eq!(
                filter.clear_observed_free_space(&mut map, 0.05, origin, stamp, &clear),
                0
            );
        }
        assert_eq!(
            filter.clear_observed_free_space(&mut map, 0.05, origin, 3, &clear),
            1
        );
        assert_eq!(map, VoxelIndex::from([(120, 0, 0), (40, 40, 0)]));
    }

    #[test]
    fn dynamic_cloud_hit_resets_evidence_and_near_returns_block_far_rays() {
        let mut filter = DynamicCloudFilter::default();
        let mut map = VoxelIndex::from([(40, 0, 0)]);
        let origin = [0.025; 3];
        let clear = [Point3::new(4.025, 0.025, 0.025)];
        filter.clear_observed_free_space(&mut map, 0.05, origin, 1, &clear);
        filter.clear_observed_free_space(&mut map, 0.05, origin, 2, &clear);
        let hit = [Point3::new(2.025, 0.025, 0.025)];
        filter.clear_observed_free_space(&mut map, 0.05, origin, 3, &hit);
        for stamp in 4..7 {
            let occluded = [Point3::new(1.025, 0.025, 0.025), clear[0]];
            filter.clear_observed_free_space(&mut map, 0.05, origin, stamp, &occluded);
        }
        for stamp in 7..9 {
            assert_eq!(
                filter.clear_observed_free_space(&mut map, 0.05, origin, stamp, &clear),
                0
            );
        }
        assert_eq!(
            filter.clear_observed_free_space(&mut map, 0.05, origin, 9, &clear),
            1
        );
    }

    #[test]
    fn dynamic_cloud_protects_grazing_surface_and_expired_evidence() {
        let mut filter = DynamicCloudFilter::default();
        let mut map = VoxelIndex::from([(40, 0, 0), (-40, 0, 0)]);
        let clear = [Point3::new(4.025, 0.025, 0.025)];
        filter.clear_observed_free_space(&mut map, 0.05, [0.025; 3], 1, &clear);
        filter.clear_observed_free_space(&mut map, 0.05, [0.025; 3], 2, &clear);
        filter.clear_observed_free_space(
            &mut map,
            0.05,
            [0.025; 3],
            EVIDENCE_WINDOW_NS + 3,
            &clear,
        );
        assert_eq!(map.len(), 2);
        for stamp in EVIDENCE_WINDOW_NS + 4..EVIDENCE_WINDOW_NS + 8 {
            filter.clear_observed_free_space(
                &mut map,
                0.05,
                [0.025, 0.0, 0.0],
                stamp,
                &[Point3::new(4.025, 0.0, 0.0)],
            );
        }
        assert_eq!(map.len(), 2);
    }

    #[test]
    fn dynamic_cloud_preserves_static_walls_floor_and_unobserved_regions() {
        let origin = [0.025, 0.025, 1.025];
        let mut points = Vec::new();
        for i in -20..=20 {
            for j in 0..=20 {
                points.push(Point3::new(
                    -4.025,
                    i as f64 * 0.1 + 0.025,
                    j as f64 * 0.1 + 0.025,
                ));
                points.push(Point3::new(
                    j as f64 * 0.1 + 0.025,
                    i as f64 * 0.1 + 0.025,
                    0.025,
                ));
            }
        }
        let mut map = points
            .iter()
            .map(|p| voxel_key([p.x, p.y, p.z], 0.05).unwrap())
            .collect::<VoxelIndex>();
        let original = map.clone();
        let mut filter = DynamicCloudFilter::default();
        // 静态表面重复扫描且轻微噪声移动；未覆盖的旧结构也不得衰减。
        for stamp in 1..=8 {
            let scan = points
                .iter()
                .enumerate()
                .filter(|(i, _)| i % 2 == stamp as usize % 2)
                .map(|(_, p)| Point3::new(p.x + 0.005, p.y + 0.005, p.z + 0.005))
                .collect::<Vec<_>>();
            assert_eq!(
                filter.clear_observed_free_space(&mut map, 0.05, origin, stamp, &scan),
                0
            );
        }
        assert_eq!(map, original);
        // 负方向穿透同样能删除，超出清理范围的历史点保持。
        map.extend([(-40, 0, 20), (-800, 0, 20)]);
        for stamp in 9..=11 {
            filter.clear_observed_free_space(
                &mut map,
                0.05,
                origin,
                stamp,
                &[Point3::new(-50.025, 0.025, 1.025)],
            );
        }
        assert!(!map.contains(&(-40, 0, 20)));
        assert!(map.contains(&(-800, 0, 20)));
    }
}
