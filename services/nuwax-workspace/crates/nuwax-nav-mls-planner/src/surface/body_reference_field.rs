//! 同一不可变占据源上的固定几何参考；不保存当前机体、查询顺序或运动许可。
use super::*;
use std::collections::BTreeMap;

#[derive(Debug, Default)]
pub(in crate::surface) struct LegacyReferenceCache {
    entry: Option<((GroundEvidence, u64), Arc<LegacyReferenceField>)>,
}

pub(in crate::surface) struct LegacyReferenceField {
    heights: AHashMap<[i32; 3], f64>,
    quantum: f64,
}

impl std::fmt::Debug for LegacyReferenceField {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LegacyReferenceField")
            .field("bands", &self.heights.len())
            .field("quantum", &self.quantum)
            .finish()
    }
}

#[derive(Clone, Copy)]
struct Band {
    xy: [i32; 2],
    low: i64,
    high: i64,
}

impl SurfaceObservations {
    pub(in crate::surface) fn legacy_reference_field(
        &self,
        evidence: GroundEvidence,
        radius: f64,
    ) -> Arc<LegacyReferenceField> {
        let key = (evidence, radius.to_bits());
        if let Some((cached, field)) = &self
            .legacy_reference_cache
            .lock()
            .unwrap_or_else(|error| error.into_inner())
            .entry
        {
            if *cached == key {
                return field.clone();
            }
        }
        // 只在冷源上计算，不持有查询缓存锁遍历地图；并发首建随后复用先发布的结果。
        let field = Arc::new(LegacyReferenceField::build(self, evidence, radius));
        let mut cache = self
            .legacy_reference_cache
            .lock()
            .unwrap_or_else(|error| error.into_inner());
        if let Some((cached, previous)) = &cache.entry {
            if *cached == key {
                return previous.clone();
            }
        }
        // 一个源只保留当前包络的参考场，避免不同诊断包络累积整图副本。
        cache.entry = Some((key, field.clone()));
        field
    }
}

impl LegacyReferenceField {
    fn build(source: &SurfaceObservations, evidence: GroundEvidence, radius: f64) -> Self {
        let quantum = f64::from(source.size_m);
        let mut columns: Vec<_> = source.navigation_columns().collect();
        columns.sort_unstable_by_key(|entry| entry.0);
        let mut bands = Vec::new();
        let mut levels = BTreeMap::<i64, Vec<usize>>::new();
        for (xy, occupied) in columns {
            let mut begin = 0;
            while begin < occupied.len() {
                let mut end = begin;
                while end + 1 < occupied.len()
                    && occupied[end].checked_add(1) == Some(occupied[end + 1])
                {
                    end += 1;
                }
                let high = i64::from(occupied[end]) + 1;
                let low = if evidence == GroundEvidence::OccupiedVolume {
                    high
                } else {
                    i64::from(occupied[begin]) + 1
                };
                let index = bands.len();
                bands.push(Band { xy, low, high });
                // 总成员数不超过原始占据格数；不枚举未观测的竖直空隙。
                for level in low..=high {
                    levels.entry(level).or_default().push(index);
                }
                begin = end + 1;
            }
        }
        let mut chosen = vec![0_usize; bands.len()];
        let mut values: Vec<_> = bands
            .iter()
            .map(|band| (band.low + band.high) as f64 * quantum * 0.5)
            .collect();
        for (level, members) in levels {
            let lookup: AHashMap<_, _> = members
                .iter()
                .enumerate()
                .map(|(i, &id)| (bands[id].xy, i))
                .collect();
            let mut visited = vec![false; members.len()];
            for seed in 0..members.len() {
                if visited[seed] {
                    continue;
                }
                let mut component = vec![seed];
                visited[seed] = true;
                let mut cursor = 0;
                while cursor < component.len() {
                    let xy = bands[members[component[cursor]]].xy;
                    cursor += 1;
                    for dx in -1..=1 {
                        for dy in -1..=1 {
                            if dx == 0 && dy == 0 {
                                continue;
                            }
                            let (Some(x), Some(y)) = (xy[0].checked_add(dx), xy[1].checked_add(dy))
                            else {
                                continue;
                            };
                            if let Some(&next) = lookup.get(&[x, y]) {
                                if !visited[next] {
                                    visited[next] = true;
                                    component.push(next);
                                }
                            }
                        }
                    }
                }
                let origin = bands[members[seed]].xy;
                let direction = component
                    .iter()
                    .map(|&i| bands[members[i]].xy)
                    .find(|xy| *xy != origin);
                let spans_area = direction.is_some_and(|second| {
                    component.iter().any(|&i| {
                        let p = bands[members[i]].xy;
                        (i128::from(p[0]) - i128::from(origin[0]))
                            * (i128::from(second[1]) - i128::from(origin[1]))
                            != (i128::from(p[1]) - i128::from(origin[1]))
                                * (i128::from(second[0]) - i128::from(origin[0]))
                    })
                });
                let (mut mass, mut sx, mut sy, mut sz, mut xx, mut yy, mut xy, mut xz, mut yz) =
                    (0., 0., 0., 0., 0., 0., 0., 0., 0.);
                for &member in &component {
                    let band = bands[members[member]];
                    let x = (i64::from(band.xy[0]) - i64::from(origin[0])) as f64 * quantum;
                    let y = (i64::from(band.xy[1]) - i64::from(origin[1])) as f64 * quantum;
                    let z = (band.low + band.high - 2 * level) as f64 * quantum * 0.5;
                    let weight = 1.0 / ((band.high - band.low) as f64).powi(2).mul_add(1.0, 1.0);
                    mass += weight;
                    sx += weight * x;
                    sy += weight * y;
                    sz += weight * z;
                    xx += weight * x * x;
                    yy += weight * y * y;
                    xy += weight * x * y;
                    xz += weight * x * z;
                    yz += weight * y * z;
                }
                let a = xx - sx * sx / mass;
                let b = yy - sy * sy / mass;
                let c = xy - sx * sy / mass;
                let determinant = a * b - c * c;
                // 斜率未分辨时只取水平模型，仍保留同一区域的加权常数项。
                // 枚举高度只负责成员关联，不能替代拟合得到的参考高度。
                // 同族的零宽薄带都处于该高度；水平模型必须满足这项精确约束。
                let exact_horizontal = component.iter().any(|&member| {
                    let band = bands[members[member]];
                    band.low == band.high
                });
                let (mut gx, mut gy, mut height) =
                    (0.0, 0.0, if exact_horizontal { 0.0 } else { sz / mass });
                if spans_area && determinant > 0.0 {
                    let d = xz - sx * sz / mass;
                    let e = yz - sy * sz / mass;
                    let x = (d * b - e * c) / determinant;
                    let y = (e * a - d * c) / determinant;
                    // 这是源格与足迹给出的几何分辨偏好，不是传感器噪声置信度。
                    if x.is_finite() && y.is_finite() && x.hypot(y) * 2.0 * radius > quantum {
                        gx = x;
                        gy = y;
                        height = (sz - gx * sx - gy * sy) / mass;
                    }
                }
                for &member in &component {
                    let id = members[member];
                    // 源高度按序处理；同样成员数保持较低的共同源高度，结果与遍历顺序无关。
                    if component.len() <= chosen[id] {
                        continue;
                    }
                    let band = bands[id];
                    let x = (i64::from(band.xy[0]) - i64::from(origin[0])) as f64 * quantum;
                    let y = (i64::from(band.xy[1]) - i64::from(origin[1])) as f64 * quantum;
                    values[id] = (level as f64 * quantum + height + gx * x + gy * y)
                        .clamp(band.low as f64 * quantum, band.high as f64 * quantum);
                    chosen[id] = component.len();
                }
            }
        }
        let heights = bands
            .iter()
            .zip(values)
            .map(|(band, height)| ([band.xy[0], band.xy[1], (band.high - 1) as i32], height))
            .collect();
        Self { heights, quantum }
    }

    pub(super) fn height(&self, point: [f64; 3], low: f64) -> Option<f64> {
        // 精确薄面及实测层不需要选择区间代表值。
        if point[2] == low {
            return Some(low);
        }
        let key = [
            (point[0] / self.quantum).floor() as i32,
            (point[1] / self.quantum).floor() as i32,
            ((point[2] / self.quantum).round() as i64 - 1) as i32,
        ];
        self.heights.get(&key).copied()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn source(
        shift: [i32; 3],
        thin: bool,
    ) -> (
        SurfaceObservations,
        BTreeMap<[i32; 3], BTreeMap<[i32; 3], SurfaceObservation>>,
    ) {
        let mut blocks = BTreeMap::<[i32; 3], BTreeMap<[i32; 3], SurfaceObservation>>::new();
        for x in -20..20 {
            for y in -20..20 {
                let values = if thin {
                    vec![if x < 0 { -1 } else { 1 }, 39]
                } else {
                    (if x < 0 { -3 } else { -5 }..=-1).collect::<Vec<_>>()
                };
                for z in values {
                    let key = [x + shift[0], y + shift[1], z + shift[2]];
                    blocks
                        .entry(key.map(|v| v.div_euclid(32)))
                        .or_default()
                        .insert(
                            key,
                            SurfaceObservation {
                                observed_ns: 0,
                                hits: 0,
                            },
                        );
                }
            }
        }
        let mut source = SurfaceObservations::new(0.05, 32).unwrap();
        for (&key, values) in &blocks {
            source.replace_block(key, values.clone()).unwrap();
        }
        source.mark_complete_geometry_source();
        (source, blocks)
    }

    #[test]
    fn common_region_reference_does_not_turn_downward_thickness_into_height() {
        let (source, _) = source([0; 3], false);
        let q = f64::from(0.05_f32);
        let field = source.legacy_reference_field(GroundEvidence::MeasuredSurface, 0.34);
        let constant = field.heights[&[-1, 0, -1]];
        assert!((-2.0 * q..=0.0).contains(&constant));
        assert!(constant > -2.0 * q, "区域常数项不能被枚举族高度代替");
        for height in field.heights.values() {
            assert_eq!(
                *height, constant,
                "同一水平高度被整片真实区间共同支持时，下向加厚不能让参考下降"
            );
        }
    }

    #[test]
    fn region_reference_keeps_thin_steps_and_distinct_floors() {
        let (source, _) = source([0; 3], true);
        let q = f64::from(0.05_f32);
        let field = source.legacy_reference_field(GroundEvidence::MeasuredSurface, 0.34);
        for (key, &height) in &field.heights {
            assert_eq!(height, (f64::from(key[2]) + 1.0) * q);
        }
        assert_eq!(field.heights[&[0, 0, 1]], 2.0 * q);
        assert_eq!(field.heights[&[0, 0, 39]], 40.0 * q);
    }

    #[test]
    fn fixed_reference_is_source_order_and_world_translation_invariant() {
        let (original, blocks) = source([0; 3], false);
        let first = original.legacy_reference_field(GroundEvidence::MeasuredSurface, 0.34);
        let mut reversed = SurfaceObservations::new(0.05, 32).unwrap();
        for (key, values) in blocks.into_iter().rev() {
            reversed.replace_block(key, values).unwrap();
        }
        let second = reversed.legacy_reference_field(GroundEvidence::MeasuredSurface, 0.34);
        assert_eq!(first.heights, second.heights);
        let shift = [2048, -4096, 512];
        let (translated, _) = source(shift, false);
        let moved = translated.legacy_reference_field(GroundEvidence::MeasuredSurface, 0.34);
        for (key, &height) in &first.heights {
            let next = std::array::from_fn(|i| key[i] + shift[i]);
            assert!(
                (moved.heights[&next] - height - f64::from(shift[2]) * first.quantum).abs() < 1e-12
            );
        }
    }

    #[test]
    fn immutable_reference_cache_survives_evidence_refresh_and_invalidates_real_membership() {
        let (mut source, blocks) = source([0; 3], false);
        let first = source.legacy_reference_field(GroundEvidence::MeasuredSurface, 0.34);
        let old = source.clone();
        let private = source.fork_for_planning();
        assert!(Arc::ptr_eq(
            &first,
            &private.legacy_reference_field(GroundEvidence::MeasuredSurface, 0.34)
        ));
        let block = [0, 0, -1];
        let mut values = blocks[&block].clone();
        for value in values.values_mut() {
            value.observed_ns = 100;
            value.hits = 4;
        }
        source.replace_block(block, values.clone()).unwrap();
        assert!(Arc::ptr_eq(
            &first,
            &source.legacy_reference_field(GroundEvidence::MeasuredSurface, 0.34)
        ));
        assert!(
            !source.is_complete_geometry_source(),
            "参考缓存不能替代完整源发布事实"
        );
        values.remove(&[0, 0, -1]);
        source.replace_block(block, values).unwrap();
        assert!(!Arc::ptr_eq(
            &first,
            &source.legacy_reference_field(GroundEvidence::MeasuredSurface, 0.34)
        ));
        assert!(Arc::ptr_eq(
            &first,
            &old.legacy_reference_field(GroundEvidence::MeasuredSurface, 0.34)
        ));
    }
}
