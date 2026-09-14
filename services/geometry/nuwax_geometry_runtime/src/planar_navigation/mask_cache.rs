//! 只缓存禁区几何掩码，物理占据与扫描观测始终来自本次输入。

use crate::{
    forbidden_zones::{compose_effective_map, LoadedForbiddenZones, ZoneLoadStatus},
    lcm_runtime::MapState,
};

#[derive(PartialEq)]
struct Layout {
    width: u32,
    height: u32,
    resolution: u32,
    origin: [u64; 2],
    frame: String,
}

impl Layout {
    fn read(map: &MapState) -> Result<Self, String> {
        if map.width == 0
            || map.height == 0
            || !map.resolution.is_finite()
            || map.resolution <= 0.0
            || (map.width as usize).checked_mul(map.height as usize) != Some(map.data.len())
        {
            return Err("raw occupancy layout is invalid".into());
        }
        Ok(Self {
            width: map.width,
            height: map.height,
            resolution: map.resolution.to_bits(),
            origin: [map.origin_x.to_bits(), map.origin_y.to_bits()],
            frame: map.frame_id.clone(),
        })
    }
}

#[derive(Default)]
enum Mask {
    #[default]
    Free,
    All,
    Bits(Vec<u64>),
}

#[derive(Default)]
pub(super) struct ForbiddenMaskCache {
    layout: Option<Layout>,
    zones: Option<LoadedForbiddenZones>,
    mask: Mask,
    builds: u64,
}

fn same_zones(left: Option<&LoadedForbiddenZones>, right: Option<&LoadedForbiddenZones>) -> bool {
    match (left, right) {
        (None, None) => true,
        (Some(a), Some(b)) => {
            a.signature == b.signature
                && a.generation == b.generation
                && a.status == b.status
                && a.zones == b.zones
        }
        _ => false,
    }
}

impl ForbiddenMaskCache {
    pub(super) fn apply(
        &mut self,
        mut map: MapState,
        zones: Option<&LoadedForbiddenZones>,
    ) -> Result<MapState, String> {
        let layout = Layout::read(&map)?;
        if self.layout.as_ref() != Some(&layout) || !same_zones(self.zones.as_ref(), zones) {
            let mask = match zones {
                None => Mask::Free,
                Some(zones) if matches!(zones.status, ZoneLoadStatus::FailClosed(_)) => Mask::All,
                Some(zones) if !zones.zones.iter().any(|zone| zone.enabled) => Mask::Free,
                Some(zones) => {
                    // 原合成算法在全自由底图上给出纯禁区，保留所有形状及内外区域语义。
                    let mut free = map.clone();
                    free.data.fill(0);
                    let compiled = compose_effective_map(&free, zones)?;
                    if compiled.data.iter().all(|value| *value == 100) {
                        Mask::All
                    } else {
                        let mut bits = vec![0u64; map.data.len().div_ceil(64)];
                        for (index, value) in compiled.data.iter().enumerate() {
                            if *value == 100 {
                                bits[index / 64] |= 1u64 << (index % 64);
                            }
                        }
                        Mask::Bits(bits)
                    }
                }
            };
            self.mask = mask;
            self.layout = Some(layout);
            self.zones = zones.cloned();
            self.builds = self.builds.saturating_add(1);
        }
        match &self.mask {
            Mask::Free => {}
            Mask::All => map.data.fill(100),
            Mask::Bits(words) => {
                for (word_index, word) in words.iter().enumerate() {
                    let mut remaining = *word;
                    while remaining != 0 {
                        let index = word_index * 64 + remaining.trailing_zeros() as usize;
                        map.data[index] = 100;
                        remaining &= remaining - 1;
                    }
                }
            }
        }
        Ok(map)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid(width: u32, height: u32) -> MapState {
        MapState {
            width,
            height,
            resolution: 0.1,
            origin_x: -1.0,
            origin_y: -1.0,
            frame_id: "map".into(),
            revision: 7,
            timestamp_ns: 17,
            data: (0..width as usize * height as usize)
                .map(|index| [-1, 0, 100][index % 3])
                .collect(),
        }
    }
    fn zones() -> LoadedForbiddenZones {
        LoadedForbiddenZones {signature:"one".into(),generation:1,status:ZoneLoadStatus::Valid,
            zones:serde_json::from_value(serde_json::json!([
                {"id":"circle","shape":"circle","mode":"inside","center":{"x":1.0,"y":1.0},"radius_m":0.4},
                {"id":"area","shape":"rectangle","mode":"outside","vertices":[{"x":-0.8,"y":-0.8},{"x":3.0,"y":2.0}]}
            ])).unwrap() }
    }

    #[test]
    fn cached_mask_matches_full_composition_and_preserves_current_physical_values() {
        let mut cache = ForbiddenMaskCache::default();
        let raw = grid(64, 48);
        let original = raw.data.clone();
        let mut rules = zones();
        for turn in 0..4 {
            let mut current = raw.clone();
            current.timestamp_ns += turn;
            current.data.rotate_left(turn as usize * 17);
            let pointer = current.data.as_ptr();
            let expected = compose_effective_map(&current, &rules).unwrap();
            let actual = cache.apply(current, Some(&rules)).unwrap();
            assert_eq!(actual.data, expected.data);
            assert_eq!(actual.timestamp_ns, expected.timestamp_ns);
            assert_eq!(actual.revision, expected.revision);
            assert_eq!(
                actual.data.as_ptr(),
                pointer,
                "合成输出应消费原扫描缓冲，不再次复制"
            );
        }
        assert_eq!(cache.builds, 1);
        assert_eq!(raw.data, original);
        rules.zones[0].radius_m = Some(0.8);
        let expected = compose_effective_map(&raw, &rules).unwrap();
        assert_eq!(
            cache.apply(raw.clone(), Some(&rules)).unwrap().data,
            expected.data
        );
        assert_eq!(cache.builds, 2, "同签名内容改变也不能复用旧掩码");
        rules.status = ZoneLoadStatus::FailClosed("不可读取".into());
        assert!(cache
            .apply(raw.clone(), Some(&rules))
            .unwrap()
            .data
            .iter()
            .all(|value| *value == 100));
        assert_eq!(cache.apply(raw.clone(), None).unwrap().data, original);
    }

    #[test]
    fn mask_rebuilds_on_layout_or_zone_generation_and_free_path_keeps_buffer() {
        let mut cache = ForbiddenMaskCache::default();
        let mut rules = zones();
        let mut raw = grid(65, 49);
        cache.apply(raw.clone(), Some(&rules)).unwrap();
        raw.origin_x += 0.2;
        assert_eq!(
            cache.apply(raw.clone(), Some(&rules)).unwrap().data,
            compose_effective_map(&raw, &rules).unwrap().data
        );
        raw.resolution = 0.2;
        assert_eq!(
            cache.apply(raw.clone(), Some(&rules)).unwrap().data,
            compose_effective_map(&raw, &rules).unwrap().data
        );
        raw = grid(40, 30);
        cache.apply(raw.clone(), Some(&rules)).unwrap();
        rules.generation += 1;
        cache.apply(raw.clone(), Some(&rules)).unwrap();
        assert_eq!(cache.builds, 5);
        let pointer = raw.data.as_ptr();
        let output = cache.apply(raw, None).unwrap();
        assert_eq!(output.data.as_ptr(), pointer);
        let mut bad = output;
        bad.resolution = f32::NAN;
        assert!(cache.apply(bad, None).is_err());
    }

    #[test]
    #[ignore = "显式运行优化构建基准，避免普通回归依赖计时"]
    fn planar_mask_cache_release_benchmark() {
        assert!(!cfg!(debug_assertions), "基准必须使用优化构建");
        let mut raw = grid(1024, 1024);
        raw.resolution = 0.01;
        let rules = zones();
        let mut cache = ForbiddenMaskCache::default();
        cache.apply(raw.clone(), Some(&rules)).unwrap();
        for sample in 0..5 {
            let input = raw.clone();
            let start = std::time::Instant::now();
            let expected = compose_effective_map(&input, &rules).unwrap();
            let full = start.elapsed().as_secs_f64() * 1e3;
            let start = std::time::Instant::now();
            let actual = cache.apply(input, Some(&rules)).unwrap();
            let cached = start.elapsed().as_secs_f64() * 1e3;
            assert_eq!(actual.data, expected.data);
            println!(
                "{}",
                serde_json::json!({"样本":sample,"格数":raw.data.len(),"全量合成毫秒":full,"缓存合成毫秒":cached,"掩码构建次数":cache.builds})
            );
        }
    }
}
