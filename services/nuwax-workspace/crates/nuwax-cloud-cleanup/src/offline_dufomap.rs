//! 完成地图清理：DUFOMap 离线参数及缺少扫描时的半径离群兜底。
use ahash::AHashMap;
use nuwax_dufomap_native::Params;

/// 力度只连续缩小命中保护距离：0% 为0.40米，50% 为0.35米，100% 为0.30米。
/// 未知膨胀、分辨率与射线模型固定，避免滑条改变体素划分和量程。
pub fn params(strength_percent: u8) -> Result<Params, String> {
    if strength_percent > 100 {
        return Err("清理力度必须为0～100%".into());
    }
    Ok(Params {
        resolution: 0.10,
        inflate_unknown: 2,
        inflate_unknown_compensation: false,
        inflate_hits_dist: 0.40 - 0.001 * f32::from(strength_percent),
        sliding_window_size: 5,
        ..Params::default()
    })
}

/// 统计半径内至少两个其它点；候选来自保存地图本身，不生成自由空间或补造地面。
pub fn radius_outliers(
    points: &[[f32; 3]],
    strength: u8,
    cancelled: &impl Fn() -> bool,
) -> Result<Vec<bool>, String> {
    params(strength)?;
    let radius = 0.30 - 0.0015 * f64::from(strength);
    let key = |p: [f32; 3]| p.map(|x| (f64::from(x) / radius).floor() as i64);
    let mut buckets: AHashMap<[i64; 3], Vec<usize>> = AHashMap::new();
    for (i, &p) in points.iter().enumerate() {
        buckets.entry(key(p)).or_default().push(i);
    }
    let mut out = Vec::with_capacity(points.len());
    for (i, &p) in points.iter().enumerate() {
        if i % 4096 == 0 && cancelled() {
            return Err("清理已取消，原图未改变".into());
        }
        let k = key(p);
        let mut neighbours = 0;
        'search: for x in -1..=1 {
            for y in -1..=1 {
                for z in -1..=1 {
                    if let Some(indices) = buckets.get(&[k[0] + x, k[1] + y, k[2] + z]) {
                        for &j in indices {
                            if i != j
                                && (0..3)
                                    .map(|a| (f64::from(p[a]) - f64::from(points[j][a])).powi(2))
                                    .sum::<f64>()
                                    <= radius * radius
                            {
                                neighbours += 1;
                                if neighbours >= 2 {
                                    break 'search;
                                }
                            }
                        }
                    }
                }
            }
        }
        out.push(neighbours < 2);
    }
    Ok(out)
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn strength_mapping_is_continuous_monotonic_and_keeps_ray_model_fixed() {
        let mut previous = f32::INFINITY;
        for s in 0..=100 {
            let p = params(s).unwrap();
            assert!(p.inflate_hits_dist < previous);
            assert_eq!(p.resolution, 0.10);
            assert!(!p.inflate_unknown_compensation);
            assert_eq!(p.inflate_unknown, 2);
            assert_eq!(p.sliding_window_size, 5);
            previous = p.inflate_hits_dist;
        }
        assert!(params(101).is_err());
    }
    #[test]
    fn legacy_fallback_removes_isolated_returns_but_keeps_thin_structure() {
        let mut points: Vec<_> = (0..20).map(|i| [0., 0., i as f32 * 0.05]).collect();
        points.push([3., 3., 3.]);
        for s in [0, 50, 100] {
            let flags = radius_outliers(&points, s, &|| false).unwrap();
            assert!(flags[..20].iter().all(|v| !v));
            assert!(flags[20]);
        }
        assert!(radius_outliers(&points, 50, &|| true).is_err());
    }

    #[test]
    fn sampled_offline_scan_keeps_thin_static_objects_and_strength_masks_are_nested() {
        use nuwax_dufomap_native::DufoMap;

        // 与稀疏参数评估相同的固定2048束：细杆、薄桌面、桌腿均有逐点真值。
        let mut structures = vec![
            ([-1., -1.63, -0.13], [12., 1.63, 0.03]),
            ([-1., -1.63, 2.97], [12., 1.63, 3.13]),
            ([-1., -1.73, 0.03], [12., -1.63, 2.97]),
            ([-1., 1.63, 0.03], [12., 1.73, 2.97]),
            ([9.97, -1.63, 0.03], [10.07, 1.63, 2.97]),
            ([-1.13, -1.63, 0.03], [-1.03, 1.63, 2.97]),
            ([4.43, -1.05, 0.03], [4.49, -0.99, 1.75]),
            ([4.85, 0.80, 0.77], [5.75, 1.45, 0.83]),
        ];
        for x in [4.87, 5.65] {
            for y in [0.82, 1.35] {
                structures.push(([x, y, 0.03], [x + 0.06, y + 0.06, 0.77]));
            }
        }
        let hit = |origin: [f32; 3], direction: [f32; 3], low: [f32; 3], high: [f32; 3]| {
            let (mut near, mut far) = (0.0f32, f32::INFINITY);
            for axis in 0..3 {
                if direction[axis].abs() < 1e-9 {
                    if origin[axis] < low[axis] || origin[axis] > high[axis] {
                        return None;
                    }
                } else {
                    let a = (low[axis] - origin[axis]) / direction[axis];
                    let b = (high[axis] - origin[axis]) / direction[axis];
                    near = near.max(a.min(b));
                    far = far.min(a.max(b));
                    if near > far {
                        return None;
                    }
                }
            }
            (near > 1e-4).then_some(near)
        };
        let mut scans = Vec::new();
        let mut originals = Vec::new();
        let mut person_truth = Vec::new();
        for frame in 0..90 {
            let origin = [0.53 + 3.0 * frame as f32 / 89.0, 0.013, 0.53];
            let walker = (15..45).contains(&frame).then(|| {
                let y = -1.0 + 2.0 * (frame - 15) as f32 / 29.0 + 0.017;
                ([5.83, y - 0.2, 0.03], [6.17, y + 0.2, 1.73])
            });
            let mut points = Vec::new();
            for azimuth in 0..64 {
                for elevation in 0..32 {
                    let a = (-45.0 + 90.0 * azimuth as f32 / 63.0).to_radians();
                    let e = (-30.0 + 60.0 * elevation as f32 / 31.0).to_radians();
                    let direction = [a.cos() * e.cos(), a.sin() * e.cos(), e.sin()];
                    let static_distance = structures
                        .iter()
                        .filter_map(|&(low, high)| hit(origin, direction, low, high))
                        .fold(f32::INFINITY, f32::min);
                    let person_distance = walker
                        .and_then(|(low, high)| hit(origin, direction, low, high))
                        .unwrap_or(f32::INFINITY);
                    let distance = static_distance.min(person_distance);
                    let point =
                        std::array::from_fn(|axis| origin[axis] + direction[axis] * distance);
                    points.push(point);
                    originals.push(point);
                    person_truth.push(person_distance < static_distance);
                }
            }
            scans.push((origin, points));
        }
        let mut previous = vec![false; originals.len()];
        for strength in 0..=100 {
            let mut map = DufoMap::new(&params(strength).unwrap()).unwrap();
            let mut last_saved: Option<[f32; 3]> = None;
            for (origin, points) in &scans {
                // 匀速无旋转场景：默认v2规则由位移达到0.1米选中，先插入后统一判定。
                if last_saved.is_none_or(|last| origin[0] - last[0] >= 0.1) {
                    map.insert(points, *origin, false).unwrap();
                    last_saved = Some(*origin);
                }
            }
            map.propagate();
            let flags = map.seen_free(&originals).unwrap();
            assert!(
                flags
                    .iter()
                    .zip(&person_truth)
                    .all(|(free, person)| !free || *person),
                "力度{strength}误删静态墙面、地面、细杆、桌面或桌腿"
            );
            assert!(
                flags
                    .iter()
                    .zip(&previous)
                    .all(|(now, before)| !before || *now),
                "提高力度不能恢复已删除点"
            );
            assert!(
                flags.iter().any(|free| *free),
                "必须实际删除行人，不能空集合通过"
            );
            previous = flags;
        }
    }
}
