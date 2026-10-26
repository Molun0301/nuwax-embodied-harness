//! 完成地图清理：DUFOMap 离线参数及缺少扫描时的半径离群兜底。
use ahash::AHashMap;
use nuwax_dufomap_native::Params;

/// 力度只连续缩小命中保护距离：0% 为0.30米，50% 为0.25米，100% 为0.20米。
/// 未知膨胀、分辨率与射线模型固定，避免滑条改变体素划分和量程。
pub fn params(strength_percent: u8) -> Result<Params, String> {
    if strength_percent > 100 {
        return Err("清理力度必须为0～100%".into());
    }
    Ok(Params {
        resolution: 0.15,
        inflate_unknown_compensation: false,
        inflate_hits_dist: 0.30 - 0.001 * f32::from(strength_percent),
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
            assert_eq!(p.resolution, 0.15);
            assert!(!p.inflate_unknown_compensation);
            assert_eq!(p.inflate_unknown, 1);
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
}
