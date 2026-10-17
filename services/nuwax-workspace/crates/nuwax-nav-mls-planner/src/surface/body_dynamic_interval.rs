//! 动态区间目标保持连续；源参考固定，实际机体高度不进入优化锚点。
use super::*;

/// 原区间残差、固定高度偏差与各向同性倾斜正则的共同最小化。
/// 系数仅来自原样本权重、区间宽度与空间跨度，不构成碰撞或支撑许可。
pub(super) fn fit_dynamic_contact_plane(
    points: &[([f64; 3], f64, f64)],
    origin: [f64; 3],
) -> Option<(f64, f64, f64)> {
    if points.len() < 3 {
        return None;
    }
    // Thin observations use the original fit and its rank check directly;
    // constructing and inverting the same moment matrix twice adds no evidence.
    if points.iter().all(|&(point, low, _)| point[2] == low) {
        return original_fit_contact_plane(points, origin);
    }
    let mut rows = Vec::with_capacity(points.len());
    let mut matrix = [[0.0; 3]; 3];
    let (mut mass, mut uncertainty, mut span) = (0.0, 0.0, 0.0);
    for &(p, low, w) in points {
        let v = [p[0] - origin[0], p[1] - origin[1], 1.0];
        for i in 0..3 {
            for j in 0..3 {
                matrix[i][j] += w * v[i] * v[j];
            }
        }
        mass += w;
        uncertainty += w * (p[2] - low).powi(2) / 12.0;
        span += w * (v[0] * v[0] + v[1] * v[1]);
        rows.push((v, low - origin[2], p[2] - origin[2], w));
    }
    // 先保持原始样本的秩要求；正则不能为共线输入制造地面。
    let inverse = |matrix: [[f64; 3]; 3]| -> Option<[[f64; 3]; 3]> {
        let mut a = [[0.0; 6]; 3];
        for i in 0..3 {
            a[i][..3].copy_from_slice(&matrix[i]);
            a[i][i + 3] = 1.0;
        }
        for c in 0..3 {
            let p = (c..3).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
            if a[p][c].abs() < 1e-10 {
                return None;
            }
            a.swap(c, p);
            let v = a[c][c];
            for j in 0..6 {
                a[c][j] /= v;
            }
            for i in 0..3 {
                if i != c {
                    let f = a[i][c];
                    for j in 0..6 {
                        a[i][j] -= f * a[c][j];
                    }
                }
            }
        }
        Some(std::array::from_fn(|i| {
            std::array::from_fn(|j| a[i][j + 3])
        }))
    };
    inverse(matrix)?;
    if uncertainty == 0.0 {
        return original_fit_contact_plane(points, origin);
    }
    // 高于固定参考的真实支撑沿原几何强度响应；低于参考的偏差由全部样本
    // 共同约束，避免孤立低回波拖低机体。两半二次项在零点值和导数连续。
    let alpha_up = mass * uncertainty / (uncertainty + span);
    let alpha_down = mass;
    matrix[0][0] += uncertainty;
    matrix[1][1] += uncertainty;
    let scalar = scalar_interval_minimum(&rows, alpha_up, alpha_down);
    let mut slope_gradient = [0.0; 2];
    for &(v, low, high, w) in &rows {
        let residual = w * (scalar - scalar.clamp(low, high));
        for i in 0..2 {
            slope_gradient[i] += v[i] * residual;
        }
    }
    if slope_gradient[0].hypot(slope_gradient[1]) <= uncertainty {
        return Some((0.0, 0.0, origin[2] + scalar));
    }
    let surrogate = |rhs: [f64; 3], alpha: f64| {
        let height_mass = matrix[2][2] + alpha;
        let a = matrix[0][0] - matrix[0][2].powi(2) / height_mass;
        let b = matrix[1][1] - matrix[1][2].powi(2) / height_mass;
        let c = matrix[0][1] - matrix[0][2] * matrix[1][2] / height_mass;
        let d = rhs[0] - matrix[0][2] * rhs[2] / height_mass;
        let e = rhs[1] - matrix[1][2] * rhs[2] / height_mass;
        let solve = |lambda: f64| {
            let det = (a + lambda) * (b + lambda) - c * c;
            [
                (d * (b + lambda) - e * c) / det,
                (e * (a + lambda) - d * c) / det,
            ]
        };
        let slope = if d.hypot(e) <= uncertainty {
            [0.0; 2]
        } else {
            let mut lo = 0.0;
            let mut hi = a.max(b);
            let mut g = solve(hi);
            while hi * g[0].hypot(g[1]) < uncertainty {
                hi *= 2.0;
                g = solve(hi);
            }
            for _ in 0..64 {
                let mid = lo + (hi - lo) * 0.5;
                if mid == lo || mid == hi {
                    break;
                }
                let g = solve(mid);
                if mid * g[0].hypot(g[1]) < uncertainty {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            solve(lo + (hi - lo) * 0.5)
        };
        [
            slope[0],
            slope[1],
            (rhs[2] - matrix[0][2] * slope[0] - matrix[1][2] * slope[1]) / height_mass,
        ]
    };
    let mut plane = [0.0, 0.0, scalar];
    for _ in 0..128 {
        let mut rhs = [0.0; 3];
        for &(v, low, high, w) in &rows {
            let z = dot(v, plane).clamp(low, high);
            for i in 0..3 {
                rhs[i] += w * v[i] * z;
            }
        }
        let lower = surrogate(rhs, alpha_down);
        let next = if lower[2] <= 0.0 {
            lower
        } else {
            surrogate(rhs, alpha_up)
        };
        let stop = plane.iter().zip(next).all(|(a, b)| (a - b).abs() < 1e-12);
        plane = next;
        if stop {
            break;
        }
    }
    Some((plane[0], plane[1], origin[2] + plane[2]))
}

// 所有端点事件共同确定一维凸目标的精确最小点，不按区间是否相交切换算法。
fn scalar_interval_minimum(
    rows: &[([f64; 3], f64, f64, f64)],
    alpha_up: f64,
    alpha_down: f64,
) -> f64 {
    let derivative = rows
        .iter()
        .map(|&(_, low, high, w)| -w * 0.0_f64.clamp(low, high))
        .sum::<f64>();
    if derivative == 0.0 {
        return 0.0;
    }
    // 导数单调；零点导数决定极值所在半区，不按区间交集或上次解切换目标。
    let alpha = if derivative > 0.0 {
        alpha_down
    } else {
        alpha_up
    };
    let mut mass = alpha + rows.iter().map(|row| row.3).sum::<f64>();
    let mut moment = rows.iter().map(|row| row.1 * row.3).sum::<f64>();
    let mut events: Vec<_> = rows
        .iter()
        .flat_map(|&(_, low, high, w)| [(low, -w), (high, w)])
        .collect();
    events.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut cursor = 0;
    while cursor < events.len() {
        let at = events[cursor].0;
        let root = moment / mass;
        if root <= at {
            return root;
        }
        let mut change = 0.0;
        while cursor < events.len() && events[cursor].0 == at {
            change += events[cursor].1;
            cursor += 1;
        }
        mass += change;
        moment += change * at;
    }
    moment / mass
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thin_fast_path_preserves_original_fit_and_rank_rejection() {
        for origin in [[0.0, 0.0, 0.3], [10.0, -20.0, 0.6]] {
            for slope in [0.0, 0.2] {
                let points: Vec<_> = (-2..=2).flat_map(|x| (-2..=2).map(move |y| {
                    let (x, y) = (f64::from(x) * 0.1, f64::from(y) * 0.1);
                    let z = slope * x - slope * 0.5 * y + 0.3;
                    ([x, y, z], z, 1.0)
                })).collect();
                assert_eq!(fit_dynamic_contact_plane(&points, origin),
                    original_fit_contact_plane(&points, origin));
            }
        }
        for count in 0..5 {
            let points: Vec<_> = (0..count).map(|x| ([f64::from(x), 0.0, 0.3], 0.3, 1.0)).collect();
            assert_eq!(fit_dynamic_contact_plane(&points, [0.0; 3]), None);
        }
    }

    fn fixture() -> Vec<([f64; 3], f64, f64)> {
        let mut points = Vec::new();
        for x in [-0.2, 0.0, 0.2] {
            for y in [-0.2, 0.0, 0.2] {
                let (low, high) = if x == 0.2 && y == 0.0 {
                    (0.0, 0.0)
                } else {
                    (-0.1, 0.1)
                };
                points.push(([x, y, high], low, 1.0));
            }
        }
        points
    }

    #[test]
    fn weak_endpoint_cannot_jump_when_a_common_interval_disappears() {
        let points = fixture();
        let origin = [0.0, 0.0, 0.05];
        let before = fit_dynamic_contact_plane(&points, origin).unwrap();
        assert_eq!((before.0, before.1), (0.0, 0.0));
        let mut previous = f64::INFINITY;
        for weight in [1e-2, 1e-4, 1e-6, 1e-8, 1e-10] {
            let mut next = points.clone();
            next.push(([-0.2, 0.2, 0.2], 0.2, weight));
            let after = fit_dynamic_contact_plane(&next, origin).unwrap();
            let difference = (after.0 - before.0)
                .abs()
                .max((after.1 - before.1).abs())
                .max((after.2 - before.2).abs());
            assert!(difference < previous);
            previous = difference;
        }
        assert!(previous < 1e-9, "原共同/非共同分支在此反例留下有限倾斜跳变");
    }

    #[test]
    fn continuous_interval_fit_preserves_world_offsets_and_weight_scale() {
        let points = fixture();
        let origin = [0.0, 0.0, 0.05];
        let expected = fit_dynamic_contact_plane(&points, origin).unwrap();
        for shift in [[17.25, -23.75, 1000.125], [-107.5, 211.125, -2000.0625]] {
            for scale in [0.25, 1.0, 13.0] {
                let moved: Vec<_> = points
                    .iter()
                    .map(|&(p, low, w)| {
                        (
                            [p[0] + shift[0], p[1] + shift[1], p[2] + shift[2]],
                            low + shift[2],
                            w * scale,
                        )
                    })
                    .collect();
                let actual =
                    fit_dynamic_contact_plane(&moved, [shift[0], shift[1], origin[2] + shift[2]])
                        .unwrap();
                assert!(
                    (actual.0 - expected.0).abs() < 1e-9
                        && (actual.1 - expected.1).abs() < 1e-9
                        && (actual.2 - expected.2 - shift[2]).abs() < 1e-9
                );
            }
        }
    }

    #[test]
    fn continuous_interval_regularization_does_not_flatten_true_thin_planes() {
        let points: Vec<_> = fixture()
            .iter()
            .map(|&(p, _, w)| {
                let z = 0.2 * p[0] - 0.1 * p[1] + 0.3;
                ([p[0], p[1], z], z, w)
            })
            .collect();
        let actual = fit_dynamic_contact_plane(&points, [0.0, 0.0, -0.04]).unwrap();
        assert!(
            (actual.0 - 0.2).abs() < 1e-12
                && (actual.1 + 0.1).abs() < 1e-12
                && (actual.2 - 0.3).abs() < 1e-12
        );
    }

    #[test]
    fn isolated_low_return_cannot_erase_a_supported_region_rise() {
        let mut before = Vec::new();
        let mut after = Vec::new();
        for ix in -2..=2 {
            for iy in -2..=2 {
                let (x, y) = (f64::from(ix) * 0.1, f64::from(iy) * 0.1);
                before.push(([x, y, -0.5], -0.55, 1.0));
                let high = if ix == 0 && iy == 0 { -0.6 } else { -0.45 };
                after.push(([x, y, high], -0.6, 1.0));
            }
        }
        let initial = fit_dynamic_contact_plane(&before, [0.0, 0.0, -0.5]).unwrap();
        let raised = fit_dynamic_contact_plane(&after, [0.0, 0.0, -0.45]).unwrap();
        let rise = raised.2 - initial.2;
        assert!(
            rise > 0.04 && rise < 0.05,
            "固定区域升高五厘米时，一个低处薄回波仍有贡献，但不能抵消整片区域的缓升：{rise}"
        );
        assert_eq!((raised.0, raised.1), (0.0, 0.0));
    }

    #[test]
    fn shrinking_intervals_converge_with_their_source_reference() {
        // 固定参考取自同一原区间；区间缩小时它也收敛到原薄面，不能任意钉在别处。
        for width in [1e-2, 1e-4, 1e-6, 1e-8] {
            let points: Vec<_> = fixture()
                .into_iter()
                .map(|(p, _, w)| {
                    let z = 0.2 * p[0] - 0.1 * p[1] + 0.3;
                    ([p[0], p[1], z + width], z - width, w)
                })
                .collect();
            let actual = fit_dynamic_contact_plane(&points, [0.0, 0.0, 0.3 + width * 0.5]).unwrap();
            assert!((actual.2 - 0.3).abs() <= width);
            assert!((actual.0 - 0.2).abs() < 10.0 * width);
            assert!((actual.1 + 0.1).abs() < 10.0 * width);
        }
    }

    #[test]
    fn asymmetric_height_anchor_responds_to_a_higher_common_support() {
        let points: Vec<_> = fixture()
            .into_iter()
            .map(|(p, _, w)| ([p[0], p[1], 1.05], 1.0, w))
            .collect();
        let plane = fit_dynamic_contact_plane(&points, [0.0, 0.0, 0.955]).unwrap();
        assert_eq!((plane.0, plane.1), (0.0, 0.0));
        assert!(
            plane.2 > 0.99,
            "整片更高支撑不能被较低固定参考压住：{plane:?}"
        );
        let before = fit_dynamic_contact_plane(&points, [0.0, 0.0, 1.0 - 1e-9]).unwrap();
        let after = fit_dynamic_contact_plane(&points, [0.0, 0.0, 1.0 + 1e-9]).unwrap();
        assert!(
            (after.2 - before.2).abs() < 2.1e-9,
            "两半目标在零偏差处必须连续"
        );
    }
}
