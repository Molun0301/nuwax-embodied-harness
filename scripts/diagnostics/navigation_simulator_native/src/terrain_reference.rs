//! 原生环境独立读取原始区间构造固定参考，不读取导航路线或生产预测。
use std::collections::{BTreeMap, HashMap};
type Sample = (f64, f64, f64, f64, f64);

pub struct ReferenceField {
    heights: HashMap<[i32; 3], f64>,
    spacing: f64,
}
#[derive(Clone, Copy)]
struct Band {
    xy: [i32; 2],
    lo: i64,
    hi: i64,
}

fn root(parents: &mut [usize], mut at: usize) -> usize {
    while parents[at] != at {
        parents[at] = parents[parents[at]];
        at = parents[at];
    }
    at
}

fn solve(mut matrix: [[f64; 4]; 3]) -> Option<[f64; 3]> {
    for col in 0..3 {
        let pivot =
            (col..3).max_by(|&i, &j| matrix[i][col].abs().total_cmp(&matrix[j][col].abs()))?;
        if matrix[pivot][col].abs() < 1e-14 {
            return None;
        }
        matrix.swap(col, pivot);
        let divisor = matrix[col][col];
        for j in col..4 {
            matrix[col][j] /= divisor;
        }
        for i in 0..3 {
            if i != col {
                let factor = matrix[i][col];
                for j in col..4 {
                    matrix[i][j] -= factor * matrix[col][j];
                }
            }
        }
    }
    Some([matrix[0][3], matrix[1][3], matrix[2][3]])
}

impl ReferenceField {
    pub fn new(columns: &HashMap<(i32, i32), Vec<(f64, f64)>>, spacing: f64, radius: f64) -> Self {
        let mut keys: Vec<_> = columns.keys().copied().collect();
        keys.sort_unstable();
        let mut bands = Vec::new();
        let mut levels = BTreeMap::<i64, Vec<usize>>::new();
        for key in keys {
            for &(low, high) in &columns[&key] {
                let band = Band {
                    xy: [key.0, key.1],
                    lo: (low / spacing).round() as i64,
                    hi: (high / spacing).round() as i64,
                };
                let id = bands.len();
                for level in band.lo..=band.hi {
                    levels.entry(level).or_default().push(id);
                }
                bands.push(band);
            }
        }
        let mut counts = vec![0_usize; bands.len()];
        let mut heights: Vec<_> = bands
            .iter()
            .map(|b| (b.lo + b.hi) as f64 * spacing * 0.5)
            .collect();
        for (level, ids) in levels {
            let index: HashMap<_, _> = ids
                .iter()
                .enumerate()
                .map(|(i, &id)| (bands[id].xy, i))
                .collect();
            let mut parents: Vec<_> = (0..ids.len()).collect();
            for (i, &id) in ids.iter().enumerate() {
                let p = bands[id].xy;
                for (dx, dy) in [(-1, -1), (-1, 0), (-1, 1), (0, -1)] {
                    let (Some(x), Some(y)) = (p[0].checked_add(dx), p[1].checked_add(dy)) else {
                        continue;
                    };
                    if let Some(&j) = index.get(&[x, y]) {
                        let a = root(&mut parents, i);
                        let b = root(&mut parents, j);
                        parents[a.max(b)] = a.min(b);
                    }
                }
            }
            let mut components = BTreeMap::<usize, Vec<usize>>::new();
            for (i, &id) in ids.iter().enumerate() {
                components
                    .entry(root(&mut parents, i))
                    .or_default()
                    .push(id);
            }
            for members in components.values() {
                let origin = bands[members[0]].xy;
                let mut matrix = [[0.0; 4]; 3];
                let second = members.iter().map(|&i| bands[i].xy).find(|p| *p != origin);
                let spans_area = second.is_some_and(|b| {
                    members.iter().any(|&i| {
                        let p = bands[i].xy;
                        (i128::from(p[0]) - i128::from(origin[0]))
                            * (i128::from(b[1]) - i128::from(origin[1]))
                            != (i128::from(p[1]) - i128::from(origin[1]))
                                * (i128::from(b[0]) - i128::from(origin[0]))
                    })
                });
                for &id in members {
                    let band = bands[id];
                    let row = [
                        (i64::from(band.xy[0]) - i64::from(origin[0])) as f64 * spacing,
                        (i64::from(band.xy[1]) - i64::from(origin[1])) as f64 * spacing,
                        1.0,
                    ];
                    let height = (band.lo + band.hi - 2 * level) as f64 * spacing * 0.5;
                    let weight = 1.0 / (1.0 + ((band.hi - band.lo) as f64).powi(2));
                    for i in 0..3 {
                        for j in 0..3 {
                            matrix[i][j] += weight * row[i] * row[j];
                        }
                        matrix[i][3] += weight * row[i] * height;
                    }
                }
                let plane = spans_area
                    .then(|| solve(matrix))
                    .flatten()
                    .filter(|p| {
                        p.iter().all(|v| v.is_finite()) && p[0].hypot(p[1]) * 2.0 * radius > spacing
                    })
                    .unwrap_or([
                        0.0,
                        0.0,
                        if members.iter().any(|&id| bands[id].lo == bands[id].hi) {
                            // 同族精确薄面约束水平模型，不能被宽带均值替换。
                            0.0
                        } else {
                            matrix[2][3] / matrix[2][2]
                        },
                    ]);
                for &id in members {
                    if members.len() <= counts[id] {
                        continue;
                    }
                    let band = bands[id];
                    let dx = (i64::from(band.xy[0]) - i64::from(origin[0])) as f64 * spacing;
                    let dy = (i64::from(band.xy[1]) - i64::from(origin[1])) as f64 * spacing;
                    heights[id] =
                        (level as f64 * spacing + plane[0] * dx + plane[1] * dy + plane[2])
                            .clamp(band.lo as f64 * spacing, band.hi as f64 * spacing);
                    counts[id] = members.len();
                }
            }
        }
        let heights = bands
            .iter()
            .zip(heights)
            .map(|(band, height)| ([band.xy[0], band.xy[1], (band.hi - 1) as i32], height))
            .collect();
        Self { heights, spacing }
    }

    pub fn height(&self, xy: [f64; 2], low: f64, high: f64) -> Result<f64, String> {
        if low == high {
            return Ok(low);
        }
        let key = [
            (xy[0] / self.spacing).floor() as i32,
            (xy[1] / self.spacing).floor() as i32,
            ((high / self.spacing).round() as i64 - 1) as i32,
        ];
        self.heights
            .get(&key)
            .copied()
            .ok_or_else(|| format!("独立固定参考缺少原区间{key:?}"))
    }
}

/// 用独立协方差消元和二分求根核对连续区间目标；输入只来自原点云环境。
pub fn fit_intervals(samples: &[Sample], reference: f64) -> Result<[f64; 3], String> {
    let (mut sw, mut sx, mut sy, mut xx, mut yy, mut xy, mut width) =
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for &(x, y, low, high, w) in samples {
        sw += w;
        sx += w * x;
        sy += w * y;
        xx += w * x * x;
        yy += w * y * y;
        xy += w * x * y;
        width += w * (high - low).powi(2) / 12.0;
    }
    let a0 = xx - sx * sx / sw;
    let b0 = yy - sy * sy / sw;
    let c0 = xy - sx * sy / sw;
    if a0 * b0 - c0 * c0 <= 0.0 {
        return Err("独立环境的原始地面样本共线".into());
    }
    // 同一个凸目标的上下两半在参考处导数连续；精确薄面保持无偏。
    let alpha_down = if width > 0.0 { sw } else { 0.0 };
    let alpha_up = sw * width / (width + xx + yy);
    let derivative = |height: f64| {
        (if height < reference {
            alpha_down
        } else {
            alpha_up
        }) * (height - reference)
            + samples
                .iter()
                .map(|&(_, _, l, h, w)| w * (height - height.clamp(l, h)))
                .sum::<f64>()
    };
    let scalar = if derivative(reference) == 0.0 {
        reference
    } else {
        let mut low = samples.iter().map(|s| s.2).fold(reference, f64::min);
        let mut high = samples.iter().map(|s| s.3).fold(reference, f64::max);
        for _ in 0..64 {
            let mid = low + (high - low) * 0.5;
            if mid == low || mid == high {
                break;
            }
            let g = derivative(mid);
            if g == 0.0 {
                low = mid;
                high = mid;
                break;
            }
            if g < 0.0 {
                low = mid;
            } else {
                high = mid;
            }
        }
        low + (high - low) * 0.5
    };
    let (mut gx, mut gy) = (0.0, 0.0);
    for &(x, y, low, high, w) in samples {
        let residual = w * (scalar - scalar.clamp(low, high));
        gx += x * residual;
        gy += y * residual;
    }
    if width > 0.0 && gx.hypot(gy) <= width {
        return Ok([0.0, 0.0, scalar]);
    }
    let mut plane = [0.0, 0.0, scalar - reference];
    for _ in 0..128 {
        let (mut z, mut xz, mut yz) = (0.0, 0.0, 0.0);
        for &(x, y, low, high, w) in samples {
            let value =
                (plane[0] * x + plane[1] * y + plane[2]).clamp(low - reference, high - reference);
            z += w * value;
            xz += w * x * value;
            yz += w * y * value;
        }
        let solve_half = |alpha: f64| {
            let normal_mass = sw + alpha;
            let a = xx + width - sx * sx / normal_mass;
            let b = yy + width - sy * sy / normal_mass;
            let c = xy - sx * sy / normal_mass;
            let d = xz - sx * z / normal_mass;
            let e = yz - sy * z / normal_mass;
            let solve_slope = |penalty: f64| {
                let det = (a + penalty) * (b + penalty) - c * c;
                [
                    (d * (b + penalty) - e * c) / det,
                    (e * (a + penalty) - d * c) / det,
                ]
            };
            let slope = if width == 0.0 {
                solve_slope(0.0)
            } else if d.hypot(e) <= width {
                [0.0; 2]
            } else {
                let mut low = 0.0;
                let mut high = a.max(b);
                let mut v = solve_slope(high);
                while high * v[0].hypot(v[1]) < width {
                    high *= 2.0;
                    v = solve_slope(high);
                }
                for _ in 0..64 {
                    let mid = low + (high - low) * 0.5;
                    if mid == low || mid == high {
                        break;
                    }
                    let v = solve_slope(mid);
                    if mid * v[0].hypot(v[1]) < width {
                        low = mid;
                    } else {
                        high = mid;
                    }
                }
                solve_slope(low + (high - low) * 0.5)
            };
            let solution = [
                slope[0],
                slope[1],
                (z - slope[0] * sx - slope[1] * sy) / normal_mass,
            ];
            solution
        };
        let lower = solve_half(alpha_down);
        let next = if lower[2] <= 0.0 {
            lower
        } else {
            solve_half(alpha_up)
        };
        let stop = (0..3).all(|i| (next[i] - plane[i]).abs() < 1e-12);
        plane = next;
        if stop {
            break;
        }
    }
    Ok([plane[0], plane[1], reference + plane[2]])
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture() -> Vec<Sample> {
        let mut samples = Vec::new();
        for x in [-0.2, 0.0, 0.2] {
            for y in [-0.2, 0.0, 0.2] {
                let (low, high) = if x == 0.2 && y == 0.0 {
                    (0.0, 0.0)
                } else {
                    (-0.1, 0.1)
                };
                samples.push((x, y, low, high, 1.0));
            }
        }
        samples
    }
    #[test]
    fn independent_joint_solver_matches_analytic_minima_with_vanishing_conflict() {
        let base = fixture();
        for weight in [0.0, 1e-2, 1e-6, 1e-10] {
            let mut samples = base.clone();
            if weight > 0.0 {
                samples.push((-0.2, 0.2, 0.2, 0.2, weight));
            }
            let alpha = 9.0 + weight;
            let expected = (alpha * 0.05 + weight * 0.2) / (alpha + 1.0 + weight);
            let actual = fit_intervals(&samples, 0.05).unwrap();
            assert_eq!(actual[..2], [0.0, 0.0]);
            assert!(
                (actual[2] - expected).abs() < 1e-11,
                "共同区间消失时仍应收敛到同一解析水平解"
            );
        }
    }
    #[test]
    fn independent_solver_preserves_region_rise_with_one_lower_return() {
        let mut samples = Vec::new();
        for ix in -2..=2 {
            for iy in -2..=2 {
                samples.push((
                    f64::from(ix) * 0.1,
                    f64::from(iy) * 0.1,
                    -0.6,
                    if ix == 0 && iy == 0 { -0.6 } else { -0.45 },
                    1.0,
                ));
            }
        }
        let actual = fit_intervals(&samples, -0.45).unwrap();
        assert_eq!(actual[..2], [0.0, 0.0]);
        assert!(
            actual[2] > -0.46 && actual[2] < -0.45,
            "低处薄回波仍需参与，但不能抹去区域升高：{actual:?}"
        );
    }
    #[test]
    fn independent_solver_keeps_higher_support_and_anchor_transition_continuous() {
        let samples: Vec<_> = fixture()
            .into_iter()
            .map(|(x, y, _, _, w)| (x, y, 1.0, 1.05, w))
            .collect();
        let raised = fit_intervals(&samples, 0.955).unwrap();
        assert_eq!(raised[..2], [0.0, 0.0]);
        assert!(raised[2] > 0.99);
        let before = fit_intervals(&samples, 1.0 - 1e-9).unwrap();
        let after = fit_intervals(&samples, 1.0 + 1e-9).unwrap();
        assert!((after[2] - before[2]).abs() < 2.1e-9);
    }
    #[test]
    fn independent_joint_solver_keeps_world_height_origin_and_true_thin_slope() {
        let base = fixture();
        let expected = fit_intervals(&base, 0.05).unwrap();
        for offset in [1000.125, -2000.0625] {
            let moved: Vec<_> = base
                .iter()
                .map(|&(x, y, l, h, w)| (x, y, l + offset, h + offset, w))
                .collect();
            let actual = fit_intervals(&moved, 0.05 + offset).unwrap();
            assert!((actual[2] - expected[2] - offset).abs() < 1e-9);
            assert_eq!(actual[..2], expected[..2]);
        }
        let thin: Vec<_> = base
            .iter()
            .map(|&(x, y, _, _, w)| {
                let z = 0.2 * x - 0.1 * y + 0.3;
                (x, y, z, z, w)
            })
            .collect();
        let actual = fit_intervals(&thin, -0.04).unwrap();
        assert!(
            (actual[0] - 0.2).abs() < 1e-12
                && (actual[1] + 0.1).abs() < 1e-12
                && (actual[2] - 0.3).abs() < 1e-12
        );
    }
}
