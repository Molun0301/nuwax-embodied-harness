//! 两条探索链共用的传感器视域与有界遮挡查询；不产生已观察证据。
pub use nuwax_nav_types_messages::{ObservationKind, ObservationModel};
use std::collections::BTreeSet;

/// 单个待证实区域，标识只在同一地图域及分辨率内有效。
#[derive(Clone, Copy, Debug)]
pub struct ObservationTarget {
    /// 地图离散单元；三维高度必须参与去重。
    pub cell: [i32; 3],
    /// 目标单元中心的世界坐标。
    pub position: [f64; 3],
}

/// 有限视线检查得到的潜在收益；不是传感器实际完成的观察。
#[derive(Clone, Debug, Default)]
pub struct ViewEstimate {
    /// 当前候选能够看见的去重单元。
    pub cells: BTreeSet<[i32; 3]>,
    /// 实际执行的遮挡查询数。
    pub rays: usize,
    /// 候选样本是否超过当前射线预算。
    pub budget_limited: bool,
}

fn rotate(q: [f64; 4], p: [f64; 3]) -> [f64; 3] {
    let t = [
        2.0 * (q[1] * p[2] - q[2] * p[1]),
        2.0 * (q[2] * p[0] - q[0] * p[2]),
        2.0 * (q[0] * p[1] - q[1] * p[0]),
    ];
    [
        p[0] + q[3] * t[0] + q[1] * t[2] - q[2] * t[1],
        p[1] + q[3] * t[1] + q[2] * t[0] - q[0] * t[2],
        p[2] + q[3] * t[2] + q[0] * t[1] - q[1] * t[0],
    ]
}
fn inverse(q: [f64; 4]) -> [f64; 4] {
    [-q[0], -q[1], -q[2], q[3]]
}

/// 将观察目标投到已标定传感器坐标后检查视域，再按预算均匀抽取遮挡查询。
/// `visible`由地图拥有方实现，只读其物理遮挡和边界，不能把未知区域写成自由。
pub fn estimate_view(
    model: &ObservationModel,
    body_position: [f64; 3],
    body_rotation: [f64; 4],
    targets: &[ObservationTarget],
    mut visible: impl FnMut([f64; 3], [f64; 3]) -> bool,
) -> Result<ViewEstimate, String> {
    model.validate()?;
    if body_position
        .iter()
        .chain(body_rotation.iter())
        .any(|v| !v.is_finite())
        || (body_rotation.iter().map(|v| v * v).sum::<f64>() - 1.0).abs() > 1e-6
    {
        return Err("观察收益缺少有效机体姿态".into());
    }
    let offset = rotate(body_rotation, model.sensor_translation_body_m);
    let origin = std::array::from_fn(|i| body_position[i] + offset[i]);
    let mut eligible = Vec::new();
    let mut seen = BTreeSet::new();
    for target in targets {
        if target.position.iter().any(|v| !v.is_finite()) || !seen.insert(target.cell) {
            continue;
        }
        let delta = std::array::from_fn(|i| target.position[i] - origin[i]);
        let sensor = rotate(
            inverse(model.sensor_rotation_body_xyzw),
            rotate(inverse(body_rotation), delta),
        );
        let horizontal = sensor[0].hypot(sensor[1]);
        let range = horizontal.hypot(sensor[2]);
        let azimuth = sensor[1].atan2(sensor[0]);
        let elevation = sensor[2].atan2(horizontal);
        let vertical = if model.kind == ObservationKind::Planar {
            sensor[2].abs() <= 1e-5
        } else {
            elevation >= model.vertical_min_rad - 1e-9 && elevation <= model.vertical_max_rad + 1e-9
        };
        if range >= model.min_range_m
            && range <= model.max_range_m
            && azimuth.abs() <= model.horizontal_fov_rad * 0.5 + 1e-9
            && vertical
        {
            eligible.push(target);
        }
    }
    eligible.sort_by_key(|target| target.cell);
    let count = eligible.len().min(model.max_rays);
    let mut estimate = ViewEstimate {
        budget_limited: eligible.len() > count,
        ..Default::default()
    };
    for i in 0..count {
        let target = eligible[i * eligible.len() / count];
        estimate.rays += 1;
        if visible(origin, target.position) {
            estimate.cells.insert(target.cell);
        }
    }
    Ok(estimate)
}

/// 二维候选使用实际传感器扫描平面，在有界占据图中遇到首个未知格结束视线。
pub fn estimate_planar_view(
    model: &ObservationModel,
    grid: &nuwax_nav_types_messages::OccupancyGrid2D,
    body_xy: [f64; 2],
    yaw: f64,
    cells: &[[i32; 3]],
) -> Result<ViewEstimate, String> {
    if model.kind != ObservationKind::Planar {
        return Err("二维探索要求平面扫描标定".into());
    }
    let rotation = [0.0, 0.0, (yaw * 0.5).sin(), (yaw * 0.5).cos()];
    let offset = rotate(rotation, model.sensor_translation_body_m);
    let origin = [body_xy[0] + offset[0], body_xy[1] + offset[1], offset[2]];
    let normal = rotate(
        rotation,
        rotate(model.sensor_rotation_body_xyzw, [0.0, 0.0, 1.0]),
    );
    if normal[2].abs() < 0.5 {
        return Err("扫描平面不能投影为当前二维观察区域".into());
    }
    let targets: Vec<_> = cells
        .iter()
        .filter_map(|&cell| {
            let p = grid.cell_center(u32::try_from(cell[0]).ok()?, u32::try_from(cell[1]).ok()?)?;
            let z = origin[2]
                - (normal[0] * (p.x - origin[0]) + normal[1] * (p.y - origin[1])) / normal[2];
            Some(ObservationTarget {
                cell,
                position: [p.x, p.y, z],
            })
        })
        .collect();
    estimate_view(
        model,
        [body_xy[0], body_xy[1], 0.0],
        rotation,
        &targets,
        |origin, target| grid_view_is_clear(grid, [origin[0], origin[1]], [target[0], target[1]]),
    )
}

/// 精确遍历二维射线经过的格；目标可为未知，沿途必须有自由证据。
pub fn grid_view_is_clear(
    grid: &nuwax_nav_types_messages::OccupancyGrid2D,
    from: [f64; 2],
    to: [f64; 2],
) -> bool {
    use nuwax_nav_types_messages::Point2;
    let Some((x, y)) = grid.world_to_cell(Point2::new(from[0], from[1])) else {
        return false;
    };
    let Some((tx, ty)) = grid.world_to_cell(Point2::new(to[0], to[1])) else {
        return false;
    };
    let (mut x, mut y) = (i64::from(x), i64::from(y));
    let (tx, ty) = (i64::from(tx), i64::from(ty));
    let dx = to[0] - from[0];
    let dy = to[1] - from[1];
    let sx = if dx >= 0.0 { 1 } else { -1 };
    let sy = if dy >= 0.0 { 1 } else { -1 };
    let step_x = if dx.abs() > 1e-12 {
        grid.resolution / dx.abs()
    } else {
        f64::INFINITY
    };
    let step_y = if dy.abs() > 1e-12 {
        grid.resolution / dy.abs()
    } else {
        f64::INFINITY
    };
    let mut next_x = if dx.abs() > 1e-12 {
        (grid.origin.x + (x + if sx > 0 { 1 } else { 0 }) as f64 * grid.resolution - from[0]) / dx
    } else {
        f64::INFINITY
    };
    let mut next_y = if dy.abs() > 1e-12 {
        (grid.origin.y + (y + if sy > 0 { 1 } else { 0 }) as f64 * grid.resolution - from[1]) / dy
    } else {
        f64::INFINITY
    };
    let clear = |x: i64, y: i64| {
        u32::try_from(x)
            .ok()
            .zip(u32::try_from(y).ok())
            .and_then(|(x, y)| grid.get(x, y))
            .is_some_and(|cell| {
                cell.is_known_traversable()
                    || ((x, y) == (tx, ty)
                        && matches!(cell, nuwax_nav_types_messages::OccupancyCell::Unknown))
            })
    };
    for _ in 0..=(x - tx).unsigned_abs() + (y - ty).unsigned_abs() + 2 {
        if !clear(x, y) {
            return false;
        }
        if (x, y) == (tx, ty) {
            return true;
        }
        if (next_x - next_y).abs() < 1e-10 {
            if !clear(x + sx, y) || !clear(x, y + sy) {
                return false;
            }
            x += sx;
            y += sy;
            next_x += step_x;
            next_y += step_y;
        } else if next_x < next_y {
            x += sx;
            next_x += step_x;
        } else {
            y += sy;
            next_y += step_y;
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    fn model() -> ObservationModel {
        ObservationModel {
            kind: ObservationKind::Volume,
            min_range_m: 0.1,
            max_range_m: 5.0,
            horizontal_fov_rad: std::f64::consts::FRAC_PI_2,
            vertical_min_rad: -0.4,
            vertical_max_rad: 0.4,
            sensor_translation_body_m: [0.5, 0.0, 0.0],
            sensor_rotation_body_xyzw: [0.0, 0.0, 0.0, 1.0],
            max_rays: 16,
        }
    }
    #[test]
    fn planar_unknown_and_spatial_corner_occlusion_stop_visibility() {
        use nuwax_nav_types_messages::{OccupancyCell, OccupancyGrid2D, Point2};
        let mut grid =
            OccupancyGrid2D::filled(Point2::new(0.0, 0.0), 1.0, 8, 3, OccupancyCell::Free);
        grid.set(3, 1, OccupancyCell::Unknown);
        grid.set(5, 1, OccupancyCell::Unknown);
        assert!(grid_view_is_clear(&grid, [0.5, 1.5], [3.5, 1.5]));
        assert!(!grid_view_is_clear(&grid, [0.5, 1.5], [5.5, 1.5]));
        assert!(!voxel_view_is_clear(
            [0.5, 0.5, 0.5],
            [3.5, 3.5, 3.5],
            1.0,
            32,
            |cell| cell == [1, 0, 0]
        ));
        assert!(voxel_view_is_clear(
            [0.5, 0.5, 0.5],
            [3.5, 3.5, 3.5],
            1.0,
            32,
            |_| false
        ));
        assert!(!voxel_view_is_clear(
            [0.5, 0.5, 0.5],
            [3.5, 3.5, 3.5],
            1.0,
            1,
            |_| false
        ));
    }

    #[test]
    fn extrinsics_full_pose_and_occlusion_gate_information_without_creating_evidence() {
        let targets = vec![
            ObservationTarget {
                cell: [0, 2, 0],
                position: [0.0, 2.0, 0.0],
            },
            ObservationTarget {
                cell: [2, 0, 0],
                position: [2.0, 0.0, 0.0],
            },
            ObservationTarget {
                cell: [0, 2, 3],
                position: [0.0, 2.0, 3.0],
            },
        ];
        let half = std::f64::consts::FRAC_1_SQRT_2;
        let view = estimate_view(
            &model(),
            [0.0; 3],
            [0.0, 0.0, half, half],
            &targets,
            |origin, target| {
                assert!(origin[0].abs() < 1e-6 && (origin[1] - 0.5).abs() < 1e-6);
                target[1] > 1.0
            },
        )
        .unwrap();
        assert_eq!(view.cells, BTreeSet::from([[0, 2, 0]]));
        assert!(estimate_view(
            &model(),
            [0.0; 3],
            [0.0, 0.0, half, half],
            &targets,
            |_, _| false
        )
        .unwrap()
        .cells
        .is_empty());
    }
    #[test]
    fn duplicate_samples_and_query_budget_are_bounded() {
        let targets: Vec<_> = (0..100)
            .flat_map(|i| {
                [ObservationTarget {
                    cell: [i, 0, 0],
                    position: [1.0 + i as f64 * 0.01, 0.0, 0.0],
                }; 2]
            })
            .collect();
        let view = estimate_view(
            &model(),
            [0.0; 3],
            [0.0, 0.0, 0.0, 1.0],
            &targets,
            |_, _| true,
        )
        .unwrap();
        assert_eq!(view.rays, 16);
        assert_eq!(view.cells.len(), 16);
        assert!(view.budget_limited);
    }
}

/// 遍历三维视线经过的全部体素，角点同时检查相邻单元，预算不足返回不可见。
pub fn voxel_view_is_clear(
    from: [f64; 3],
    to: [f64; 3],
    size: f64,
    max_steps: usize,
    mut occupied: impl FnMut([i32; 3]) -> bool,
) -> bool {
    if !size.is_finite()
        || size <= 0.0
        || from
            .iter()
            .chain(to.iter())
            .any(|v| !v.is_finite() || (v / size).abs() > i32::MAX as f64 * 0.5)
    {
        return false;
    }
    let mut cell = from.map(|v| (v / size).floor() as i32);
    let target = to.map(|v| (v / size).floor() as i32);
    let delta: [f64; 3] = std::array::from_fn(|i| to[i] - from[i]);
    let signs = delta.map(|v| if v >= 0.0 { 1 } else { -1 });
    let steps = delta.map(|v| {
        if v.abs() > 1e-12 {
            size / v.abs()
        } else {
            f64::INFINITY
        }
    });
    let mut next: [f64; 3] = std::array::from_fn(|i| {
        if delta[i].abs() > 1e-12 {
            ((cell[i] + if signs[i] > 0 { 1 } else { 0 }) as f64 * size - from[i]) / delta[i]
        } else {
            f64::INFINITY
        }
    });
    for _ in 0..max_steps {
        if occupied(cell) {
            return false;
        }
        if cell == target {
            return true;
        }
        let time = next.into_iter().fold(f64::INFINITY, f64::min);
        let axes: Vec<_> = (0..3).filter(|&i| (next[i] - time).abs() < 1e-10).collect();
        if axes.is_empty() {
            return false;
        }
        for mask in 1..(1 << axes.len()) {
            let mut touched = cell;
            for (bit, &axis) in axes.iter().enumerate() {
                if mask & (1 << bit) != 0 {
                    touched[axis] += signs[axis];
                }
            }
            if occupied(touched) {
                return false;
            }
        }
        for axis in axes {
            cell[axis] += signs[axis];
            next[axis] += steps[axis];
        }
    }
    false
}
