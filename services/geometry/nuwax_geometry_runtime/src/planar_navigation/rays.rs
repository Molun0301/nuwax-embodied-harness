//! 派生局部观测使用真实射线原点；候选预算耗尽时整体报错，不安装半帧。
use nuwax_slam_2d_runtime::input::NavigationRay;

#[cfg(test)]
pub(super) fn trace_rays(
    grid: &mut nuwax_mapping::OccupancyGrid,
    rays: &[NavigationRay],
    pose: [f64; 3],
) -> Result<(), String> {
    project_updates(grid.width, grid.height, grid.resolution, grid.origin, rays, pose, |cell, value| grid.data[cell] = value)
}

/// 全图和局部块共用同一格坐标运算，避免窗口移动造成浮点边界偏移。
pub(super) fn project_updates(
    width: u32,
    height: u32,
    resolution: f64,
    origin: [f64; 3],
    rays: &[NavigationRay],
    pose: [f64; 3],
    mut write: impl FnMut(usize, i8),
) -> Result<(), String> {
    let (sin, cos) = pose[2].sin_cos();
    let world = |p: [f64; 2]| {
        [
            pose[0] + cos * p[0] - sin * p[1],
            pose[1] + sin * p[0] + cos * p[1],
        ]
    };
    let coords = |p: [f64; 2]| {
        [
            (p[0] - origin[0]) / resolution,
            (p[1] - origin[1]) / resolution,
        ]
    };
    let index = |p: [f64; 2]| {
        let x = p[0].floor() as i64;
        let y = p[1].floor() as i64;
        (x >= 0 && y >= 0 && x < i64::from(width) && y < i64::from(height))
            .then(|| y as usize * width as usize + x as usize)
    };
    let mut endpoints = Vec::new();
    let mut remaining = 8_000_000_usize;
    for ray in rays {
        let from = coords(world(ray.origin_base_m));
        let to = coords(world(ray.endpoint_base_m));
        if from.iter().chain(to.iter()).any(|v| !v.is_finite()) {
            return Err("二维射线投影越界".into());
        }
        if ray.trace_free {
            trace_cells(from, to, width, height, &mut remaining, |cell| {
                write(cell, 0)
            })?;
        }
        if ray.hit {
            if let Some(cell) = index(to) {
                endpoints.push(cell);
            }
        }
    }
    // 同一扫描的命中集合统一最后写入，另一光束不能擦掉已观察的障碍。
    for cell in endpoints {
        write(cell, 100);
    }
    Ok(())
}

fn trace_cells(
    from: [f64; 2],
    to: [f64; 2],
    width: u32,
    height: u32,
    remaining: &mut usize,
    mut visit: impl FnMut(usize),
) -> Result<(), String> {
    if width == 0 || height == 0 {
        return Ok(());
    }
    let delta = [to[0] - from[0], to[1] - from[1]];
    let sizes = [f64::from(width), f64::from(height)];
    let mut low = 0.0_f64;
    let mut high = 1.0_f64;
    for i in 0..2 {
        if delta[i] == 0.0 {
            if from[i] < 0.0 || from[i] >= sizes[i] {
                return Ok(());
            }
        } else {
            let a = -from[i] / delta[i];
            let b = (sizes[i] - from[i]) / delta[i];
            low = low.max(a.min(b));
            high = high.min(a.max(b));
        }
    }
    if low >= high {
        return Ok(());
    }
    let nudge = (1e-9 / delta[0].abs().max(delta[1].abs()).max(1.0)).min((high - low) * 0.25);
    let start = low + nudge;
    let end = high - nudge;
    let mut cell = [
        (from[0] + start * delta[0]).floor() as i64,
        (from[1] + start * delta[1]).floor() as i64,
    ];
    let step = delta.map(|d| {
        if d > 0.0 {
            1_i64
        } else if d < 0.0 {
            -1
        } else {
            0
        }
    });
    let stride = delta.map(|d| {
        if d == 0.0 {
            f64::INFINITY
        } else {
            1.0 / d.abs()
        }
    });
    let mut next = std::array::from_fn::<_, 2, _>(|i| {
        if delta[i] == 0.0 {
            f64::INFINITY
        } else {
            ((cell[i] + i64::from(step[i] > 0)) as f64 - from[i]) / delta[i]
        }
    });
    for _ in 0..u64::from(width) + u64::from(height) + 2 {
        if cell[0] < 0 || cell[1] < 0 || cell[0] >= i64::from(width) || cell[1] >= i64::from(height)
        {
            return Ok(());
        }
        if *remaining == 0 {
            return Err("二维派生射线超出单帧计算预算".into());
        }
        *remaining -= 1;
        visit(cell[1] as usize * width as usize + cell[0] as usize);
        let advance = next[0].min(next[1]);
        if advance > end {
            return Ok(());
        }
        for axis in 0..2 {
            if next[axis] <= advance + 1e-12 {
                cell[axis] += step[axis];
                next[axis] += stride[axis];
            }
        }
    }
    Err("二维射线遍历未在地图边界内结束".into())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn moved_ray_origin_preserves_unobserved_old_origin_and_no_return_endpoint_is_free() {
        let mut grid = nuwax_mapping::OccupancyGrid::new(5, 3, 1.0);
        grid.data[5] = 100;
        let ray = NavigationRay {
            origin_base_m: [1.0, 0.0],
            endpoint_base_m: [3.0, 0.0],
            hit: true,
            trace_free: true,
            source_timestamp_ns: 1,
        };
        trace_rays(&mut grid, &[ray.clone()], [0.5, 1.5, 0.0]).unwrap();
        assert_eq!(grid.data[5], 100);
        assert_eq!(grid.data[6], 0);
        assert_eq!(grid.data[8], 100);
        let miss = NavigationRay { hit: false, ..ray };
        trace_rays(&mut grid, &[miss], [0.5, 1.5, 0.0]).unwrap();
        assert_eq!(grid.data[8], 0);
    }
    #[test]
    fn clipping_crossings_endpoint_precedence_and_budget_are_explicit() {
        let mut grid = nuwax_mapping::OccupancyGrid::new(4, 4, 1.0);
        let hit = NavigationRay {
            origin_base_m: [-2.0, 1.5],
            endpoint_base_m: [1.5, 1.5],
            hit: true,
            trace_free: true,
            source_timestamp_ns: 1,
        };
        let miss = NavigationRay {
            endpoint_base_m: [10.0, 1.5],
            hit: false,
            ..hit.clone()
        };
        trace_rays(&mut grid, &[hit, miss], [0.0; 3]).unwrap();
        assert_eq!(grid.data[4], 0);
        assert_eq!(grid.data[5], 100);
        assert_eq!(grid.data[7], 0);
        let mut cells = Vec::new();
        trace_cells([0.2, 0.2], [3.8, 3.8], 4, 4, &mut 100, |c| cells.push(c)).unwrap();
        assert_eq!(cells, vec![0, 5, 10, 15]);
        assert!(trace_cells([0.1, 0.1], [3.9, 0.1], 4, 4, &mut 1, |_| {}).is_err());
    }
}
