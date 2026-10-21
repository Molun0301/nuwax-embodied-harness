//! 将已有不可变感知转换为原生规划输入，不执行地面投影或运动许可。
use crate::SurfaceSnapshot;
use nuwax_nav_pct_native::{Band, Cell, Config};
use std::collections::{BTreeMap, BTreeSet};

/// 原地面高度与实体占据，二者保持独立。
#[derive(Clone, Debug)]
pub struct NativeMap {
    /// 原始占据分辨率。
    pub resolution: f64,
    /// 真实地面代表点，缺测列不补高度。
    pub ground: Vec<[f64; 3]>,
    /// SCAN 静态夹具的真实表面点；不重复注入实体 band 的人工暴露顶边界。
    /// PCT 的 ground/cells/bands 保持原语义；真实注册雷达帧不经过此视图。
    pub static_collision_ground: Vec<[f64; 3]>,
    /// 非地面占据的体素中心。
    pub obstacles: Vec<[f64; 3]>,
    /// 全局层析输入。
    pub cells: Vec<Cell>,
    /// 已知实体的真实高度区间。
    pub bands: Vec<Band>,
    /// 显式虚拟禁行列，不把有限遮罩外的范围额外标为占据。
    pub forbidden_columns: Vec<[i32; 2]>,
}

/// 比较静态占据、地面语义与独立地面记录，不使用地图序号猜测相等。
pub fn same_source_geometry(a: &SurfaceSnapshot, b: &SurfaceSnapshot) -> bool {
    a.same_geometry(b)
        && match (a.observations(), b.observations()) {
            (Some(a), Some(b)) => a.same_geometry(b),
            (None, None) => true,
            _ => false,
        }
}

fn band_points(band: Band, resolution: f64, points: &mut Vec<[f64; 3]>) {
    let xy = [band.x, band.y].map(|v| (f64::from(v) + 0.5) * resolution);
    let count = ((band.high - band.low) / resolution).ceil().max(1.0) as usize;
    for i in 0..count {
        points.push([
            xy[0],
            xy[1],
            band.low + (i as f64 + 0.5) * (band.high - band.low) / count as f64,
        ]);
    }
}

/// 从独立地面记录与原始点分类准备输入；纯地面缺测不形成障碍。
pub fn from_surface(map: &SurfaceSnapshot) -> Result<NativeMap, String> {
    from_surface_window(map, None)
}

/// 只转换指定局部窗口；窗口只限制当前算法输入，不改变源地图或未知语义。
pub fn from_surface_window(
    map: &SurfaceSnapshot,
    window: Option<([f64; 2], f64)>,
) -> Result<NativeMap, String> {
    let measured_surface =
        map.ground_evidence() == nuwax_nav_mls_planner::surface::GroundEvidence::MeasuredSurface;
    let occupied_volume =
        map.ground_evidence() == nuwax_nav_mls_planner::surface::GroundEvidence::OccupiedVolume;
    let (resolution, columns) = map.navigation_source_columns(window);
    let nearby = |point: &[f64; 3]| {
        window.is_none_or(|(center, radius)| {
            (point[0] - center[0]).abs() <= radius && (point[1] - center[1]).abs() <= radius
        })
    };
    let forbidden_columns = map
        .navigation_forbidden_columns(resolution)
        .into_iter()
        .filter(|column| {
            let p = [
                (f64::from(column[0]) + 0.5) * resolution,
                (f64::from(column[1]) + 0.5) * resolution,
                0.,
            ];
            nearby(&p)
        })
        .collect();
    let mut result = NativeMap {
        resolution,
        ground: Vec::new(),
        static_collision_ground: Vec::new(),
        obstacles: Vec::new(),
        cells: Vec::new(),
        bands: Vec::new(),
        forbidden_columns,
    };

    let point_resolution = f64::from(map.source_voxel_size_m());
    let mut raw = BTreeMap::<[i32; 2], (Vec<Cell>, Vec<Band>)>::new();
    if let Some(points) = map
        .navigation_measured_points()
        .filter(|_| measured_surface)
    {
        let selected = window.map(|_| points.iter().copied().filter(nearby).collect::<Vec<_>>());
        let points = selected.as_deref().unwrap_or(points);
        let (cells, bands) = nuwax_nav_pct_native::classify_points(
            &Config {
                resolution,
                point_resolution,
                ..Config::default()
            },
            points,
        )?;
        for cell in cells {
            raw.entry([cell.x, cell.y]).or_default().0.push(cell);
        }
        for band in bands {
            raw.entry([band.x, band.y]).or_default().1.push(band);
        }
    }
    if measured_surface {
        // 旧精细存档只有实际体素，未保存格内均值与独立地面记录；按原量化中心恢复这些已有测量样本。
        // 明确的地面分类（包括空列）优先，不从缺测列或已分类为空的列重新生成候选。
        let mut samples = Vec::new();
        for (column, bands) in map.navigation_source_columns(window).1 {
            if raw.contains_key(&column)
                || map.observations().is_some_and(|observations| {
                    observations.ground_records(column).next().is_some()
                        || observations.classified_empty_ground(column)
                })
            {
                continue;
            }
            let xy = column.map(|v| (f64::from(v) + 0.5) * resolution);
            for [low, high] in bands {
                let count = ((high - low) / point_resolution).round().max(1.0) as usize;
                for index in 0..count {
                    samples.push([
                        xy[0],
                        xy[1],
                        low + (index as f64 + 0.5) * (high - low) / count as f64,
                    ]);
                }
            }
        }
        if !samples.is_empty() {
            let (cells, bands) = nuwax_nav_pct_native::classify_points(
                &Config {
                    resolution,
                    point_resolution,
                    ..Config::default()
                },
                &samples,
            )?;
            for cell in cells {
                raw.entry([cell.x, cell.y]).or_default().0.push(cell);
            }
            for band in bands {
                raw.entry([band.x, band.y]).or_default().1.push(band);
            }
        }
    }
    for (column, bands) in columns {
        let xy = column.map(|v| (f64::from(v) + 0.5) * resolution);
        let records: Vec<_> = map
            .observations()
            .filter(|_| measured_surface)
            .into_iter()
            .flat_map(|o| o.ground_records(column))
            .collect();
        let classified = !records.is_empty()
            || map
                .observations()
                .is_some_and(|o| o.classified_empty_ground(column));
        if !classified {
            if let Some((cells, measured_bands)) = raw.remove(&column) {
                for cell in &cells {
                    result.ground.push([xy[0], xy[1], cell.ground]);
                    result.static_collision_ground.push([xy[0], xy[1], cell.ground]);
                }
                for band in &measured_bands {
                    // 薄表面沿原生共享分类归入地面；高实体保留整带，不生成墙中间楼层。
                    if band.high - band.low > point_resolution * 1.5 {
                        band_points(*band, resolution, &mut result.obstacles);
                    }
                }
                result.cells.extend(cells);
                result.bands.extend(measured_bands);
                continue;
            }
        }
        let mut members = BTreeSet::new();
        let mut grounds = Vec::new();
        for record in records {
            members.extend(record.members());
            if let Some(height) = record.height_at(xy) {
                grounds.push(height);
            }
        }
        // 实体体积不使用采样中位数或测量成员豁免；相邻格只合并求顶，全部占据仍保留。
        if occupied_volume && !classified {
            let mut exposed_top = None;
            for &[low, high] in &bands {
                if let Some(top) = exposed_top {
                    if low > top {
                        grounds.push(top);
                    }
                }
                exposed_top = Some(exposed_top.map_or(high, |top: f64| top.max(high)));
            }
            grounds.extend(exposed_top);
        }
        for &[low, high] in &bands {
            let first = (low / resolution).floor() as i32;
            let last = (high / resolution).ceil() as i32;
            for z in first..last.max(first + 1) {
                if members.contains(&z) {
                    continue;
                }
                let band = Band {
                    x: column[0],
                    y: column[1],
                    low: low.max(f64::from(z) * resolution),
                    high: high.min(f64::from(z + 1) * resolution),
                };
                result.bands.push(band);
                band_points(band, resolution, &mut result.obstacles);
            }
        }
        for height in grounds {
            let ceiling = result
                .bands
                .iter()
                .rev()
                .take_while(|band| [band.x, band.y] == column)
                .filter(|band| band.low > height)
                .map(|band| band.low)
                .min_by(f64::total_cmp)
                .unwrap_or(f64::INFINITY);
            result.cells.push(Cell {
                x: column[0],
                y: column[1],
                ground: height,
                ceiling,
            });
            result.ground.push([xy[0], xy[1], height]);
            if !(occupied_volume && !classified) {
                result.static_collision_ground.push([xy[0], xy[1], height]);
            }
            // 地面成员只保留真实表面，量化厚度不作为机身障碍。
            result.bands.push(Band {
                x: column[0],
                y: column[1],
                low: height,
                high: height,
            });
        }
    }
    Ok(result)
}

/// 加入原感知来源已经区分的近期地面与障碍，不重新猜测标签。
pub fn append_recent(map: &SurfaceSnapshot, input: &mut NativeMap) {
    let (ground, obstacles) = map.navigation_recent_samples();
    for p in ground {
        let x = (p[0] / input.resolution).floor() as i32;
        let y = (p[1] / input.resolution).floor() as i32;
        input.cells.push(Cell {
            x,
            y,
            ground: p[2],
            ceiling: f64::INFINITY,
        });
        input.bands.push(Band {
            x,
            y,
            low: p[2],
            high: p[2],
        });
        input.ground.push(p);
        input.static_collision_ground.push(p);
    }
    for p in obstacles {
        let x = (p[0] / input.resolution).floor() as i32;
        let y = (p[1] / input.resolution).floor() as i32;
        let half = f64::from(map.source_voxel_size_m()) * 0.5;
        input.bands.push(Band {
            x,
            y,
            low: p[2] - half,
            high: p[2] + half,
        });
        input.obstacles.push(p);
    }
}

/// 局部规划使用相同静态来源并加入当前实测点。
pub fn local_input(map: &SurfaceSnapshot) -> Result<NativeMap, String> {
    let mut input = from_surface(map)?;
    append_recent(map, &mut input);
    Ok(input)
}

/// 从真实机体配置准备全局原生内核的层与占据输入。
pub fn pct_input(
    map: &SurfaceSnapshot,
    body_to_ground: f64,
    boundary: &[[f64; 2]],
) -> Result<(Config, Vec<Cell>, Vec<Band>), String> {
    let mut input = local_input(map)?;
    for column in &input.forbidden_columns {
        input.bands.push(Band {
            x: column[0],
            y: column[1],
            low: f64::NEG_INFINITY,
            high: f64::INFINITY,
        });
    }
    let mut config = Config {
        resolution: input.resolution,
        point_resolution: f64::from(map.source_voxel_size_m()),
        robot_height: f64::from(map.config().robot_height),
        footprint_radius: f64::from(map.config().wall_clearance_m),
        body_z_min: -body_to_ground,
        body_z_max: f64::from(map.config().robot_height) - body_to_ground,
        body_to_ground,
        max_step: f64::from(map.config().step_threshold_m),
        max_step_down: f64::from(map.config().step_threshold_m),
        max_slope: f64::from(map.config().max_slope_deg).to_radians(),
        boundary: boundary.to_vec(),
        ..Config::default()
    };
    if let Some(body) = map.body_envelope() {
        config.footprint = body.footprint_xy_m.clone();
        config.footprint_radius = body
            .footprint_xy_m
            .iter()
            .map(|p| p[0].hypot(p[1]))
            .fold(0., f64::max);
        config.body_z_min = body.z_bounds_m[0];
        config.body_z_max = body.z_bounds_m[1];
        config.robot_height = body.z_bounds_m[1] + body_to_ground;
        config.max_step = body.max_step_up_m;
        config.max_step_down = body.max_step_down_m;
        config.max_slope = body.max_cross_slope_deg.to_radians();
    }
    Ok((config, input.cells, input.bands))
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_mls_planner::{surface::GroundEvidence, SurfaceObservation, SurfaceObservations};
    use std::sync::Arc;

    fn fine_column(levels: &[i32]) -> SurfaceSnapshot {
        let quantum = 0.05_f32;
        let (_, base) = crate::native_test_helpers::fixture();
        let points: Vec<_> = levels
            .iter()
            .map(|&z| (quantum * 0.5, quantum * 0.5, (z as f32 + 0.5) * quantum))
            .collect();
        let mut observations = SurfaceObservations::new(quantum, 32).unwrap();
        let mut blocks = BTreeMap::<_, BTreeMap<_, _>>::new();
        for &z in levels {
            blocks.entry([0, 0, z.div_euclid(32)]).or_default().insert(
                [0, 0, z],
                SurfaceObservation {
                    observed_ns: 1,
                    hits: 1,
                },
            );
        }
        for (block, values) in blocks {
            observations.replace_block(block, values).unwrap();
        }
        observations.mark_complete_geometry_source();
        SurfaceSnapshot::from_points(&points, base.config().clone(), quantum)
            .unwrap()
            .with_observations(Some(Arc::new(observations)))
    }

    #[test]
    fn same_raw_returns_keep_measured_height_but_volume_uses_exposed_tops() {
        let source = fine_column(&[0, 1, 2, 10, 11]);
        let measured = from_surface(&source).unwrap();
        let volume = source
            .clone()
            .with_ground_evidence(GroundEvidence::OccupiedVolume);
        let input = from_surface(&volume).unwrap();
        let q = f64::from(0.05_f32);
        assert_eq!(measured.ground.len(), 2);
        assert_eq!(measured.static_collision_ground, measured.ground);
        assert!(input.static_collision_ground.is_empty());
        assert!((measured.ground[0][2] - 0.125).abs() < 1e-8);
        assert!((measured.ground[1][2] - 0.575).abs() < 1e-7);
        assert_eq!(
            input.ground.iter().map(|p| p[2]).collect::<Vec<_>>(),
            vec![3. * q, 12. * q]
        );
        assert!(
            !same_source_geometry(&source, &volume),
            "来源含义变化必须撤销旧原生缓存"
        );

        // 完整实体按原细格保留；局部窗口和全局消费者取得同一真实暴露顶。
        let occupied: BTreeSet<_> = input
            .obstacles
            .iter()
            .map(|p| (p[2] / q).floor() as i32)
            .collect();
        assert_eq!(occupied, BTreeSet::from([0, 1, 2, 10, 11]));
        let local = from_surface_window(&volume, Some(([0., 0.], 0.1))).unwrap();
        assert_eq!(local.ground, input.ground);
        assert_eq!(local.static_collision_ground, input.static_collision_ground);
        assert_eq!(local.obstacles, input.obstacles);
        let (config, cells, bands) = pct_input(&volume, 0.4, &[]).unwrap();
        assert_eq!(config.resolution, q);
        assert_eq!(
            cells.iter().map(|cell| cell.ground).collect::<Vec<_>>(),
            vec![3. * q, 12. * q]
        );
        assert!(bands
            .iter()
            .any(|band| band.low == 2. * q && band.high == 3. * q));
        assert!(bands
            .iter()
            .any(|band| band.low == 10. * q && band.high == 11. * q));
    }

    #[test]
    fn volume_keeps_wall_interior_and_separate_ceiling_occupied() {
        let levels: Vec<_> = (0..20).chain(40..42).collect();
        let source = fine_column(&levels).with_ground_evidence(GroundEvidence::OccupiedVolume);
        let input = from_surface(&source).unwrap();
        let q = f64::from(0.05_f32);
        assert_eq!(
            input.ground.iter().map(|p| p[2]).collect::<Vec<_>>(),
            vec![20. * q, 42. * q],
            "连续墙内不能每格造地面，断开的楼板保留独立露顶"
        );
        let occupied: BTreeSet<_> = input
            .obstacles
            .iter()
            .map(|p| (p[2] / q).floor() as i32)
            .collect();
        assert_eq!(
            occupied,
            levels.into_iter().collect(),
            "真实墙、高台及上方楼板的占据不得丢失"
        );
        assert_eq!(input.cells[0].ceiling, 40. * q);
    }

    #[test]
    fn raw_noise_keeps_measured_height_and_wall_has_no_middle_floor() {
        let (_, base) = crate::native_test_helpers::fixture();
        let mut points = vec![(0.05_f32, 0.05_f32, -0.02_f32), (0.05, 0.05, 0.02)];
        points.extend((0..20).map(|z| (0.25, 0.05, z as f32 * 0.05)));
        let map = SurfaceSnapshot::from_points(&points, base.config().clone(), 0.1).unwrap();
        let input = from_surface(&map).unwrap();
        let floor: Vec<_> = input.cells.iter().filter(|c| c.x == 0).collect();
        assert_eq!(floor.len(), 1);
        assert!(floor[0].ground.abs() <= 0.021, "量化格顶不能成为地面高度");
        let wall: Vec<_> = input.cells.iter().filter(|c| c.x == 2).collect();
        assert_eq!(wall.len(), 1, "连续墙中间不能产生多个假楼层");
        assert!(wall[0].ground > 0.85);
        assert!(input
            .obstacles
            .iter()
            .any(|p| p[0] > 0.2 && p[2] > 0.3 && p[2] < 0.7));
    }
    #[test]
    fn recent_labels_remain_separate_and_global_consumes_real_obstacle() {
        let (_, base) = crate::native_test_helpers::fixture();
        let map = base
            .with_recent_obstacle_points(1, 1, 0, &[0], [(0, [0.5, 1., 0.04]), (1, [1., 1., 0.5])])
            .unwrap();
        let mut input = NativeMap {
            resolution: 0.1,
            ground: vec![],
            static_collision_ground: vec![],
            obstacles: vec![],
            cells: vec![],
            bands: vec![],
            forbidden_columns: vec![],
        };
        append_recent(&map, &mut input);
        assert_eq!(input.ground, vec![[0.5, 1., 0.04]]);
        assert_eq!(input.static_collision_ground, input.ground);
        assert_eq!(input.obstacles.len(), 1);
        assert!(input
            .bands
            .iter()
            .any(|band| band.low > 0.4 && band.high > 0.5));
    }
}
