//! 将已有不可变感知转换为原生规划输入，不执行地面投影或运动许可。
use crate::SurfaceSnapshot;
use nuwax_nav_pct_native::{Band, Cell, Config};
use std::collections::{BTreeMap, BTreeSet};

/// 全局导航与探索共用的地形来源，旧表面链保留到设备验收。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PctTerrainSource {
    #[default]
    Tomography,
    Mls,
}

impl PctTerrainSource {
    pub fn parse(value: &str) -> Result<Self, String> {
        match value.trim() {
            "" | "tomography" => Ok(Self::Tomography),
            "mls" => Ok(Self::Mls),
            value => Err(format!("NUWAX_NAV_PCT_TERRAIN_SOURCE 不支持 {value:?}，应为 tomography 或 mls")),
        }
    }

    pub fn from_runtime() -> Result<Self, String> {
        Self::parse(&nuwax_config::navigation()?.pct_terrain_source)
    }
}

/// 原地面高度与实体占据，二者保持独立。
#[derive(Clone, Debug)]
pub struct NativeMap {
    /// 原始占据分辨率。
    pub resolution: f64,
    /// 真实地面代表点，缺测列不补高度。
    pub ground: Vec<[f64; 3]>,
    /// SCAN 持久层的真实表面点；不重复注入实体 band 的人工暴露顶边界。
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
    from_surface_selection(map, window, None)
}

fn from_surface_selection(
    map: &SurfaceSnapshot,
    window: Option<([f64; 2], f64)>,
    selected: Option<&BTreeSet<[i32; 2]>>,
) -> Result<NativeMap, String> {
    let measured_surface =
        map.ground_evidence() == nuwax_nav_mls_planner::surface::GroundEvidence::MeasuredSurface;
    let occupied_volume =
        map.ground_evidence() == nuwax_nav_mls_planner::surface::GroundEvidence::OccupiedVolume;
    let (resolution, columns) = map.navigation_source_columns(window);
    let columns: Vec<_> = match selected {
        Some(keys) => match map.observations().filter(|o|
            o.is_complete_geometry_source() && f64::from(o.voxel_size_m()) == resolution) {
            Some(observations) => keys.iter().filter_map(|&xy| {
                observations.column_levels(xy[0], xy[1]).map(|levels|
                    (xy, levels.iter().map(|&z|
                        [f64::from(z) * resolution, f64::from(z + 1) * resolution]).collect()))
            }).filter(|(xy, _)| window.is_none_or(|(center, radius)| (0..2).all(|i|
                ((f64::from(xy[i]) + 0.5) * resolution - center[i]).abs() <= radius))).collect(),
            None => columns.filter(|(xy, _)| keys.contains(xy)).collect(),
        },
        None => columns.collect(),
    };
    let nearby = |point: &[f64; 3]| {
        window.is_none_or(|(center, radius)| {
            (point[0] - center[0]).abs() <= radius && (point[1] - center[1]).abs() <= radius
        })
    };
    let forbidden_columns = if selected.is_some() { Vec::new() } else { map
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
        .collect() };
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
        let chosen = (window.is_some() || selected.is_some()).then(|| points.iter().copied()
            .filter(nearby).filter(|p| selected.is_none_or(|keys|
                keys.contains(&[p[0], p[1]].map(|v| (v / resolution).floor() as i32)))).collect::<Vec<_>>());
        let points = chosen.as_deref().unwrap_or(points);
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
        for &(column, ref bands) in &columns {
            if raw.contains_key(&column)
                || map.observations().is_some_and(|observations| {
                    observations.ground_records(column).next().is_some()
                        || observations.classified_empty_ground(column)
                })
            {
                continue;
            }
            let xy = column.map(|v| (f64::from(v) + 0.5) * resolution);
            for &[low, high] in bands {
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


#[derive(Default)]
pub(crate) struct NativeColumn {
    #[cfg(test)]
    pct_ground: Vec<[f64; 3]>,
    #[cfg(test)]
    cells: Vec<Cell>,
    #[cfg(test)]
    bands: Vec<Band>,
    pub ground: Vec<[f64; 3]>,
    pub obstacles: Vec<[f64; 3]>,
}

pub(crate) struct NativeColumns {
    pub resolution: f64,
    pub columns: BTreeMap<[i32; 2], NativeColumn>,
    order: Vec<[i32; 2]>,
    pub forbidden_columns: Vec<[i32; 2]>,
}
impl NativeColumns {
    pub fn new(map: NativeMap, source: &SurfaceSnapshot, window: ([f64; 2], f64)) -> Self {
        let order = source.navigation_source_columns(Some(window)).1.map(|(xy, _)| xy).collect();
        let mut result = Self { resolution: map.resolution, columns: BTreeMap::new(), order,
            forbidden_columns: map.forbidden_columns };
        for p in map.static_collision_ground {
            result.columns.entry([p[0], p[1]].map(|v| (v / result.resolution).floor() as i32))
                .or_default().ground.push(p);
        }
        for p in map.obstacles {
            result.columns.entry([p[0], p[1]].map(|v| (v / result.resolution).floor() as i32))
                .or_default().obstacles.push(p);
        }
        #[cfg(test)] {
            for p in map.ground {
                result.columns.entry([p[0], p[1]].map(|v| (v / result.resolution).floor() as i32))
                    .or_default().pct_ground.push(p);
            }
            for c in map.cells { result.columns.entry([c.x, c.y]).or_default().cells.push(c); }
            for b in map.bands { result.columns.entry([b.x, b.y]).or_default().bands.push(b); }
        }
        result
    }
    #[cfg(test)]
    pub fn native_map(&self) -> NativeMap {
        NativeMap { resolution: self.resolution,
            ground: self.columns.values().flat_map(|c| c.pct_ground.iter().copied()).collect(),
            static_collision_ground: self.ground().collect(), obstacles: self.obstacles().collect(),
            cells: self.columns.values().flat_map(|c| c.cells.iter().copied()).collect(),
            bands: self.columns.values().flat_map(|c| c.bands.iter().copied()).collect(),
            forbidden_columns: self.forbidden_columns.clone() }
    }
    pub fn replace(&mut self, map: &SurfaceSnapshot, window: ([f64; 2], f64),
        keys: &[[i32; 2]]) -> Result<(), String> {
        let keys: BTreeSet<_> = keys.iter().copied().collect();
        let update = from_surface_selection(map, Some(window), Some(&keys))?;
        let mut update = Self::new(update, map, window);
        std::mem::swap(&mut self.order, &mut update.order);
        for key in keys { self.columns.remove(&key); }
        self.columns.extend(update.columns);
        Ok(())
    }
    pub fn ground(&self) -> impl Iterator<Item = [f64; 3]> + '_ {
        self.order.iter().filter_map(|xy| self.columns.get(xy)).flat_map(|c| c.ground.iter().copied())
    }
    pub fn obstacles(&self) -> impl Iterator<Item = [f64; 3]> + '_ {
        self.order.iter().filter_map(|xy| self.columns.get(xy)).flat_map(|c| c.obstacles.iter().copied())
    }
}

/// 一格余量及一个地图提交周期内机身角点的最大位移，复用现有运动与周期配置。
pub(crate) fn collision_ground_padding(config: &nuwax_nav_scan_native::Config) -> f64 {
    let radius = (config.body_length * 0.5 + config.body_center_x.abs())
        .hypot(config.body_width * 0.5 + config.body_center_y.abs());
    let translation = config
        .max_velocity
        .max(config.max_reverse_velocity)
        .hypot(config.max_lateral_velocity);
    config.resolution + config.sample_dt * (translation + radius * config.max_yaw_rate)
}

/// 近期地面覆盖同列旧地面；外扩机身内，相对支撑或同列最低层可跨的高地面过时。
pub(crate) fn collision_ground(
    persistent: Vec<[f64; 3]>,
    recent: Vec<[f64; 3]>,
    config: &nuwax_nav_scan_native::Config,
    position: [f64; 3],
    yaw: f64,
) -> Vec<[f64; 3]> {
    let r = config.resolution;
    let column = |p: &[f64; 3]| [p[0], p[1]].map(|v| (v / r).floor() as i32);
    let recent_columns: BTreeSet<_> = recent.iter().map(column).collect();
    let mut lowest_ground = BTreeMap::<[i32; 2], f64>::new();
    for p in &persistent {
        let height = lowest_ground.entry(column(p)).or_insert(p[2]);
        *height = height.min(p[2]);
    }
    let (s, c) = yaw.sin_cos();
    let (ac, as_) = (c.abs(), s.abs());
    let center = [
        position[0] + c * config.body_center_x - s * config.body_center_y,
        position[1] + s * config.body_center_x + c * config.body_center_y,
    ];
    let padding = collision_ground_padding(config);
    let (hx, hy) = (
        config.body_length * 0.5 + padding,
        config.body_width * 0.5 + padding,
    );
    let (ex, ey) = (ac * hx + as_ * hy, as_ * hx + ac * hy);
    let support = position[2] - config.body_to_ground;
    persistent
        .into_iter()
        .filter(|p| {
            let key = column(p);
            if recent_columns.contains(&key) {
                return false;
            }
            // 四轴矩形/格子相交，机身外扩覆盖异步快照后一周期内平移和转向新触及的列。
            let [dx, dy] = [0, 1].map(|i| (f64::from(key[i]) + 0.5) * r - center[i]);
            let under_body = dx.abs() <= ex + r * 0.5
                && dy.abs() <= ey + r * 0.5
                && (c * dx + s * dy).abs() <= hx + r * 0.5 * (ac + as_)
                && (-s * dx + c * dy).abs() <= hy + r * 0.5 * (ac + as_);
            // 当前支撑或同列最低地面提供高度基准，避免叠加机身高度估计误差。
            let height_above_support = p[2] - support;
            let height_above_lowest = p[2] - lowest_ground[&key];
            let stale_ground = (height_above_support > 0.
                && height_above_support <= config.max_step_up)
                || (height_above_lowest > 0. && height_above_lowest <= config.max_step_up);
            !(under_body && stale_ground)
        })
        .chain(recent)
        .collect()
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
    let config = pct_config(map, body_to_ground, boundary, input.resolution);
    Ok((config, input.cells, input.bands))
}

/// 两种地形来源共用完整机体参数，来源切换不改变碰撞和通行能力。
fn pct_config(
    map: &SurfaceSnapshot,
    body_to_ground: f64,
    boundary: &[[f64; 2]],
    resolution: f64,
) -> Config {
    let mut config = Config {
        resolution,
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
        // 全局与SCAN执行端共用同一长宽矩形，凸包的窄前角不能成为不可执行的通路。
        let bounds = body.footprint_xy_m.iter().fold(
            [[f64::INFINITY; 2], [f64::NEG_INFINITY; 2]], |mut bounds, point| {
                for i in 0..2 { bounds[0][i] = bounds[0][i].min(point[i]);
                    bounds[1][i] = bounds[1][i].max(point[i]); }
                bounds
            });
        config.footprint = vec![bounds[0], [bounds[1][0], bounds[0][1]],
            bounds[1], [bounds[0][0], bounds[1][1]]];
        config.footprint_radius = config.footprint
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
    config
}

/// 从当前快照的源点生成层析；占据带和禁区交现有原生桥判定。
pub fn pct_input_from_tomogram(
    map: &SurfaceSnapshot,
    body_to_ground: f64,
    boundary: &[[f64; 2]],
) -> Result<(Config, Vec<Cell>, Vec<Band>), String> {
    let (resolution, columns) = map.navigation_source_columns(None);
    let config = pct_config(map, body_to_ground, boundary, resolution);
    let mut points = Vec::new();
    // 完整精细源带独立地面记录时，地面取记录的连续高度，成员体素不再作为占据采样。
    // 体素中心按源精度量化，缓坡或略倾斜的地图会被切成逐格台阶，两层体素厚的地面也会挡住机体。
    let ground = map.observations().filter(|source| {
        map.ground_evidence() == nuwax_nav_mls_planner::surface::GroundEvidence::MeasuredSurface
            && source.is_complete_geometry_source()
            && source.voxel_size_m().to_bits() == map.source_voxel_size_m().to_bits()
    });
    if let Some(measured) = map.navigation_measured_points().filter(|_| ground.is_none()) {
        points.extend(measured.iter().map(|p| p.map(|v| v as f32)));
    } else {
        // 没有地面记录的列（旧存档或已分类为空）仅恢复已有源体素中心。
        for (column, bands) in columns {
            let xy = column.map(|v| (f64::from(v) + 0.5) * resolution);
            let mut members = BTreeSet::new();
            for record in ground.into_iter().flat_map(|source| source.ground_records(column)) {
                members.extend(record.members());
                if let Some(height) = record.height_at(xy) {
                    points.push([xy[0] as f32, xy[1] as f32, height as f32]);
                }
            }
            for [low, high] in bands {
                let count = ((high - low) / config.point_resolution).round().max(1.0) as usize;
                for index in 0..count {
                    let z = low + (index as f64 + 0.5) * (high - low) / count as f64;
                    if members.contains(&((z / resolution).floor() as i32)) {
                        continue;
                    }
                    points.push([xy[0] as f32, xy[1] as f32, z as f32]);
                }
            }
        }
    }
    let (recent_ground, recent_obstacles) = map.navigation_recent_samples();
    points.extend(recent_ground.iter().map(|p| p.map(|v| v as f32)));
    let slice_dh = config.robot_height.max(config.point_resolution);
    let min_z = points.iter().map(|p| f64::from(p[2])).fold(f64::INFINITY, f64::min);
    let slice_h0 = if min_z.is_finite() { min_z + slice_dh } else { 0.0 };
    let mut tomogram = nuwax_pct_tomography::build(&points, &nuwax_pct_tomography::TomographyConfig {
        resolution,
        point_resolution: config.point_resolution,
        slice_h0,
        slice_dh,
    })?;
    // 近期非地面只补实体占据，不能把动态物体顶面变成全局地形。
    let half = config.point_resolution * 0.5;
    tomogram.bands.extend(recent_obstacles.iter().map(|p| Band {
        x: (p[0] / resolution).floor() as i32,
        y: (p[1] / resolution).floor() as i32,
        low: p[2] - half,
        high: p[2] + half,
    }));
    tomogram.bands.extend(map.navigation_forbidden_columns(resolution).into_iter().map(|column| Band {
        x: column[0],
        y: column[1],
        low: f64::NEG_INFINITY,
        high: f64::INFINITY,
    }));
    Ok((config, tomogram.cells, tomogram.bands))
}

/// 显式来源便于两个消费者使用同一配置，并把来源纳入缓存身份。
pub fn pct_input_for_source(
    map: &SurfaceSnapshot,
    body_to_ground: f64,
    boundary: &[[f64; 2]],
    source: PctTerrainSource,
) -> Result<(Config, Vec<Cell>, Vec<Band>), String> {
    match source {
        PctTerrainSource::Tomography => pct_input_from_tomogram(map, body_to_ground, boundary),
        PctTerrainSource::Mls => pct_input(map, body_to_ground, boundary),
    }
}

#[cfg(test)]
#[path = "native_map_tomography_tests.rs"]
mod tomography_tests;

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
    fn global_route_does_not_fit_a_narrow_nose_where_scan_rectangle_cannot_fit() {
        let (_, base) = crate::native_test_helpers::fixture();
        let mut body = base.body_envelope().unwrap().clone();
        body.footprint_xy_m = vec![[-0.4,-0.2],[0.1,-0.2],[0.4,-0.05],
            [0.4,0.05],[0.1,0.2],[-0.4,0.2]];
        body.max_step_up_m=0.0; body.max_step_down_m=0.0;
        let map = base.with_body_envelope(body).unwrap();
        let config = pct_config(&map, 0.4, &[], 0.05);
        let mut cells=Vec::new(); let mut bands=Vec::new();
        for x in -12..45 { for y in -12..13 {
            cells.push(Cell { x,y,ground:0.0,ceiling:f64::INFINITY });
            let half = if x>=26 {0.15} else {0.25};
            if ((f64::from(y)+0.5)*0.05).abs()>=half {
                bands.push(Band { x,y,low:0.0,high:1.0 });
            }
        }}
        let mut planner=nuwax_nav_pct_native::Planner::from_cells(config,&cells,&bands).unwrap();
        assert!(planner.plan([0.025,0.025,0.4],[1.025,0.025,0.4]).is_err(),
            "前端凸包虽能伸入窄口，执行端的完整矩形放不下，不能交付完整路线");
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
