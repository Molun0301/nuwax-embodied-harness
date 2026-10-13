use ahash::{AHashMap, AHashSet};
use crate::cancellation::checkpoint;
#[cfg(feature = "mls-adapter")]
use nuwax_nav_mls_planner::SurfaceSnapshot;
use std::collections::VecDeque;
use nuwax_types::ground_layer::{GroundApproximation, GroundEvidence, GroundLayer};

/// 一个压缩后的层析单元：同列重复切片只保留一组地面/顶面。
#[derive(Clone, Debug)]
pub struct Cell {
    pub column: [i32; 2],
    pub layer: i32,
    pub ground: f64,
    /// 实测地面带的范围，参考高程不冒充精确接触平面。
    pub ground_interval: [f64; 2],
    /// Compatibility provenance, not a claim of measured point statistics.
    pub ground_approximation: GroundApproximation,
    pub ceiling: f64,
    /// 已知障碍的层内距离，不把未采到地面的列直接当成墙。
    pub clearance: f64,
    pub(crate) support_clearance: f64,
    pub cost: f64,
    pub(crate) neighbors: Vec<usize>,
}
#[derive(Debug)]
pub struct Tomogram {
    pub resolution: f64,
    pub cells: Vec<Cell>,
    pub(crate) columns: AHashMap<[i32; 2], Vec<usize>>,
    known_columns: AHashSet<[i32; 2]>,
    pub(crate) step_up: f64,
    pub(crate) step_down: f64,
    pub(crate) slope: f64,
    pub(crate) body_height: f64,
    pub(crate) half_width: f64,
    pub(crate) clearance_radius: f64,
}
impl Tomogram {
    #[cfg(feature = "mls-adapter")]
    pub fn from_surface(source: &SurfaceSnapshot) -> Result<Self, String> {
        Self::from_surface_window(source, None)
    }
    /// 近场滑动窗口仅派生周围列，沿用全局层析相同的物理参数与代价计算。
    #[cfg(feature = "mls-adapter")]
    pub fn local_from_surface(
        source: &SurfaceSnapshot,
        center: [f64; 2],
        radius: f64,
    ) -> Result<Self, String> {
        Self::from_surface_window(source, Some((center, radius)))
    }
    /// Build a local derived map with an explicitly scoped request token.
    #[cfg(feature = "mls-adapter")]
    pub fn local_from_surface_cancellable(
        source: &SurfaceSnapshot, center: [f64; 2], radius: f64,
        cancellation: &crate::PlanningCancellation,
    ) -> Result<Self, String> {
        crate::with_planning_cancellation(cancellation, || Self::local_from_surface(source, center, radius))
    }
    #[cfg(feature = "mls-adapter")]
    fn from_surface_window(
        source: &SurfaceSnapshot,
        window: Option<([f64; 2], f64)>,
    ) -> Result<Self, String> {
        checkpoint()?;
        let config = source.config();
        let body = source.body_envelope();
        let required_height = body.map_or(f64::from(config.robot_height), |m| {
            m.body_to_support_z_m + m.z_bounds_m[1]
        });
        let half_width = body.map_or(f64::from(config.wall_clearance_m), |m| {
            let (lo, hi) = m
                .footprint_xy_m
                .iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                    (lo.min(p[1]), hi.max(p[1]))
                });
            (hi - lo) * 0.5
        });
        // The source supplies one iterator; its creation and individual next() calls
        // remain indivisible, so check on both sides of that MLS boundary.
        checkpoint()?;
        let (resolution, columns) = source.navigation_source_columns(window);
        checkpoint()?;
        let result = Self::from_columns_with_layers(
            resolution,
            f64::from(source.source_voxel_size_m()),
            columns.take_while(|_| !crate::planning_cancelled()).map(|(column, mut bands)| {
                let allowed = source.navigation_column_allowed(column, resolution);
                if !allowed { bands.clear(); }
                (column, bands)
            }),
            required_height,
            half_width,
            body.map_or(f64::from(config.step_threshold_m), |m| m.max_step_up_m),
            body.map_or(f64::from(config.step_threshold_m), |m| m.max_step_down_m),
            f64::from(config.max_slope_deg),
            body.map_or(half_width, |body| body.radius()),
            source.ground_evidence()==nuwax_nav_mls_planner::surface::GroundEvidence::OccupiedVolume,
            |column,bands|source.navigation_measured_ground_layers(column,resolution,bands),
        )?;
        checkpoint()?;
        Ok(result)
    }

    /// 原始高度带和物理参数来自同一个规范地图及机器人标定。
    #[allow(clippy::too_many_arguments)]
    pub fn from_columns(
        resolution: f64,
        columns: impl IntoIterator<Item = ([i32; 2], Vec<[f64; 2]>)>,
        body_height: f64,
        half_width: f64,
        step_up: f64,
        step_down: f64,
        max_slope_deg: f64,
    ) -> Result<Self, String> {
        Self::from_columns_with_quantum(
            resolution,
            resolution,
            columns,
            body_height,
            half_width,
            step_up,
            step_down,
            max_slope_deg,
            half_width,
        )
    }
    /// 从原始占据高度带构建层析图；源量化与足迹半径独立于水平规划分辨率。
    #[allow(clippy::too_many_arguments)]
    pub fn from_columns_with_quantum(
        resolution: f64,
        source_quantum: f64,
        columns: impl IntoIterator<Item = ([i32; 2], Vec<[f64; 2]>)>,
        body_height: f64,
        half_width: f64,
        step_up: f64,
        step_down: f64,
        max_slope_deg: f64,
        clearance_radius: f64,
    ) -> Result<Self, String> {
        Self::from_columns_with_evidence(resolution,source_quantum,columns,body_height,half_width,
            step_up,step_down,max_slope_deg,clearance_radius,false)
    }
    /// 实体占据与表面测量带使用相同占据数据，按声明含义选择暴露顶或测量参考。
    #[allow(clippy::too_many_arguments)]
    pub fn from_columns_with_evidence(
        resolution: f64,
        source_quantum: f64,
        columns: impl IntoIterator<Item = ([i32; 2], Vec<[f64; 2]>)>,
        body_height: f64,
        half_width: f64,
        step_up: f64,
        step_down: f64,
        max_slope_deg: f64,
        clearance_radius: f64,
        occupied_volume:bool,
    ) -> Result<Self, String> {
        Self::from_columns_with_layers(resolution,source_quantum,columns,body_height,half_width,
            step_up,step_down,max_slope_deg,clearance_radius,occupied_volume,|_,_|None)
    }

    #[allow(clippy::too_many_arguments)]
    fn from_columns_with_layers(
        resolution: f64,
        source_quantum: f64,
        columns: impl IntoIterator<Item = ([i32; 2], Vec<[f64; 2]>)>,
        body_height: f64,
        half_width: f64,
        step_up: f64,
        step_down: f64,
        max_slope_deg: f64,
        clearance_radius: f64,
        occupied_volume:bool,
        measured_layers: impl Fn([i32;2], &[[f64;2]])->Option<Vec<(GroundLayer,f64)>>,
    ) -> Result<Self, String> {
        checkpoint()?;
        if !resolution.is_finite()
            || resolution <= 0.0
            || !source_quantum.is_finite()
            || source_quantum <= 0.0
            || !clearance_radius.is_finite()
            || clearance_radius < 0.0
            || !body_height.is_finite()
            || body_height <= 0.0
            || !half_width.is_finite()
            || half_width < 0.0
            || !step_up.is_finite()
            || step_up < 0.0
            || !step_down.is_finite()
            || step_down < 0.0
            || !max_slope_deg.is_finite()
            || !(0.0..90.0).contains(&max_slope_deg)
        {
            return Err("层析物理参数无效".into());
        }
        let mut map = Self {
            resolution,
            cells: Vec::new(),
            columns: AHashMap::new(),
            known_columns: AHashSet::new(),
            step_up,
            step_down,
            slope: max_slope_deg.to_radians().tan(),
            body_height,
            half_width,
            clearance_radius,
        };
        // 切片间隔只控制层号；相同地面/顶面的重复切片在此直接压缩。
        let spacing = body_height.max(resolution);
        for (column, mut bands) in columns {
            checkpoint()?;
            map.known_columns.insert(column);
            bands.retain(|b| b[0].is_finite() && b[1].is_finite() && b[0] < b[1]);
            bands.sort_unstable_by(|a, b| a[0].total_cmp(&b[0]));
            let mut merged: Vec<[f64; 2]> = Vec::new();
            for band in bands {
                checkpoint()?;
                if let Some(last) = merged.last_mut().filter(|last| band[0] <= last[1] + 1e-6) {
                    last[1] = last[1].max(band[1]);
                } else {
                    merged.push(band);
                }
            }
            let layers=measured_layers(column,&merged).unwrap_or_else(|| {
                merged.iter().enumerate().map(|(i,band)| {
                    let layer=GroundLayer::from_legacy_band(*band,source_quantum,
                        if occupied_volume {GroundEvidence::OccupiedVolume}else{GroundEvidence::MeasuredSurface})
                        .expect("validated height band");
                    let ceiling=merged.get(i+1).map_or(f64::INFINITY,|next|next[0]);
                    (layer,ceiling)
                }).collect()
            });
            for (layer,ceiling) in layers {
                checkpoint()?;
                let ground=layer.representative_z;
                if ceiling - ground + 1e-6 < body_height {
                    continue;
                }
                let id = map.cells.len();
                map.cells.push(Cell {
                    column,
                    layer: (ground / spacing).floor() as i32,
                    ground,
                    ground_interval: layer.occupied_interval,
                    ground_approximation: layer.approximation,
                    ceiling,
                    clearance: f64::INFINITY,
                    support_clearance: f64::INFINITY,
                    cost: 1.0,
                    neighbors: Vec::with_capacity(8),
                });
                map.columns.entry(column).or_default().push(id);
            }
        }
        checkpoint()?;
        if map.cells.is_empty() {
            return Err("层析地图没有可用地面".into());
        }
        map.connect()?;
        map.compute_costs()?;
        checkpoint()?;
        Ok(map)
    }
    pub fn position(&self, id: usize) -> [f64; 3] {
        let c = &self.cells[id];
        [
            (f64::from(c.column[0]) + 0.5) * self.resolution,
            (f64::from(c.column[1]) + 0.5) * self.resolution,
            c.ground,
        ]
    }
    pub fn cell_at(&self, point: [f64; 3]) -> Option<usize> {
        if !point.into_iter().all(f64::is_finite) {
            return None;
        }
        let column = [
            (point[0] / self.resolution).floor() as i32,
            (point[1] / self.resolution).floor() as i32,
        ];
        self.columns
            .get(&column)?
            .iter()
            .copied()
            .min_by(|&a, &b| {
                (self.cells[a].ground - point[2])
                    .abs()
                    .total_cmp(&(self.cells[b].ground - point[2]).abs())
            })
            .filter(|&id| {
                (self.cells[id].ground - point[2]).abs()
                    <= self.step_up.max(self.step_down).max(self.resolution) * 1.5 + 1e-6
            })
    }
    pub fn nearest(&self, point: [f64; 3], radius: f64) -> Option<usize> {
        let x = (point[0] / self.resolution).floor() as i32;
        let y = (point[1] / self.resolution).floor() as i32;
        let cells = (radius / self.resolution).ceil() as i32;
        let mut best = None;
        for ix in x - cells..=x + cells {
            for iy in y - cells..=y + cells {
                if let Some(ids) = self.columns.get(&[ix, iy]) {
                    for &id in ids {
                        let distance = crate::distance(self.position(id), point);
                        if distance <= radius && best.is_none_or(|(_, old)| distance < old) {
                            best = Some((id, distance));
                        }
                    }
                }
            }
        }
        best.map(|(id, _)| id)
    }
    fn compatible(&self, a: usize, b: usize, distance: f64) -> bool {
        let a = &self.cells[a];
        let b = &self.cells[b];
        let rise = b.ground - a.ground;
        let limit = if rise >= 0.0 {
            self.step_up
        } else {
            self.step_down
        };
        rise.abs() <= limit.max(distance * self.slope) + 1e-6
            && a.ceiling.min(b.ceiling) - a.ground.max(b.ground) + 1e-6 >= self.body_height
    }
    fn neighbor_in(&self, a: usize, column: [i32; 2], distance: f64) -> Option<usize> {
        self.columns
            .get(&column)?
            .iter()
            .copied()
            .filter(|&b| self.compatible(a, b, distance))
            .min_by(|&b, &c| {
                (self.cells[a].ground - self.cells[b].ground)
                    .abs()
                    .total_cmp(&(self.cells[a].ground - self.cells[c].ground).abs())
            })
    }
    fn connect(&mut self) -> Result<(), String> {
        for id in 0..self.cells.len() {
            checkpoint()?;
            let [x, y] = self.cells[id].column;
            let mut neighbors = Vec::with_capacity(8);
            // One cell's cardinal choices also serve its diagonal corner checks.
            // Lazy stack slots preserve the first lookup and short-circuit order.
            let mut cardinal = [None; 4];
            let mut cardinal_neighbor = |dx, dy, distance| {
                let index = match (dx, dy) {
                    (-1, 0) => 0,
                    (0, -1) => 1,
                    (0, 1) => 2,
                    (1, 0) => 3,
                    _ => unreachable!("cardinal direction"),
                };
                *cardinal[index].get_or_insert_with(|| {
                    self.neighbor_in(id, [x + dx, y + dy], distance)
                })
            };
            for dx in -1..=1 {
                for dy in -1..=1 {
                    if dx == 0 && dy == 0 {
                        continue;
                    }
                    let distance = self.resolution * f64::from(dx * dx + dy * dy).sqrt();
                    let next = if dx == 0 || dy == 0 {
                        cardinal_neighbor(dx, dy, distance)
                    } else {
                        self.neighbor_in(id, [x + dx, y + dy], distance)
                    };
                    let Some(next) = next else { continue; };
                    if dx != 0 && dy != 0 {
                        // 对角连接不能穿过两面相接的墙角或落差。
                        let Some(a) = cardinal_neighbor(dx, 0, self.resolution) else {
                            continue;
                        };
                        let Some(b) = cardinal_neighbor(0, dy, self.resolution) else {
                            continue;
                        };
                        if !self.compatible(a, next, self.resolution)
                            || !self.compatible(b, next, self.resolution)
                        {
                            continue;
                        }
                    }
                    neighbors.push(next);
                }
            }
            self.cells[id].neighbors = neighbors;
        }
        checkpoint()?;
        Ok(())
    }
    fn compute_costs(&mut self) -> Result<(), String> {
        let mut walls = VecDeque::new();
        let mut unknown = VecDeque::new();
        for id in 0..self.cells.len() {
            checkpoint()?;
            self.cells[id].clearance = f64::INFINITY;
            self.cells[id].support_clearance = f64::INFINITY;
            let [x, y] = self.cells[id].column;
            let mut wall = false;
            let mut missing = false;
            for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                let column = [x + dx, y + dy];
                if !self.known_columns.contains(&column) {
                    missing = true;
                } else if self.neighbor_in(id, column, self.resolution).is_none() {
                    wall = true;
                }
            }
            if wall {
                self.cells[id].clearance = self.resolution * 0.5;
                walls.push_back(id);
            }
            if missing {
                self.cells[id].support_clearance = self.resolution * 0.5;
                unknown.push_back(id);
            }
        }
        for (mut queue, is_wall) in [(walls, true), (unknown, false)] {
            while let Some(id) = queue.pop_front() {
                checkpoint()?;
                let value = if is_wall {
                    self.cells[id].clearance
                } else {
                    self.cells[id].support_clearance
                };
                let distance = value + self.resolution;
                let neighbors = self.cells[id].neighbors.clone();
                for next in neighbors {
                    let target = if is_wall {
                        &mut self.cells[next].clearance
                    } else {
                        &mut self.cells[next].support_clearance
                    };
                    if distance < *target {
                        *target = distance;
                        queue.push_back(next);
                    }
                }
            }
        }
        // 高差影响机体整个足迹，不能只在中心恰好经过阶边时付出代价。
        // 沿同层可连接单元传播有限偏好，真实台阶与唯一通道仍保持连通。
        let mut terrain = Vec::with_capacity(self.cells.len());
        let mut queue = VecDeque::new();
        for (id, cell) in self.cells.iter().enumerate() {
            checkpoint()?;
            let value = cell.neighbors.iter()
                .map(|&n| (self.cells[n].ground - cell.ground).abs() / self.resolution)
                .fold(0.0, f64::max).min(2.0);
            terrain.push(value);
            if value > 0.0 { queue.push_back(id); }
        }
        let decay = 2.0 * self.resolution / self.clearance_radius.max(self.resolution);
        while let Some(id) = queue.pop_front() {
            checkpoint()?;
            let value = terrain[id] - decay;
            if value <= 0.0 {
                continue;
            }
            for &next in &self.cells[id].neighbors {
                if value > terrain[next] + 1e-9 {
                    terrain[next] = value;
                    queue.push_back(next);
                }
            }
        }
        for id in 0..self.cells.len() {
            checkpoint()?;
            let c = &self.cells[id];
            let preferred = self.clearance_radius.max(self.half_width) + self.resolution;
            let wall = preferred / (c.clearance + self.resolution * 0.5);
            // 未测列仍不能作为路径单元，但稀疏采样边缘不冒充实体墙做强膨胀。
            let unknown = ((preferred - c.support_clearance) / preferred).clamp(0.0, 1.0);
            self.cells[id].cost = 1.0 + wall * wall + 3.0 * unknown * unknown + terrain[id];
        }
        checkpoint()?;
        Ok(())
    }
    /// 层析直线可见性只验证全局参考；精细机体许可归近场轨迹。
    pub fn segment_clear(&self, a: [f64; 3], b: [f64; 3], boundary: &[[f64; 2]]) -> bool {
        self.segment_clear_cancelled(a, b, boundary, None)
    }
    pub(crate) fn segment_clear_cancelled(
        &self, a: [f64; 3], b: [f64; 3], boundary: &[[f64; 2]],
        cancelled: Option<&std::sync::atomic::AtomicBool>,
    ) -> bool {
        if !a.into_iter().chain(b).all(f64::is_finite) {
            return false;
        }
        let n = ((a[0] - b[0]).hypot(a[1] - b[1]) / (self.resolution * 0.4))
            .ceil()
            .max(1.0) as usize;
        let mut previous: Option<usize> = None;
        for i in 0..=n {
            if crate::planning_cancelled() || cancelled.is_some_and(|flag| flag.load(std::sync::atomic::Ordering::Acquire)) { return false; }
            let t = i as f64 / n as f64;
            let p = std::array::from_fn(|j| a[j] + t * (b[j] - a[j]));
            if !crate::inside_boundary([p[0], p[1]], boundary) {
                return false;
            }
            let Some(id) = self.cell_at(p) else {
                return false;
            };
            if let Some(old) = previous {
                if old != id && !self.cells[old].neighbors.contains(&id) {
                    return false;
                }
            }
            previous = Some(id);
        }
        true
    }
}

#[cfg(test)]
mod cancellation_tests {
    use super::*;
    #[test]
    fn owned_flag_stops_column_ingestion_and_abandons_connect_and_costs() {
        use std::{cell::Cell, sync::{Arc, atomic::{AtomicBool, Ordering}}};
        let flag = Arc::new(AtomicBool::new(false));
        let token = crate::PlanningCancellation::from_flag(flag.clone());
        let visited = Cell::new(0);
        let columns = (0..10_000).map(|x| {
            visited.set(visited.get() + 1);
            if x == 7 { flag.store(true, Ordering::Release); }
            ([x, 0], vec![[-0.1, 0.0]])
        });
        let error = crate::with_planning_cancellation(&token, ||
            Tomogram::from_columns(0.1, columns, 0.5, 0.15, 0.2, 0.2, 20.0)).unwrap_err();
        assert_eq!(error, "层析规划已取消");
        assert_eq!(visited.get(), 8, "the remaining 9,992 columns must not be consumed");
        // Entry + three checkpoints per column + post-input = 302.
        // Connect consumes checkpoints 302..=402; 450 is inside compute_costs.
        for (phase, count) in [("connect", 302), ("costs", 450)] {
            let flag = Arc::new(AtomicBool::new(false));
            let flip = flag.clone();
            let token = crate::PlanningCancellation::from_flag(flag.clone());
            let consumed = Cell::new(0);
            let result = crate::cancellation::after_checkpoint_action(count,
                move || flip.store(true, Ordering::Release), ||
                crate::with_planning_cancellation(&token, ||
                    Tomogram::from_columns(0.1, (0..100).map(|x| {
                        consumed.set(consumed.get() + 1);
                        ([x, 0], vec![[-0.1, 0.0]])
                    }), 0.5, 0.15, 0.2, 0.2, 20.0)));
            assert!(flag.load(Ordering::Acquire), "{phase} checkpoint was not reached");
            assert_eq!(consumed.get(), 100, "{phase} must run after ingestion");
            assert_eq!(result.unwrap_err(), "层析规划已取消", "{phase}");
        }
        assert!(!crate::planning_cancelled());
    }

    #[test]
    fn cancellation_stops_column_input_before_collecting_the_whole_map() {
        use std::{cell::Cell, sync::{Arc, atomic::{AtomicU64, Ordering}}};
        let epoch = Arc::new(AtomicU64::new(4));
        let token = crate::PlanningCancellation::new(epoch.clone(), 4);
        let visited = Cell::new(0);
        let columns = (0..10_000).map(|x| {
            visited.set(visited.get() + 1);
            if x == 7 { epoch.store(5, Ordering::Release); }
            ([x, 0], vec![[-0.1, 0.0]])
        });
        let error = crate::with_planning_cancellation(&token, ||
            Tomogram::from_columns(0.1, columns, 0.5, 0.15, 0.2, 0.2, 20.0)).unwrap_err();
        assert!(error.contains("取消"));
        assert_eq!(visited.get(), 8);
    }
    #[test]
    fn cancellation_abandons_connect_and_cost_passes_without_returning_a_partial_map() {
        // Each column uses three checkpoints; 302 reaches connect, 450 reaches costs.
        for checkpoints in [302, 450] {
            let error = crate::cancellation::after_checkpoints(checkpoints, ||
                Tomogram::from_columns(0.1, (0..100).map(|x| ([x, 0], vec![[-0.1, 0.0]])),
                    0.5, 0.15, 0.2, 0.2, 20.0)).unwrap_err();
            assert!(error.contains("取消"), "{error}");
        }
    }
}

#[cfg(test)]
mod allocation_benchmark_tests {
    use super::*;
    use std::{fmt::Write as _, hint::black_box, time::{Duration, Instant}};

    fn columns() -> Vec<([i32; 2], Vec<[f64; 2]>)> {
        let mut columns = Vec::new();
        for x in 0..80 {
            for y in 0..64 {
                // 缺测带只留远端开口；已知高墙的门洞位于另一侧。
                if (52..54).contains(&x) && !(42..52).contains(&y) {
                    continue;
                }
                let ground = f64::from((x / 10 + y / 8) % 4) * 0.015;
                let mut bands = if x == 27 && !(20..30).contains(&y) {
                    vec![[-0.1, 2.0]]
                } else {
                    vec![[ground - 0.1, ground]]
                };
                // 局部上层与下层重叠；上层的两个测量带还需要合并。
                if (8..24).contains(&x) && (8..24).contains(&y) {
                    bands.extend([[1.4, 1.5], [1.45, 1.55]]);
                }
                columns.push(([x, y], bands));
            }
        }
        columns
    }

    fn construct(columns: Vec<([i32; 2], Vec<[f64; 2]>)>) -> Tomogram {
        Tomogram::from_columns(0.1, columns, 0.5, 0.2, 0.15, 0.15, 20.0).unwrap()
    }

    fn exact_output(map: &Tomogram) -> String {
        let mut output = String::from("tomogram-allocation-v1; route elapsed_ms excluded\n");
        writeln!(output, "parameters_bits {:?}", [map.resolution, map.body_height,
            map.half_width, map.step_up, map.step_down, map.slope, map.clearance_radius]
            .map(f64::to_bits)).unwrap();
        for (id, cell) in map.cells.iter().enumerate() {
            writeln!(output, "cell {id} column {:?} layer {} bits {:?} neighbors {:?}",
                cell.column, cell.layer,
                [cell.ground, cell.ground_interval[0], cell.ground_interval[1],
                    cell.ceiling, cell.clearance, cell.support_clearance, cell.cost]
                    .map(f64::to_bits), cell.neighbors).unwrap();
        }
        // 只排序哈希容器的键；单元、同列索引、邻接与路线顺序原样保留。
        let mut columns: Vec<_> = map.columns.iter().collect();
        columns.sort_unstable_by_key(|(column, _)| **column);
        for (column, ids) in columns {
            writeln!(output, "column {column:?} ids {ids:?}").unwrap();
        }
        let mut known: Vec<_> = map.known_columns.iter().copied().collect();
        known.sort_unstable();
        writeln!(output, "known_columns {known:?}").unwrap();
        for (name, start, goal) in [
            ("ground", [0.45, 0.65, 0.0], [7.45, 5.85, 0.03]),
            ("upper", [0.95, 0.95, 1.525], [2.25, 2.25, 1.525]),
        ] {
            let route = map.plan(start, goal, 0.01, &[], None).unwrap();
            writeln!(output, "route {name} start_bits {:?} goal_bits {:?} expanded {} cells {} points {}",
                start.map(f64::to_bits), goal.map(f64::to_bits),
                route.stats.expanded, route.stats.cells, route.points.len()).unwrap();
            for point in route.points {
                writeln!(output, "point_bits {:?}", point.map(f64::to_bits)).unwrap();
            }
        }
        output
    }

    #[test]
    #[ignore = "固定输入的手动性能基线；用 --ignored --nocapture 单独运行"]
    fn tomogram_allocation_frozen_baseline() {
        use std::io::Write as _;
        const WARMUPS: usize = 4;
        const CONSTRUCT_REPS: usize = 32;
        const COST_REPS: usize = 128;
        // 输入生成、参考输出、校验和文件写入都在计时之外。
        let columns = columns();
        let bands: usize = columns.iter().map(|(_, bands)| bands.len()).sum();
        assert_eq!((columns.len(), bands), (5_012, 5_524));
        let mut map = construct(columns.clone());
        assert_eq!((map.cells.len(), map.known_columns.len()), (5_268, 5_012));
        let edges: usize = map.cells.iter().map(|cell| cell.neighbors.len()).sum();
        let expected = exact_output(&map);

        for _ in 0..WARMUPS {
            drop(black_box(construct(black_box(columns.clone()))));
        }
        let mut construction = Duration::ZERO;
        for _ in 0..CONSTRUCT_REPS {
            let began = Instant::now();
            let built = construct(black_box(columns.clone()));
            black_box(&built);
            construction += began.elapsed();
            // 返回地图的析构不计时；消耗输入时的释放仍属于构建成本。
            drop(built);
        }

        for _ in 0..WARMUPS {
            black_box(map.compute_costs()).unwrap();
            black_box(&map);
        }
        // 复用同一精确地图，只计代价重算，不克隆输入或重建邻接。
        let began = Instant::now();
        for _ in 0..COST_REPS {
            black_box(map.compute_costs()).unwrap();
            black_box(&map);
        }
        let costs = began.elapsed();
        let actual = exact_output(&map);
        assert_eq!(actual, expected, "重复代价重算必须逐字节保持输出");
        if let Some(path) = std::env::var_os("NEH_TOMOGRAM_ALLOC_DUMP") {
            // create_new 防止覆盖冻结基线；输出不含任何计时结果。
            let mut file = std::fs::OpenOptions::new().write(true).create_new(true)
                .open(path).expect("精确输出文件必须是新路径");
            file.write_all(actual.as_bytes()).unwrap();
        }
        println!("tomogram_allocation fixture=mixed_80x64 input_columns={} input_bands={} cells={} directed_edges={} warmups={} construct_reps={} cost_reps={}",
            columns.len(), bands, map.cells.len(), edges, WARMUPS, CONSTRUCT_REPS, COST_REPS);
        println!("tomogram_allocation construct_input_clone=included returned_map_destruction=excluded consumed_input_destruction=included direct_compute=existing_exact_map final_map_destruction=excluded");
        println!("tomogram_allocation construct_total_ns={} construct_ns_per_iter={:.3} compute_total_ns={} compute_ns_per_iter={:.3}",
            construction.as_nanos(), construction.as_secs_f64() * 1e9 / CONSTRUCT_REPS as f64,
            costs.as_nanos(), costs.as_secs_f64() * 1e9 / COST_REPS as f64);
    }
}

#[cfg(all(test, feature = "mls-adapter"))]
#[path = "tomogram_masked_tests.rs"]
mod masked_surface_tests;

#[cfg(test)]
#[path = "tomogram_connect_tests.rs"]
mod connect_trial_tests;

#[cfg(test)]
mod shared_ground_tests {
    use super::*;

    #[test]
    fn pct_uses_shared_layer_geometry_for_legacy_and_volume_columns() {
        for occupied in [false,true] {
            for step in [0.0,0.15,0.5] {
                let evidence=if occupied {GroundEvidence::OccupiedVolume}else{GroundEvidence::MeasuredSurface};
                let bands=vec![[-0.15,0.0],[0.85,1.0]];
                let map=Tomogram::from_columns_with_evidence(0.05,0.05,
                    [([0,0],bands.clone())],0.3,0.0,step,step,20.0,0.0,occupied).unwrap();
                assert_eq!(map.cells.len(),2);
                for (cell,band) in map.cells.iter().zip(bands) {
                    let shared=GroundLayer::from_legacy_band(band,0.05,evidence).unwrap();
                    assert_eq!(cell.ground,shared.representative_z);
                    assert_eq!(cell.ground_interval,shared.occupied_interval);
                    assert_eq!(cell.ground_approximation,shared.approximation);
                }
                assert_eq!(map.cells[0].ceiling,0.85);
                assert!(map.cells[0].neighbors.is_empty());
            }
        }
    }
}

#[cfg(all(test,feature="mls-adapter"))]
mod measured_layer_adapter_tests {
    use super::*;
    use std::{collections::BTreeMap,sync::Arc};
    use nuwax_nav_mls_planner::{mls_planner::Config,SurfaceObservation,SurfaceObservations};
    use nuwax_nav_mls_planner::surface::SurfaceBodyEnvelope;
    use nuwax_map_chunks::{block_key,local_key,LayerObservation,LayerSample,LayerObservations};

    #[test]
    fn pct_and_body_share_measured_plane_and_preserve_legacy_fallback() {
        let fine=0.05_f32;let quantum=f64::from(fine);let plane=-0.025;
        let points:Vec<_>=(-12..12).flat_map(|x|(-12..12).map(move|y|
            ((x as f32+0.5)*fine,(y as f32+0.5)*fine,-0.025))).collect();
        let cfg=Config {world_frame:"map".into(),voxel_size:0.1,robot_height:0.2,
            surface_closing_radius:0.0,node_spacing_m:0.3,wall_clearance_m:0.0,wall_buffer_m:0.0,
            wall_buffer_weight:0.0,max_slope_deg:20.0,step_threshold_m:0.1,
            min_step_tread_depth_m:0.2,step_penalty_weight:1.0,goal_tolerance:0.1,viz_publish_hz:0.0};
        let model=SurfaceBodyEnvelope{footprint_xy_m:vec![[-0.15,-0.1],[0.15,-0.1],[0.15,0.1],[-0.15,0.1]],
            z_bounds_m:[-0.1,0.1],body_to_support_z_m:0.3,max_step_up_m:0.1,max_step_down_m:0.1,
            max_cross_slope_deg:20.0,support_uncertainty_m:0.0};
        let source=SurfaceSnapshot::from_points(&points,cfg,fine).unwrap().with_body_envelope(model).unwrap();
        let mut occupancy=BTreeMap::<_,BTreeMap<_,_>>::new();
        let mut layers=BTreeMap::<_,LayerObservations>::new();
        for x in -12..12 {for y in -12..12 {
            let key=[x,y,-1];let block=block_key(key);let local=local_key(key);
            occupancy.entry(block).or_default().insert(key,SurfaceObservation{observed_ns:100,hits:1});
            let point=[(f64::from(x)+0.5)*quantum,(f64::from(y)+0.5)*quantum,plane];
            layers.entry(block).or_default().insert(local,LayerObservation{members:vec![local],
                normal:[0.0,0.0,1.0],plane_origin:point,
                samples:vec![LayerSample{point,source_sequence:1,source_timestamp_ns:100,point_index:u32::from(local)}]});
        }}
        let mut observations=SurfaceObservations::new(fine,32).unwrap();
        for (key,values) in occupancy {observations.replace_block(key,values).unwrap();}
        observations.mark_complete_geometry_source();
        let legacy=source.with_observations(Some(Arc::new(observations.clone())));
        let old=Tomogram::from_surface(&legacy).unwrap();
        assert!(old.cells.iter().all(|c|c.ground_approximation==GroundApproximation::LegacyContactInterval));
        for (key,records) in layers {observations.replace_ground_layers(key,Arc::new(records)).unwrap();}
        observations.mark_complete_geometry_source();
        let measured=source.with_observations(Some(Arc::new(observations)));
        let pct=Tomogram::from_surface(&measured).unwrap();
        assert!(pct.cells.iter().all(|c|c.ground==plane&&c.ground_approximation==GroundApproximation::ObservedLayer));
        let body=measured.body_pose_on_surface([0.025,0.025],0.0,[0.025,0.025,0.0]).unwrap();
        assert!((body.0[2]-0.3-plane).abs()<1e-9,"PCT and body must use the same source plane: {body:?}");
        measured.validate_body_pose(body.0,body.1).unwrap();
        let mut mask=vec![false;40*40];mask[20*40+20]=true;
        let forbidden=measured.with_virtual_obstacles([-1.0,-1.0],0.05,40,40,&mask).unwrap();
        let masked=Tomogram::from_surface(&forbidden).unwrap();
        assert!(!masked.columns.contains_key(&[0,0]),"measured metadata cannot resurrect forbidden columns");
    }
}
