use ahash::{AHashMap, AHashSet};
#[cfg(feature = "mls-adapter")]
use nuwax_nav_mls_planner::SurfaceSnapshot;
use std::collections::VecDeque;

/// 一个压缩后的层析单元：同列重复切片只保留一组地面/顶面。
#[derive(Clone, Debug)]
pub struct Cell {
    pub column: [i32; 2],
    pub layer: i32,
    pub ground: f64,
    /// 实测地面带的范围，参考高程不冒充精确接触平面。
    pub ground_interval: [f64; 2],
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
    #[cfg(feature = "mls-adapter")]
    fn from_surface_window(
        source: &SurfaceSnapshot,
        window: Option<([f64; 2], f64)>,
    ) -> Result<Self, String> {
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
        let (resolution, columns) = source.navigation_source_columns(window);
        let mut blocked = AHashSet::new();
        let mut result = Self::from_columns_with_evidence(
            resolution,
            f64::from(source.source_voxel_size_m()),
            columns.filter(|(column, _)| {
                let allowed = source.navigation_column_allowed(*column, resolution);
                if !allowed {
                    blocked.insert(*column);
                }
                allowed
            }),
            required_height,
            half_width,
            body.map_or(f64::from(config.step_threshold_m), |m| m.max_step_up_m),
            body.map_or(f64::from(config.step_threshold_m), |m| m.max_step_down_m),
            f64::from(config.max_slope_deg),
            body.map_or(half_width, |body| body.radius()),
            source.ground_evidence()==nuwax_nav_mls_planner::surface::GroundEvidence::OccupiedVolume,
        )?;
        if !blocked.is_empty() {
            result.known_columns.extend(blocked);
            result.compute_costs();
        }
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
            map.known_columns.insert(column);
            bands.retain(|b| b[0].is_finite() && b[1].is_finite() && b[0] < b[1]);
            bands.sort_unstable_by(|a, b| a[0].total_cmp(&b[0]));
            let mut merged: Vec<[f64; 2]> = Vec::new();
            for band in bands {
                if let Some(last) = merged.last_mut().filter(|last| band[0] <= last[1] + 1e-6) {
                    last[1] = last[1].max(band[1]);
                } else {
                    merged.push(band);
                }
            }
            for (i, band) in merged.iter().enumerate() {
                // 厚地面带是两种LIO都可能产生的测量厚度。只在地形能力范围内
                // 使用带内代表高程，避免把每列最高回波都当成离散台阶；真实接触仍由近场判断。
                let depth = band[1] - band[0];
                let ground = if !occupied_volume && depth <= 2.0 * step_up.max(step_down) + source_quantum + 1e-6 {
                    band[1] - (depth - source_quantum).max(0.0) * 0.5
                } else {
                    band[1]
                };
                let ceiling = merged.get(i + 1).map_or(f64::INFINITY, |next| next[0]);
                if ceiling - ground + 1e-6 < body_height {
                    continue;
                }
                let id = map.cells.len();
                map.cells.push(Cell {
                    column,
                    layer: (ground / spacing).floor() as i32,
                    ground,
                    ground_interval: *band,
                    ceiling,
                    clearance: f64::INFINITY,
                    support_clearance: f64::INFINITY,
                    cost: 1.0,
                    neighbors: Vec::with_capacity(8),
                });
                map.columns.entry(column).or_default().push(id);
            }
        }
        if map.cells.is_empty() {
            return Err("层析地图没有可用地面".into());
        }
        map.connect();
        map.compute_costs();
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
    fn connect(&mut self) {
        for id in 0..self.cells.len() {
            let [x, y] = self.cells[id].column;
            let mut neighbors = Vec::with_capacity(8);
            for dx in -1..=1 {
                for dy in -1..=1 {
                    if dx == 0 && dy == 0 {
                        continue;
                    }
                    let distance = self.resolution * f64::from(dx * dx + dy * dy).sqrt();
                    let Some(next) = self.neighbor_in(id, [x + dx, y + dy], distance) else {
                        continue;
                    };
                    if dx != 0 && dy != 0 {
                        // 对角连接不能穿过两面相接的墙角或落差。
                        let Some(a) = self.neighbor_in(id, [x + dx, y], self.resolution) else {
                            continue;
                        };
                        let Some(b) = self.neighbor_in(id, [x, y + dy], self.resolution) else {
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
    }
    fn compute_costs(&mut self) {
        let mut walls = VecDeque::new();
        let mut unknown = VecDeque::new();
        for id in 0..self.cells.len() {
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
        let mut terrain: Vec<f64> = self
            .cells
            .iter()
            .map(|c| {
                c.neighbors
                    .iter()
                    .map(|&n| (self.cells[n].ground - c.ground).abs() / self.resolution)
                    .fold(0.0, f64::max)
                    .min(2.0)
            })
            .collect();
        let mut queue: VecDeque<usize> = (0..self.cells.len())
            .filter(|&id| terrain[id] > 0.0)
            .collect();
        let decay = 2.0 * self.resolution / self.clearance_radius.max(self.resolution);
        while let Some(id) = queue.pop_front() {
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
            let c = &self.cells[id];
            let preferred = self.clearance_radius.max(self.half_width) + self.resolution;
            let wall = preferred / (c.clearance + self.resolution * 0.5);
            // 未测列仍不能作为路径单元，但稀疏采样边缘不冒充实体墙做强膨胀。
            let unknown = ((preferred - c.support_clearance) / preferred).clamp(0.0, 1.0);
            self.cells[id].cost = 1.0 + wall * wall + 3.0 * unknown * unknown + terrain[id];
        }
    }
    /// 层析直线可见性只验证全局参考；精细机体许可归近场轨迹。
    pub fn segment_clear(&self, a: [f64; 3], b: [f64; 3], boundary: &[[f64; 2]]) -> bool {
        if !a.into_iter().chain(b).all(f64::is_finite) {
            return false;
        }
        let n = ((a[0] - b[0]).hypot(a[1] - b[1]) / (self.resolution * 0.4))
            .ceil()
            .max(1.0) as usize;
        let mut previous: Option<usize> = None;
        for i in 0..=n {
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
