//! `WavefrontDetector`:BFS / 8 邻居 frontier 判定 / cluster 检测。
//!
//! 公共 API:仅 `new(config)` + `diagnostics()`。
//! `detect_candidates` 为 `pub(crate)`,输出 crate-private `DetectionOutcome`。
//!
//! 检测结果与候选类型保持包内可见；`WavefrontError`作为公共返回类型公开。

use thiserror::Error;

use nuwax_nav_types_messages::geometry::Point2;
use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};

use crate::classification::PointClassification;
use crate::config::WavefrontConfig;
use crate::diagnostics::WavefrontDiagnostics;
use crate::grid::{occupancy_value, ClassificationGrid, GridPoint};

/// Wavefront detector 错误(公共 API:`select_goal_at` 必须能返回 typed error)。
#[derive(Debug, Error, PartialEq)]
pub enum WavefrontError {
    /// 机器人位姿在栅格外
    #[error("robot pose out of grid bounds: ({0}, {1})")]
    RobotPoseOutOfBounds(f64, f64),
    /// 起点轮廓尚无完整观测；不能据此派发导航，也不能把初图缺测当永久围困。
    #[error("robot footprint not fully observed at: ({0}, {1})")]
    RobotFootprintUnobserved(f64, f64),
}

/// 检测结果(crate-private;selector 消费它产生 `GoalSelection`)。
#[derive(Debug, Clone)]
pub(crate) enum DetectionOutcome {
    /// 候选 cluster 列表(已按 centroid 稳定顺序)
    Candidates(Vec<FrontierCandidate>),
    /// 找不到任何可达 free cell(机器人被封死)
    NoReachableFreeSpace,
}

/// 内部 cluster 几何。
#[derive(Debug, Clone)]
pub(crate) struct FrontierCluster {
    /// cluster 中心(世界坐标)
    pub centroid: Point2,
    /// cluster cell 数
    pub cell_count: usize,
    /// cluster 周长(米;本实现按 cell 数 * resolution 估算)
    pub perimeter_m: f64,
    /// cluster 网格总面积(平方米 = cell_count * resolution^2)
    pub area_m2: f64,
}

/// 内部 candidate。
#[derive(Debug, Clone)]
pub(crate) struct FrontierCandidate {
    /// cluster 几何
    pub cluster: FrontierCluster,
    /// 位于前沿内侧、具有机器人轮廓余量且真实可达的接近目标。
    pub goal_position: Point2,
    /// 原始(尚未打分)id 槽位
    pub debug_index: usize,
}

/// 8 邻居固定顺序输出(`[(dx,dy); 8]`,跳 `(0,0)`,外层 dx / 内层 dy)。
pub(crate) const NEIGHBOR_OFFSETS: [(i32, i32); 8] = [
    (-1, -1),
    (-1, 0),
    (-1, 1),
    (0, -1),
    (0, 1),
    (1, -1),
    (1, 0),
    (1, 1),
];

/// 本地固定容量邻居 buffer(方案 §4.7 显式要求 `NeighborBuf`,
/// **不**新增 `arrayvec` 依赖,使用本 crate 内置 small struct)。
#[derive(Debug, Clone, Copy)]
pub(crate) struct NeighborBuf {
    items: [Option<GridPoint>; 8],
    len: u8,
}

impl NeighborBuf {
    /// 长度
    #[allow(dead_code)] // API 预留:集成测试可断言长度
    #[must_use]
    pub fn len(self) -> usize {
        self.len as usize
    }
    /// 是否为空
    #[allow(dead_code)] // API 预留:与 `len` 配套,集成测试可断言
    #[must_use]
    pub fn is_empty(self) -> bool {
        self.len == 0
    }
    /// 迭代
    pub fn iter(self) -> impl Iterator<Item = GridPoint> + 'static {
        let mut items = self.items;
        let len = self.len;
        (0..len as usize).filter_map(move |i| items[i].take())
    }
}

/// v18 §4.7 显式要求的 `neighbors8` helper(替代零散的 `for (ox, oy) in NEIGHBOR_OFFSETS` 调用)。
pub(crate) fn neighbors8(p: GridPoint, grid: &OccupancyGrid2D) -> NeighborBuf {
    let mut buf = NeighborBuf {
        items: [None; 8],
        len: 0,
    };
    for (ox, oy) in NEIGHBOR_OFFSETS.iter() {
        let nx = p.x as i32 + ox;
        let ny = p.y as i32 + oy;
        if nx < 0 || ny < 0 || nx >= grid.width as i32 || ny >= grid.height as i32 {
            continue;
        }
        // 紧凑存储:有效邻居放 items[0..len],保持 len == 实际数量 + 索引连续
        buf.items[buf.len as usize] = Some(GridPoint::new(nx as u32, ny as u32));
        buf.len += 1;
    }
    buf
}

/// Wavefront detector(无 tokio,无 stream,可单测)。
pub struct WavefrontDetector {
    config: WavefrontConfig,
    classification: ClassificationGrid,
    diagnostics: WavefrontDiagnostics,
    grid_footprint: Option<nuwax_nav_types_messages::GridFootprint>,
}

impl WavefrontDetector {
    /// 构造 detector
    #[must_use]
    pub fn new(config: WavefrontConfig) -> Self {
        Self {
            config,
            classification: ClassificationGrid::new(0, 0),
            diagnostics: WavefrontDiagnostics::default(),
            grid_footprint: None,
        }
    }

    /// 设置本次位姿对应的机体轮廓；输入地图已经由拥有方区分自体区域。
    pub fn set_grid_footprint(
        &mut self,
        footprint: Option<nuwax_nav_types_messages::GridFootprint>,
    ) {
        self.grid_footprint = footprint;
    }

    /// 当前配置(只读)
    #[must_use]
    pub fn config(&self) -> &WavefrontConfig {
        &self.config
    }

    /// 摘要诊断(只读)
    #[must_use]
    pub fn diagnostics(&self) -> &WavefrontDiagnostics {
        &self.diagnostics
    }

    /// 检测 frontier 候选(公共诊断写回 `self.diagnostics`)。
    ///
    /// 流程(方案 §4.8 + v18 HIGH-2 修正):
    /// 1. robot -> cell;越界返回 `RobotPoseOutOfBounds`
    /// 2. 清空 classification
    /// 3. 起点圆盘须已知可通行；缺测等待，已知障碍拒绝，不寻找替代起点。
    /// 4. 主 BFS 只遍历可达自由格，并禁止穿过对角墙角。
    /// 5. 收集 frontier cluster:仅从 MAP_CLOSED 区域相邻的 frontier seed 出发
    ///    (满足 v18 "reachable frontier 语义闭环")
    /// 6. 返回 `DetectionOutcome::Candidates`
    pub(crate) fn detect_candidates(
        &mut self,
        robot_pose: Point2,
        costmap: &OccupancyGrid2D,
        _session: &crate::session::ExplorationSession,
    ) -> Result<DetectionOutcome, WavefrontError> {
        self.classification = ClassificationGrid::new(costmap.width, costmap.height);

        let (start_x, start_y) =
            costmap
                .world_to_cell(robot_pose)
                .ok_or(WavefrontError::RobotPoseOutOfBounds(
                    robot_pose.x,
                    robot_pose.y,
                ))?;

        self.diagnostics.last_candidates.clear();
        // 非自由起点没有已证实的通行路径，不能任意投影到墙另一侧。
        let start_cell = GridPoint::new(start_x, start_y);
        let footprint_map = if let Some(footprint) = self.grid_footprint.as_ref() {
            let mut view = costmap.clone();
            for y in 0..costmap.height {
                for x in 0..costmap.width {
                    if !footprint.is_clear(costmap, x, y) {
                        view.set(x, y, OccupancyCell::Inflated);
                    }
                }
            }
            if footprint.is_clear_at(costmap, robot_pose) {
                view.set(start_x, start_y, OccupancyCell::Free);
            }
            view
        } else {
            match start_footprint_state(
                costmap,
                start_cell,
                self.config.obstacle_inflation_radius_m,
            ) {
                StartFootprintState::Blocked => return Ok(DetectionOutcome::NoReachableFreeSpace),
                StartFootprintState::Unobserved => {
                    return Err(WavefrontError::RobotFootprintUnobserved(
                        robot_pose.x,
                        robot_pose.y,
                    ));
                }
                StartFootprintState::Traversable => {}
            }
            footprint_safe_map(costmap, self.config.obstacle_inflation_radius_m)
        };
        if !cell_at(&footprint_map, start_x, start_y).is_known_traversable() {
            return Ok(DetectionOutcome::NoReachableFreeSpace);
        }
        let safe_reachable = reachable_free_region(start_cell, &footprint_map);

        // 主搜索只把真实连通的自由格标记为可达。
        let mut queue = std::collections::VecDeque::new();
        queue.push_back(start_cell);
        self.classification
            .set(start_cell.x, start_cell.y, PointClassification::MAP_OPEN);
        while let Some(p) = queue.pop_front() {
            self.classification
                .insert_flags(p.x, p.y, PointClassification::MAP_CLOSED);
            for n in neighbors8(p, costmap).iter() {
                if self
                    .classification
                    .get(n.x, n.y)
                    .contains(PointClassification::MAP_OPEN)
                {
                    continue;
                }
                if is_traversable_step(p, n, costmap) {
                    self.classification
                        .set(n.x, n.y, PointClassification::MAP_OPEN);
                    queue.push_back(n);
                }
            }
        }

        // 5. 收集 frontier cluster:仅从 MAP_CLOSED reachable region
        //    邻接的 frontier 出发;不可达区域(全 unknown / 隔离 free / 墙外)
        //    的 frontier **不**纳入候选(满足 v18 HIGH-2)。
        let min_cells = std::cmp::max(
            1,
            (self.config.min_frontier_perimeter_m / costmap.resolution.max(1e-6)).floor() as usize,
        );
        let mut candidates: Vec<FrontierCandidate> = Vec::new();
        let mut visited = ClassificationGrid::new(costmap.width, costmap.height);
        for y in 0..costmap.height {
            for x in 0..costmap.width {
                if visited
                    .get(x, y)
                    .contains(PointClassification::FRONTIER_OPEN)
                {
                    continue;
                }
                if !is_frontier_point(GridPoint::new(x, y), costmap, &self.config) {
                    continue;
                }
                // 守卫 (v18 HIGH-2):frontier 至少有一个邻居是 MAP_CLOSED(reachable free),
                // 才认为该 frontier 是"可达 frontier"。
                if !has_reachable_neighbor(GridPoint::new(x, y), costmap, &self.classification) {
                    continue;
                }
                // BFS 收集 cluster
                let mut cluster_cells: Vec<GridPoint> = Vec::new();
                let mut q = std::collections::VecDeque::new();
                q.push_back(GridPoint::new(x, y));
                visited.insert_flags(x, y, PointClassification::FRONTIER_OPEN);
                while let Some(p) = q.pop_front() {
                    cluster_cells.push(p);
                    for n in neighbors8(p, costmap).iter() {
                        if visited.contains_flags(n.x, n.y, PointClassification::FRONTIER_OPEN) {
                            continue;
                        }
                        if !is_frontier_point(n, costmap, &self.config) {
                            continue;
                        }
                        // cluster 内仍需 reachable 守卫
                        if !has_reachable_neighbor(n, costmap, &self.classification) {
                            continue;
                        }
                        visited.insert_flags(n.x, n.y, PointClassification::FRONTIER_OPEN);
                        q.push_back(n);
                    }
                }
                if cluster_cells.len() < min_cells {
                    continue;
                }
                let cluster = build_cluster(&cluster_cells, costmap);
                let Some(goal_position) = choose_approach_position(
                    &cluster_cells,
                    cluster.centroid,
                    robot_pose,
                    start_cell,
                    costmap,
                    &safe_reachable,
                    self.grid_footprint.as_ref().map_or(
                        self.config.obstacle_inflation_radius_m,
                        |footprint| {
                            footprint
                                .offsets
                                .iter()
                                .map(|&(x, y)| x.abs().max(y.abs()) as f64 * costmap.resolution)
                                .fold(0.0, f64::max)
                        },
                    ),
                ) else {
                    continue;
                };
                candidates.push(FrontierCandidate {
                    cluster,
                    goal_position,
                    debug_index: candidates.len(),
                });
            }
        }

        // 稳定排序:centroid.y / centroid.x / cell_count
        candidates.sort_by(|a, b| {
            a.cluster
                .centroid
                .y
                .partial_cmp(&b.cluster.centroid.y)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then_with(|| {
                    a.cluster
                        .centroid
                        .x
                        .partial_cmp(&b.cluster.centroid.x)
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .then_with(|| a.cluster.cell_count.cmp(&b.cluster.cell_count))
        });

        // diagnostics 摘要
        self.diagnostics.last_candidates.clear();
        for c in &candidates {
            self.diagnostics
                .last_candidates
                .push(crate::diagnostics::FrontierDebugSummary {
                    centroid: c.cluster.centroid,
                    cell_count: c.cluster.cell_count,
                    perimeter_m: c.cluster.perimeter_m,
                    score: 0.0,
                    score_breakdown: crate::diagnostics::FrontierScoreBreakdown {
                        total: 0.0,
                        information_gain: 0.0,
                        explored_goals: 0.0,
                        distance: 0.0,
                        obstacle_clearance: 0.0,
                        direction_momentum: 0.0,
                    },
                });
        }

        Ok(DetectionOutcome::Candidates(candidates))
    }
}

fn cell_at(grid: &OccupancyGrid2D, x: u32, y: u32) -> OccupancyCell {
    let idx = (y * grid.width + x) as usize;
    grid.cells[idx]
}

/// 起点安全性只看真实圆盘；任何已知障碍优先于未观测状态，不能延后已证实的阻挡。
enum StartFootprintState {
    Traversable,
    Unobserved,
    Blocked,
}

fn start_footprint_state(
    grid: &OccupancyGrid2D,
    start: GridPoint,
    radius_m: f64,
) -> StartFootprintState {
    let radius = (radius_m.max(0.0) / grid.resolution).ceil() as i64;
    if !radius_m.is_finite()
        || !grid.resolution.is_finite()
        || grid.resolution <= 0.0
        || radius >= i64::from(grid.width)
        || radius >= i64::from(grid.height)
    {
        return StartFootprintState::Blocked;
    }
    let mut unobserved = false;
    for dy in -radius..=radius {
        for dx in -radius..=radius {
            if dx * dx + dy * dy > radius * radius {
                continue;
            }
            let x = i64::from(start.x) + dx;
            let y = i64::from(start.y) + dy;
            let cell =
                if x >= 0 && y >= 0 && x < i64::from(grid.width) && y < i64::from(grid.height) {
                    grid.get(x as u32, y as u32)
                } else {
                    None
                };
            match cell {
                None | Some(OccupancyCell::Unknown) => unobserved = true,
                Some(value) if !value.is_known_traversable() => {
                    return StartFootprintState::Blocked
                }
                _ => {}
            }
        }
    }
    if unobserved {
        StartFootprintState::Unobserved
    } else {
        StartFootprintState::Traversable
    }
}

fn is_traversable_step(from: GridPoint, to: GridPoint, grid: &OccupancyGrid2D) -> bool {
    if !cell_at(grid, to.x, to.y).is_known_traversable() {
        return false;
    }
    // 八邻域的对角步必须有两个自由正交邻格，不能从两面墙之间穿角。
    from.x == to.x
        || from.y == to.y
        || (cell_at(grid, from.x, to.y).is_known_traversable()
            && cell_at(grid, to.x, from.y).is_known_traversable())
}

/// frontier 判定:unknown + 至少一个 free 邻居 + strict `>` 占用阈值排除。
pub(crate) fn is_frontier_point(
    p: GridPoint,
    grid: &OccupancyGrid2D,
    cfg: &WavefrontConfig,
) -> bool {
    let cell = cell_at(grid, p.x, p.y);
    if !matches!(cell, OccupancyCell::Unknown) {
        return false;
    }
    let mut has_free = false;
    for n in neighbors8(p, grid).iter() {
        let nc = cell_at(grid, n.x, n.y);
        if nc.is_known_traversable() {
            has_free = true;
        }
        if occupancy_value(nc) > cfg.occupancy_threshold {
            return false;
        }
    }
    has_free
}

/// 该 frontier cell 是否邻接 reachable MAP_CLOSED 区域(v18 HIGH-2 reachable 守卫)。
fn has_reachable_neighbor(
    p: GridPoint,
    grid: &OccupancyGrid2D,
    classification: &ClassificationGrid,
) -> bool {
    for n in neighbors8(p, grid).iter() {
        if cell_at(grid, n.x, n.y).is_known_traversable()
            && classification
                .get(n.x, n.y)
                .contains(PointClassification::MAP_CLOSED)
        {
            return true;
        }
    }
    false
}

/// 将机器人圆形轮廓投影到原始占据图，未知、图外和障碍均不提供通行余量。
fn footprint_safe_map(grid: &OccupancyGrid2D, radius_m: f64) -> OccupancyGrid2D {
    let mut safe = grid.clone();
    let radius = (radius_m.max(0.0) / grid.resolution).ceil() as i64;
    if !radius_m.is_finite()
        || !grid.resolution.is_finite()
        || grid.resolution <= 0.0
        || radius >= i64::from(grid.width)
        || radius >= i64::from(grid.height)
    {
        safe.cells.fill(OccupancyCell::Inflated);
        return safe;
    }
    let disk: Vec<(i64, i64)> = (-radius..=radius)
        .flat_map(|dy| (-radius..=radius).map(move |dx| (dx, dy)))
        .filter(|(dx, dy)| dx * dx + dy * dy <= radius * radius)
        .collect();
    for y in 0..grid.height {
        for x in 0..grid.width {
            let free = disk.iter().all(|(dx, dy)| {
                let nx = i64::from(x) + dx;
                let ny = i64::from(y) + dy;
                nx >= 0
                    && ny >= 0
                    && nx < i64::from(grid.width)
                    && ny < i64::from(grid.height)
                    && cell_at(grid, nx as u32, ny as u32).is_known_traversable()
            });
            if !free {
                safe.set(x, y, OccupancyCell::Inflated);
            }
        }
    }
    safe
}

fn reachable_free_region(start: GridPoint, grid: &OccupancyGrid2D) -> ClassificationGrid {
    let mut reachable = ClassificationGrid::new(grid.width, grid.height);
    let mut queue = std::collections::VecDeque::from([start]);
    reachable.set(start.x, start.y, PointClassification::MAP_CLOSED);
    while let Some(point) = queue.pop_front() {
        for next in neighbors8(point, grid).iter() {
            if !reachable.contains_flags(next.x, next.y, PointClassification::MAP_CLOSED)
                && is_traversable_step(point, next, grid)
            {
                reachable.set(next.x, next.y, PointClassification::MAP_CLOSED);
                queue.push_back(next);
            }
        }
    }
    reachable
}

/// 从前沿向已知侧退让一个轮廓半径，目标及其接近区域必须真实可通行。
#[allow(clippy::too_many_arguments)]
fn choose_approach_position(
    frontier_cells: &[GridPoint],
    centroid: Point2,
    robot_pose: Point2,
    robot_cell: GridPoint,
    grid: &OccupancyGrid2D,
    classification: &ClassificationGrid,
    clearance_m: f64,
) -> Option<Point2> {
    let inset_cells = (clearance_m.max(0.0) / grid.resolution).ceil() as i64 + 1;
    frontier_cells
        .iter()
        .flat_map(|frontier| {
            (-inset_cells..=inset_cells).flat_map(move |dy| {
                (-inset_cells..=inset_cells).filter_map(move |dx| {
                    let x = i64::from(frontier.x) + dx;
                    let y = i64::from(frontier.y) + dy;
                    (x >= 0 && y >= 0 && x < i64::from(grid.width) && y < i64::from(grid.height))
                        .then_some((GridPoint::new(x as u32, y as u32), *frontier))
                })
            })
        })
        .filter(|(cell, frontier)| {
            *cell != robot_cell
                && cell_at(grid, cell.x, cell.y).is_known_traversable()
                && classification.contains_flags(cell.x, cell.y, PointClassification::MAP_CLOSED)
                && frontier_visible_from(*cell, *frontier, grid)
        })
        .filter_map(|(cell, _)| grid.cell_center(cell.x, cell.y))
        .min_by(|a, b| {
            a.distance(centroid)
                .total_cmp(&b.distance(centroid))
                .then_with(|| b.distance(robot_pose).total_cmp(&a.distance(robot_pose)))
                .then_with(|| a.y.total_cmp(&b.y))
                .then_with(|| a.x.total_cmp(&b.x))
        })
}

/// 接近点至前沿的视线在首次触及未知前必须始终位于已知自由区。
fn frontier_visible_from(start: GridPoint, target: GridPoint, grid: &OccupancyGrid2D) -> bool {
    let (mut x, mut y) = (i64::from(start.x), i64::from(start.y));
    let (tx, ty) = (i64::from(target.x), i64::from(target.y));
    let dx = (tx - x).abs();
    let dy = -(ty - y).abs();
    let sx = (tx - x).signum();
    let sy = (ty - y).signum();
    let mut error = dx + dy;
    while (x, y) != (tx, ty) {
        if !cell_at(grid, x as u32, y as u32).is_known_traversable() {
            return false;
        }
        let twice = 2 * error;
        if twice >= dy {
            error += dy;
            x += sx;
        }
        if twice <= dx {
            error += dx;
            y += sy;
        }
    }
    true
}

fn build_cluster(cells: &[GridPoint], grid: &OccupancyGrid2D) -> FrontierCluster {
    let cell_count = cells.len();
    let mut sx = 0.0_f64;
    let mut sy = 0.0_f64;
    for p in cells {
        // 地图合同的原点是格子左下角，几何点统一使用格子中心。
        sx += grid.origin.x + (f64::from(p.x) + 0.5) * grid.resolution;
        sy += grid.origin.y + (f64::from(p.y) + 0.5) * grid.resolution;
    }
    let n = cell_count.max(1) as f64;
    let centroid = Point2 {
        x: sx / n,
        y: sy / n,
    };
    let perimeter_m = (cell_count as f64) * grid.resolution;
    let area_m2 = (cell_count as f64) * grid.resolution * grid.resolution;
    FrontierCluster {
        centroid,
        cell_count,
        perimeter_m,
        area_m2,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::WavefrontConfig;
    use crate::session::ExplorationSession;
    use nuwax_nav_types_messages::geometry::Point2;
    use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};

    fn grid_free() -> OccupancyGrid2D {
        OccupancyGrid2D {
            origin: Point2 { x: 0.0, y: 0.0 },
            resolution: 0.5,
            width: 4,
            height: 4,
            cells: vec![OccupancyCell::Free; 16],
        }
    }

    #[test]
    fn detector_reports_no_reachable_when_fully_occupied() {
        let mut grid = grid_free();
        for c in grid.cells.iter_mut() {
            *c = OccupancyCell::Occupied;
        }
        let mut det = WavefrontDetector::new(WavefrontConfig::default());
        let session = ExplorationSession::new();
        let out = det
            .detect_candidates(Point2 { x: 0.0, y: 0.0 }, &grid, &session)
            .unwrap();
        assert!(matches!(out, DetectionOutcome::NoReachableFreeSpace));
    }

    #[test]
    fn detector_out_of_bounds_returns_error() {
        let grid = grid_free();
        let mut det = WavefrontDetector::new(WavefrontConfig::default());
        let session = ExplorationSession::new();
        let err = det
            .detect_candidates(Point2 { x: 999.0, y: 999.0 }, &grid, &session)
            .unwrap_err();
        assert_eq!(err, WavefrontError::RobotPoseOutOfBounds(999.0, 999.0));
    }

    #[test]
    fn low_cost_terrain_is_reachable_but_threshold_and_unknown_block_footprint() {
        let mut grid =
            OccupancyGrid2D::filled(Point2::new(0.0, 0.0), 0.05, 40, 40, OccupancyCell::Unknown);
        for y in 5..35 {
            for x in 5..35 {
                grid.set(x, y, OccupancyCell::Cost(49));
            }
        }
        let mut detector = WavefrontDetector::new(WavefrontConfig::default());
        let robot = Point2::new(1.025, 1.025);
        let session = ExplorationSession::new();
        let result = detector.detect_candidates(robot, &grid, &session).unwrap();
        assert!(
            matches!(result, DetectionOutcome::Candidates(ref goals) if !goals.is_empty()),
            "低于导航障碍阈值的已知地面必须可生成真实前沿目标"
        );
        for blocked in [OccupancyCell::Cost(50), OccupancyCell::Occupied] {
            grid.set(24, 20, blocked);
            assert!(
                matches!(
                    detector.detect_candidates(robot, &grid, &session).unwrap(),
                    DetectionOutcome::NoReachableFreeSpace
                ),
                "轮廓内的障碍或未知不可放行"
            );
            grid.set(24, 20, OccupancyCell::Cost(49));
        }
        grid.set(24, 20, OccupancyCell::Unknown);
        assert!(
            matches!(
                detector.detect_candidates(robot, &grid, &session),
                Err(WavefrontError::RobotFootprintUnobserved(..))
            ),
            "缺测必须等待，不能派发目标"
        );
        grid.set(16, 20, OccupancyCell::Cost(50));
        assert!(
            matches!(
                detector.detect_candidates(robot, &grid, &session).unwrap(),
                DetectionOutcome::NoReachableFreeSpace
            ),
            "已知障碍优先于轮廓中同时存在的未知"
        );
    }

    #[test]
    fn unknown_cells_do_not_connect_separate_free_islands() {
        let mut grid = OccupancyGrid2D::filled(
            Point2 { x: 0.0, y: 0.0 },
            1.0,
            25,
            15,
            OccupancyCell::Unknown,
        );
        for y in 4..=8 {
            for x in 1..=5 {
                grid.set(x, y, OccupancyCell::Free);
            }
        }
        for y in 4..=9 {
            for x in 15..=20 {
                grid.set(x, y, OccupancyCell::Free);
            }
        }
        let mut detector = WavefrontDetector::new(WavefrontConfig::default());
        let result = detector
            .detect_candidates(Point2 { x: 3.5, y: 6.5 }, &grid, &ExplorationSession::new())
            .unwrap();
        let DetectionOutcome::Candidates(candidates) = result else {
            panic!("机器人所在自由岛应有可达前沿");
        };
        assert_eq!(candidates.len(), 1, "未知区域另一侧的自由岛不可达");
        assert!(candidates[0].cluster.centroid.x < 6.0);
    }

    #[test]
    fn blocked_robot_cell_does_not_search_through_walls() {
        let mut grid = grid_free();
        grid.set(0, 0, OccupancyCell::Occupied);
        let mut detector = WavefrontDetector::new(WavefrontConfig::default());
        let result = detector
            .detect_candidates(
                Point2 { x: 0.25, y: 0.25 },
                &grid,
                &ExplorationSession::new(),
            )
            .unwrap();
        assert!(matches!(result, DetectionOutcome::NoReachableFreeSpace));
    }

    #[test]
    fn diagonal_corner_does_not_connect_free_regions() {
        let mut grid = OccupancyGrid2D::filled(
            Point2 { x: 0.0, y: 0.0 },
            1.0,
            12,
            12,
            OccupancyCell::Unknown,
        );
        grid.set(1, 1, OccupancyCell::Free);
        // 起点只有对角自由邻格，两个正交出口均被墙封住。
        for (x, y) in [(0, 0), (0, 1), (0, 2), (1, 0), (2, 0), (1, 2), (2, 1)] {
            grid.set(x, y, OccupancyCell::Occupied);
        }
        for y in 2..=7 {
            for x in 2..=7 {
                grid.set(x, y, OccupancyCell::Free);
            }
        }
        let mut detector = WavefrontDetector::new(WavefrontConfig::default());
        let result = detector
            .detect_candidates(Point2 { x: 1.5, y: 1.5 }, &grid, &ExplorationSession::new())
            .unwrap();
        assert!(
            matches!(result, DetectionOutcome::NoReachableFreeSpace),
            "起点轮廓已触墙，不能穿过墙角抵达另一自由区域"
        );
    }

    #[test]
    fn neighbors_offsets_match_scheme_4_7() {
        // 固定顺序:dx=-1..=1 外层,dy=-1..=1 内层,跳过 (0,0)
        assert_eq!(NEIGHBOR_OFFSETS.len(), 8);
        assert_eq!(NEIGHBOR_OFFSETS[0], (-1, -1));
        assert!(!NEIGHBOR_OFFSETS.contains(&(0, 0)));
    }

    #[test]
    fn neighbors8_returns_8_in_center_4_in_corner_3_on_edge() {
        let g = grid_free(); // 4x4
        let center = neighbors8(GridPoint::new(1, 1), &g);
        assert_eq!(center.len(), 8);
        let corner = neighbors8(GridPoint::new(0, 0), &g);
        assert_eq!(corner.len(), 3);
        let edge = neighbors8(GridPoint::new(1, 0), &g);
        assert_eq!(edge.len(), 5);
    }

    #[test]
    fn is_frontier_point_requires_unknown_and_free_neighbor() {
        let mut grid = grid_free();
        grid.cells[0] = OccupancyCell::Unknown;
        let cfg = WavefrontConfig::default();
        assert!(is_frontier_point(GridPoint::new(0, 0), &grid, &cfg));
        for c in grid.cells.iter_mut() {
            *c = OccupancyCell::Unknown;
        }
        assert!(!is_frontier_point(GridPoint::new(0, 0), &grid, &cfg));
    }

    #[test]
    fn is_frontier_point_rejects_strict_occupied_neighbor() {
        let mut grid = grid_free();
        grid.cells[0] = OccupancyCell::Unknown;
        grid.cells[1] = OccupancyCell::Occupied;
        let cfg = WavefrontConfig::default();
        assert!(!is_frontier_point(GridPoint::new(0, 0), &grid, &cfg));
    }

    #[test]
    fn is_frontier_point_uses_python_strict_threshold_for_occupied_variant() {
        let mut grid = grid_free();
        grid.cells[0] = OccupancyCell::Unknown;
        grid.cells[1] = OccupancyCell::Occupied;
        let cfg = WavefrontConfig {
            occupancy_threshold: 100,
            ..WavefrontConfig::default()
        };
        assert!(is_frontier_point(GridPoint::new(0, 0), &grid, &cfg));
    }

    /// v18 HIGH-2 反例:两块 free 区域被墙隔开,只有起点所在区域能 reach;
    /// 隔离区域(全 unknown + 自身 free)虽然技术上 is_frontier_point,
    /// 但其邻居不在 MAP_CLOSED reachable region,应被守卫排除。
    #[test]
    fn isolated_free_region_does_not_produce_candidates() {
        // 8x4 栅格:
        // - 左半 (x<4) 全 free(起点可达)
        // - 右半 (x>=4) 全 unknown(无 free 邻居可达,墙)
        let mut g = OccupancyGrid2D {
            origin: Point2 { x: 0.0, y: 0.0 },
            resolution: 0.5,
            width: 8,
            height: 4,
            cells: vec![OccupancyCell::Free; 32],
        };
        // 隔离:在 x=4 处放一列墙(覆盖整个 height)
        for y in 0..4 {
            g.cells[(y * 8 + 4) as usize] = OccupancyCell::Occupied;
        }
        // 右半 (x>=5) 改为 unknown,模拟未知区域
        for y in 0..4 {
            for x in 5..8 {
                g.cells[(y * 8 + x) as usize] = OccupancyCell::Unknown;
            }
        }
        let mut det = WavefrontDetector::new(WavefrontConfig::default());
        let session = ExplorationSession::new();
        let r = det
            .detect_candidates(Point2 { x: 0.5, y: 0.5 }, &g, &session)
            .unwrap();
        if let DetectionOutcome::Candidates(cs) = r {
            // 任何候选都应来自左半可達区域;右半不可达,不應納入
            for cand in &cs {
                let cx = cand.cluster.centroid.x;
                assert!(
                    cx < 2.5,
                    "candidate at x={cx} is in the unreachable right half; \
                     reachable guard (v18 HIGH-2) failed",
                );
            }
        } else {
            panic!("expected Candidates");
        }
    }
}
