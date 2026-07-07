//! `WavefrontDetector`:BFS / 8 邻居 frontier 判定 / cluster 检测。
//!
//! 公共 API:仅 `new(config)` + `diagnostics()`。
//! `detect_candidates` 为 `pub(crate)`,输出 crate-private `DetectionOutcome`。
//!
//! 内部类型 (`DetectionOutcome` / `FrontierCluster` / `FrontierCandidate` /
//! `WavefrontError`) 全部 `pub(crate)`;**不**列入稳定公共 API。

use thiserror::Error;

use nuwax_nav_types_messages::geometry::Point2;
use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};

use crate::classification::PointClassification;
use crate::config::WavefrontConfig;
use crate::diagnostics::WavefrontDiagnostics;
use crate::grid::{ClassificationGrid, GridPoint};

/// Wavefront detector 错误(公共 API:`select_goal_at` 必须能返回 typed error)。
#[derive(Debug, Error, PartialEq)]
pub enum WavefrontError {
    /// 机器人位姿在栅格外
    #[error("robot pose out of grid bounds: ({0}, {1})")]
    RobotPoseOutOfBounds(f64, f64),
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
}

impl WavefrontDetector {
    /// 构造 detector
    #[must_use]
    pub fn new(config: WavefrontConfig) -> Self {
        Self {
            config,
            classification: ClassificationGrid::new(0, 0),
            diagnostics: WavefrontDiagnostics::default(),
        }
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
    /// 3. BFS 找最近 free cell;若不存在返回 `DetectionOutcome::NoReachableFreeSpace`
    /// 4. 主 BFS 把可达 free/unknown 标记为 MAP_OPEN/MAP_CLOSED
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

        // 3. BFS 找最近 free cell
        let start_cell = GridPoint::new(start_x, start_y);
        let nearest_free = find_nearest_free_space(start_cell, costmap);
        let nearest_free = match nearest_free {
            Some(c) => c,
            None => return Ok(DetectionOutcome::NoReachableFreeSpace),
        };

        // 4. 主 BFS:可达 free/unknown -> MAP_OPEN,处理完 -> MAP_CLOSED
        let mut queue = std::collections::VecDeque::new();
        queue.push_back(nearest_free);
        self.classification.set(
            nearest_free.x,
            nearest_free.y,
            PointClassification::MAP_OPEN,
        );
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
                let cell = cell_at(costmap, n.x, n.y);
                if is_free_or_unknown(cell) {
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
                candidates.push(FrontierCandidate {
                    cluster,
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

fn is_free_or_unknown(cell: OccupancyCell) -> bool {
    matches!(cell, OccupancyCell::Free | OccupancyCell::Unknown)
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
        match nc {
            OccupancyCell::Free => has_free = true,
            OccupancyCell::Occupied | OccupancyCell::Inflated => return false,
            OccupancyCell::Cost(v) if (v as i16) > cfg.occupancy_threshold => return false,
            _ => {}
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
        if classification
            .get(n.x, n.y)
            .contains(PointClassification::MAP_CLOSED)
        {
            return true;
        }
    }
    false
}

/// BFS 找最近 free cell,无 free 返回 None。
pub(crate) fn find_nearest_free_space(
    start: GridPoint,
    grid: &OccupancyGrid2D,
) -> Option<GridPoint> {
    use std::collections::VecDeque;
    let mut visited = ClassificationGrid::new(grid.width, grid.height);
    let mut queue = VecDeque::new();
    queue.push_back(start);
    visited.insert_flags(start.x, start.y, PointClassification::MAP_OPEN);
    while let Some(p) = queue.pop_front() {
        if matches!(cell_at(grid, p.x, p.y), OccupancyCell::Free) {
            return Some(p);
        }
        for n in neighbors8(p, grid).iter() {
            if visited.contains_flags(n.x, n.y, PointClassification::MAP_OPEN) {
                continue;
            }
            visited.insert_flags(n.x, n.y, PointClassification::MAP_OPEN);
            queue.push_back(n);
        }
    }
    None
}

fn build_cluster(cells: &[GridPoint], grid: &OccupancyGrid2D) -> FrontierCluster {
    let cell_count = cells.len();
    let mut sx = 0.0_f64;
    let mut sy = 0.0_f64;
    for p in cells {
        let c = grid.cell_center(p.x, p.y).unwrap_or(grid.origin);
        sx += c.x;
        sy += c.y;
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
