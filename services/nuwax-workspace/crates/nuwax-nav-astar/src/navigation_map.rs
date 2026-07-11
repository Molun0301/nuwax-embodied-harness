//! 二元 / 梯度 costmap 缓存
//!
//! 对应 Python 源 `dimos/navigation/replanning_a_star/navigation_map.py`。
//!
//! 设计：
//! - `CostMap2D`：u8 占据栅格。`0=FREE`, `100=OCCUPIED`, `255=UNKNOWN`（自定义 sentinel，
//!   与 Python `CostValues.UNKNOWN` 语义一致；Foundation 切换为 `nuwax-nav-types-messages`
//!   后应使用统一 sentinel）。
//! - `NavigationMap`：缓存最近一次 binary costmap，提供 `make_gradient_costmap` 接口。
//!   梯度扩展（inflate obstacle + voronoi/gradient 距离场）本阶段提供占位实现：
//!   保留接口与签名，调用真正的 distance transform 留待后续 PR 与 mapping crate 协作。
//!   Phase 1 单元测试只断言 API 形状、缓存与 binary 切换；gradient 具体算法在 Phase 1 完成后
//!   与 mapping 板块联合完成。

use crate::helpers::Point2;

/// 占据值 sentinel
pub const OCCUPIED: u8 = 100;
/// UNKNOWN sentinel（与 Python `CostValues.UNKNOWN` 语义一致；这里用 255 区分于 0..100 的 cost 值）
pub const UNKNOWN: u8 = 255;
/// FREE sentinel
pub const FREE: u8 = 0;

/// 二元 / 梯度 costmap（u8 占据栅格）
#[derive(Debug, Clone)]
pub struct CostMap2D {
    frame_id: &'static str,
    width: usize,
    height: usize,
    /// 米/格
    resolution: f64,
    /// 占据值（u8）。255 表示 UNKNOWN。
    cells: Vec<u8>,
    /// 原点（世界系，左下角）
    origin: Point2,
}

impl CostMap2D {
    /// 构造新 costmap（默认全 FREE）。
    #[must_use]
    pub fn new(frame_id: &'static str, width: usize, height: usize, resolution: f64) -> Self {
        Self {
            frame_id,
            width,
            height,
            resolution,
            cells: vec![FREE; width * height],
            origin: Point2::new(0.0, 0.0),
        }
    }

    /// 构造全 FREE 的 costmap。
    #[must_use]
    pub fn new_all_free(
        frame_id: &'static str,
        width: usize,
        height: usize,
        resolution: f64,
    ) -> Self {
        Self::new(frame_id, width, height, resolution)
    }

    /// 构造全 OCCUPIED 的 costmap。
    #[must_use]
    pub fn new_all_occupied(
        frame_id: &'static str,
        width: usize,
        height: usize,
        resolution: f64,
    ) -> Self {
        let mut cm = Self::new(frame_id, width, height, resolution);
        for v in &mut cm.cells {
            *v = OCCUPIED;
        }
        cm
    }

    /// 构造全 UNKNOWN 的 costmap。
    #[must_use]
    pub fn new_all_unknown(
        frame_id: &'static str,
        width: usize,
        height: usize,
        resolution: f64,
    ) -> Self {
        let mut cm = Self::new(frame_id, width, height, resolution);
        for v in &mut cm.cells {
            *v = UNKNOWN;
        }
        cm
    }

    /// 设置原点（世界系左下角）。
    pub fn set_origin(&mut self, origin: Point2) {
        self.origin = origin;
    }

    /// frame_id
    #[must_use]
    pub fn frame_id(&self) -> &'static str {
        self.frame_id
    }

    /// 宽度（格）
    #[must_use]
    pub fn width(&self) -> usize {
        self.width
    }

    /// 高度（格）
    #[must_use]
    pub fn height(&self) -> usize {
        self.height
    }

    /// 分辨率（米/格）
    #[must_use]
    pub fn resolution(&self) -> f64 {
        self.resolution
    }

    /// 设置某格占据值
    pub fn set(&mut self, x: usize, y: usize, v: u8) {
        if x < self.width && y < self.height {
            self.cells[y * self.width + x] = v;
        }
    }

    /// 设置某格为 UNKNOWN
    pub fn set_unknown(&mut self, x: usize, y: usize) {
        self.set(x, y, UNKNOWN);
    }

    /// 读取某格原始值
    #[must_use]
    pub fn get(&self, x: usize, y: usize) -> u8 {
        self.cells[y * self.width + x]
    }

    /// 检查某格是否 UNKNOWN
    #[must_use]
    pub fn is_unknown(&self, x: usize, y: usize) -> bool {
        self.cells[y * self.width + x] == UNKNOWN
    }

    /// 读取某格值与是否 UNKNOWN（避免重复查表）
    #[inline]
    #[must_use]
    pub fn cell_value_with_unknown(&self, x: usize, y: usize) -> (u8, bool) {
        let v = self.cells[y * self.width + x];
        (v, v == UNKNOWN)
    }

    /// 是否在范围内
    #[inline]
    #[must_use]
    pub fn in_bounds(&self, x: i32, y: i32) -> bool {
        x >= 0 && y >= 0 && (x as usize) < self.width && (y as usize) < self.height
    }

    /// 世界坐标 -> 栅格坐标（floor）
    #[must_use]
    pub fn world_to_cell(&self, p: Point2) -> (i32, i32) {
        let fx = (p.x - self.origin.x) / self.resolution;
        let fy = (p.y - self.origin.y) / self.resolution;
        (fx.floor() as i32, fy.floor() as i32)
    }

    /// 栅格坐标中心 -> 世界坐标
    #[must_use]
    pub fn cell_center_world(&self, x: usize, y: usize) -> Point2 {
        Point2::new(
            self.origin.x + (x as f64 + 0.5) * self.resolution,
            self.origin.y + (y as f64 + 0.5) * self.resolution,
        )
    }

    /// world 点对应的栅格原始 cost（不带 inflation）
    #[must_use]
    pub fn cell_value(&self, p: Point2) -> u8 {
        let (x, y) = self.world_to_cell(p);
        if !self.in_bounds(x, y) {
            return OCCUPIED;
        }
        self.cells[y as usize * self.width + x as usize]
    }

    /// 拷贝 cells（用于 gradient costmap 重建）
    #[must_use]
    pub fn cells(&self) -> &[u8] {
        &self.cells
    }
}

// ===== rework22 P0-12:Meilhon 9-邻居 EDT mask =====
//
// Meilhon 9-neighbor Chamfer mask(Meilhon 1999 / Cuisenaire 2002),
// 误差 < 0.5%,明显优于 Chamfer-8(误差 ~1-2%)。
// 5×5 forward mask:
//
//     2  3  2
//     3  e
//  (forward 时,9 个 backward 邻居已由 backward pass 处理)
//
// 单元 cell 距离权重:corner=2, edge=3, center=e=0。
// 反向 pass 用镜像 mask。
// rework22 P1 #2:这些常量现由 nuwax_mapping::occupancy::gradient 统一实装,这里保留为参考
#[allow(dead_code)]
const MEILHON_A: f64 = 2.0; // 对角(左上)
#[allow(dead_code)]
const MEILHON_B: f64 = 3.0; // 边(上)
#[allow(dead_code)]
const MEILHON_C: f64 = 2.0; // 对角(右上)
#[allow(dead_code)]
const MEILHON_D: f64 = 3.0; // 边(左)


/// 梯度策略（与 Python `GradientStrategy` 对应）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GradientStrategy {
    /// voronoi gradient
    Voronoi,
    /// 简单梯度
    Gradient,
}

/// `NavigationMap`：缓存最近一次 binary costmap，提供 gradient 接口
///
/// 设计说明：
/// - `update(costmap)` 缓存 binary。
/// - `make_gradient_costmap(robot_increase)`：构造 obstacle inflate + 距离梯度 costmap。
///   Phase 1 提供稳定 API 形状；具体 distance transform 算法（Chamfer / EDT / Voronoi）本阶段
///   使用 4-邻接 BFS 简单实现，能保证 binary -> gradient 的转换可工作、可测，但**非生产级精度**。
///   与 mapping 板块协作时将由 `nuwax-mapping::occupancy::gradient` 提供更精确实现替换。
pub struct NavigationMap {
    robot_width: f64,
    #[allow(dead_code)] // 策略参数供运行时切换 gradient 行为,Binary 矩阵调用方暂未查询
    strategy: GradientStrategy,
    binary: Option<CostMap2D>,
}

impl NavigationMap {
    /// 构造（默认 robot_width=0.5, voronoi）
    #[must_use]
    pub fn new(robot_width: f64, strategy: GradientStrategy) -> Self {
        Self {
            robot_width,
            strategy,
            binary: None,
        }
    }

    /// 更新 binary costmap
    pub fn update(&mut self, costmap: CostMap2D) {
        self.binary = Some(costmap);
    }

    /// 当前 binary costmap（深拷贝）
    ///
    /// 若 binary 缺失返回错误。
    pub fn binary_costmap(&self) -> Result<CostMap2D, crate::helpers::NavError> {
        self.binary
            .clone()
            .ok_or(crate::helpers::NavError::MapUnavailable)
    }

    /// robot width（米）
    #[must_use]
    pub fn robot_width(&self) -> f64 {
        self.robot_width
    }

    /// 构造 gradient costmap
    ///
    /// rework22 P1 #2 升级:删除 nuwax-astar 自带的 Chamfer-8 EDT 实现,改为调用
    /// `nuwax_mapping::occupancy::gradient::make_costmap` 统一实装。
    ///
    /// 流程:
    /// 1. 将 `CostMap2D` (binary) 转 `OccupancyGrid` 桥接
    ///    - OCCUPIED (100) → OccupancyValue::Occupied
    ///    - UNKNOWN (255) → OccupancyValue::Unknown (不参与膨胀,只标记不可走)
    ///    - FREE (0) → OccupancyValue::Free
    /// 2. 调用 `nuwax_mapping::occupancy::gradient::make_costmap`
    ///    (含 Chamfer-8 EDT + UNKNOWN 不参与膨胀)
    /// 3. 将结果转回 `CostMap2D`
    ///
    /// 注: `GradientStrategy::Voronoi` 路径仍由 mapping 端实现统一处理(暂未启用
    /// 完整 Voronoi;当前走 EDT)。
    #[must_use]
    pub fn make_gradient_costmap(&self, robot_increase: f64) -> Option<CostMap2D> {
        let binary = self.binary.as_ref()?;

        let width = binary.width();
        let height = binary.height();
        let resolution = binary.resolution();

        // 阶段 1: CostMap2D → OccupancyGrid 桥接
        let mut occ_grid = nuwax_mapping::occupancy::OccupancyGrid::new(
            width as u32,
            height as u32,
            resolution,
        );
        for y in 0..height {
            for x in 0..width {
                let v = binary.get(x, y);
                let mapped = if v == OCCUPIED {
                    nuwax_mapping::occupancy::types::OccupancyValue::Occupied
                } else if v == UNKNOWN {
                    nuwax_mapping::occupancy::types::OccupancyValue::Unknown
                } else {
                    nuwax_mapping::occupancy::types::OccupancyValue::Free
                };
                occ_grid.set(y as u32, x as u32, mapped);
            }
        }

        // 阶段 2: 调统一 make_costmap
        let cfg = nuwax_mapping::occupancy::gradient::CostMapConfig {
            resolution,
            robot_increase: robot_increase * self.robot_width,
            max_cost: 90,
        };
        let result = nuwax_mapping::occupancy::gradient::make_costmap(&occ_grid, &cfg);

        // 阶段 3: 结果 → CostMap2D 桥接
        // mapping make_costmap 用 255 标记"不可走"(Occupied 和 Unknown 都用 255);
        // nav-astar 用 OCCUPIED=100 标记 occupied, UNKNOWN=255 标记 unknown。
        // 这里根据原始 binary 还原 Occupied/Unknown 标记,保持 nav-astar 契约。
        let mut grad = CostMap2D::new(binary.frame_id(), width, height, resolution);
        grad.set_origin(binary.origin);
        for y in 0..height {
            for x in 0..width {
                let v = result.get(y, x);
                if v == 255 {
                    // 还原:原 binary 是 UNKNOWN 留 UNKNOWN,否则是 OCCUPIED
                    if binary.get(x, y) == UNKNOWN {
                        grad.set(x, y, UNKNOWN);
                    } else {
                        grad.set(x, y, OCCUPIED);
                    }
                } else {
                    grad.set(x, y, v);
                }
            }
        }

        let _ = self.strategy; // 保留 strategy 字段(API 兼容)
        Some(grad)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basic_world_to_cell_roundtrip() {
        let mut cm = CostMap2D::new("map", 10, 10, 0.1);
        cm.set_origin(Point2::new(1.0, 2.0));
        let p = Point2::new(1.55, 2.55);
        let (x, y) = cm.world_to_cell(p);
        assert_eq!((x, y), (5, 5));
        let center = cm.cell_center_world(5, 5);
        assert!((center.x - 1.55).abs() < 1e-9);
        assert!((center.y - 2.55).abs() < 1e-9);
    }

    #[test]
    fn negative_origin_handled() {
        let mut cm = CostMap2D::new("map", 10, 10, 0.5);
        cm.set_origin(Point2::new(-5.0, -5.0));
        let p = Point2::new(-5.0, -5.0); // cell (0,0)
        let (x, y) = cm.world_to_cell(p);
        assert_eq!((x, y), (0, 0));
    }

    #[test]
    fn cell_value_oob_returns_occupied() {
        let cm = CostMap2D::new("map", 10, 10, 0.1);
        let v = cm.cell_value(Point2::new(100.0, 100.0));
        assert_eq!(v, OCCUPIED);
    }

    #[test]
    fn navigation_map_requires_binary() {
        let nm = NavigationMap::new(0.5, GradientStrategy::Voronoi);
        assert!(nm.binary_costmap().is_err());
        assert!(nm.make_gradient_costmap(1.0).is_none());
    }

    #[test]
    fn gradient_inflation_blocks_neighbors() {
        let mut binary = CostMap2D::new("map", 12, 12, 0.1);
        binary.set(5, 5, OCCUPIED);
        let mut nm = NavigationMap::new(0.5, GradientStrategy::Voronoi);
        nm.update(binary);

        let grad = nm.make_gradient_costmap(1.0).expect("grad");
        // 距离 (5,5) 在 5 cells 内（robot_width=0.5, resolution=0.1, inflate=5 cells）应被 inflate 阻断
        // cell (5,5) 自身: OCCUPIED
        assert_eq!(grad.get(5, 5), OCCUPIED);
        // cell (6,5) 距离 1 -> OCCUPIED
        assert_eq!(grad.get(6, 5), OCCUPIED);
        // cell (10,5) 距离 5 -> 仍在半径内 -> OCCUPIED(grid 扩大到 12x12 容纳 5 cells 半径)
        assert_eq!(grad.get(10, 5), OCCUPIED);
        // cell (11,5) 距离 6 -> 超出半径,应为 EDT cost(< 100)
        assert!(grad.get(11, 5) < OCCUPIED);
    }

    #[test]
    fn gradient_far_cells_have_low_cost() {
        let mut binary = CostMap2D::new("map", 20, 20, 0.1);
        binary.set(10, 10, OCCUPIED);
        let mut nm = NavigationMap::new(0.5, GradientStrategy::Gradient);
        nm.update(binary);

        let grad = nm.make_gradient_costmap(1.0).expect("grad");
        // (0,0) 距离 (10,10) 远，应为低 cost
        let v = grad.get(0, 0);
        assert!(v < 50, "expected low cost, got {v}");
    }

    /// P1 #2 升级验证:UNKNOWN cell 在 gradient 中保留为 UNKNOWN=255,
    /// 而非旧合约(OCCUPIED=100)。UNKNOWN 不参与障碍膨胀,但仍是"不可走"。
    #[test]
    fn unknown_in_binary_kept_as_unknown_in_gradient() {
        let mut binary = CostMap2D::new("map", 10, 10, 0.1);
        binary.set_unknown(3, 3);
        let mut nm = NavigationMap::new(0.3, GradientStrategy::Voronoi);
        nm.update(binary);

        let grad = nm.make_gradient_costmap(1.0).expect("grad");
        // P1 #2: UNKNOWN 在 gradient 中**保留为 UNKNOWN**(不再退化为 OCCUPIED)
        assert_eq!(grad.get(3, 3), UNKNOWN);
    }

    #[test]
    fn gradient_edt_distance_monotonic_farther_cells_have_lower_or_equal_cost() {
        let mut binary = CostMap2D::new("map", 21, 21, 0.1);
        binary.set(10, 10, OCCUPIED);
        let mut nm = NavigationMap::new(0.5, GradientStrategy::Voronoi);
        nm.update(binary);
        let grad = nm.make_gradient_costmap(1.0).expect("grad");

        // (10,10) = OCCUPIED
        assert_eq!(grad.get(10, 10), OCCUPIED);
        // 对角 (11,11) 距 obstacle sqrt(2) cell,正交 (10,11) 距 obstacle 1 cell
        // 应满足 cost(11,11) >= cost(10,11) - tolerance(chamfer-8 与 EDT 近似一致)
        let c_orth = grad.get(10, 11);
        let c_diag = grad.get(11, 11);
        assert!(c_diag >= c_orth, "diag {c_diag} should be >= orth {c_orth}");
        // 远处 cell cost 更低
        let c_far = grad.get(0, 0);
        assert!(c_far <= c_orth, "far {c_far} should be <= orth {c_orth}");
    }

    /// H-15 验证:voronoi 性质——两 obstacle 之间的"最远点"应在中间或附近。
    /// 对 chamfer EDT 而言,中点 cell 应该是局部最大值(cost 最高表示距 obstacle 最远)。
    /// 注意:由于 cost 编码 `90 - d*9`(距 obstacle 越远 cost 越低),这里的断言反过来:
    /// 中点 cost 应该 ≥ 远离两侧的中点外 cell 的 cost(因为更靠近 obstacle)。
    #[test]
    fn gradient_voronoi_midpoint_is_local_max_distance() {
        // 两个 obstacle 在 (2,10) 和 (18,10),中间 cell (10,10) 应是局部距 obstacle 最远点
        let mut binary = CostMap2D::new("map", 21, 21, 0.1);
        binary.set(2, 10, OCCUPIED);
        binary.set(18, 10, OCCUPIED);
        let mut nm = NavigationMap::new(0.0, GradientStrategy::Voronoi); // 不 inflate
        nm.update(binary);
        let grad = nm.make_gradient_costmap(1.0).expect("grad");

        // 中点 cell 不应是 OCCUPIED(没被 obstacle 标记)
        assert_ne!(grad.get(10, 10), OCCUPIED);
        // 中点应比直接相邻的 (9,10) 距 obstacle 更远(因对称),
        // distance encoding cost(10,10) ≤ cost(9,10)
        let mid = grad.get(10, 10);
        let near_mid = grad.get(11, 10);
        assert!(mid <= near_mid, "mid cost {mid} ≤ near-mid cost {near_mid}");
    }
}
