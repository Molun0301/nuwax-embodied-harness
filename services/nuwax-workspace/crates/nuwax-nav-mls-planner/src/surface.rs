// Copyright 2026 Nuwax Inc.
// SPDX-License-Identifier: Apache-2.0

//! 规划、执行和探索共用的不可变可通行表面，不持有业务地图或任务身份。

use std::sync::Arc;

use ahash::{AHashMap, AHashSet};
use image::{GrayImage, ImageBuffer, Luma};
use imageproc::integral_image::{integral_image, sum_image_pixels};
use validator::Validate;

use crate::adjacency::{
    build_surface_cells, build_surface_lookup, CellId, SurfaceCells, SurfaceLookup,
};
use crate::edges::PlannerGraph;
use crate::mls_planner::Config;
use crate::nodes::penalty_of;
use crate::surfaces::ColumnIz;
use crate::voxel::{surface_point_xyz, voxelize, VoxelKey};

mod observations;
pub use observations::{SurfaceObservation, SurfaceObservations};
mod body;
pub use body::SurfaceBodyEnvelope;
mod passage;
pub use passage::SurfaceBodyPassageCandidate;
mod body_query;
pub use body_query::SurfaceBodyQueryView;
mod frontiers;
pub use frontiers::SurfaceFrontierCacheStats;
mod routing;
mod centering;
mod regions;
mod wall;
use routing::frontier_point_distance;
pub use routing::SurfaceGoalCosts;
mod update;
pub use update::{SurfaceBuildKind, SurfaceBuildStats, SurfaceColumnUpdate, SurfaceMapBuild, SurfaceMapBuilder};

const MAX_SOURCE_POINTS: usize = 2_000_000;
const MAX_PROFILE_CELLS: i32 = 128;
const MAX_SEGMENT_CELLS: i64 = 1_000_000;
const MAX_CONTOUR_CELLS: i64 = 65_536;
const MAX_VIRTUAL_MASK_CELLS: usize = 16 * 1024 * 1024;
const MAX_VIRTUAL_MASK_DIMENSION: u32 = 65_536;
const DIRECTIONS: [(i32, i32); 4] = [(-1, 0), (1, 0), (0, -1), (0, 1)];

/// 共享几何拒绝原因；没有路径或支撑不能解释成成功到达。
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum SurfaceError {
    /// 配置或几何工作量不满足有界计算约束。
    #[error("invalid surface configuration: {0}")]
    InvalidConfig(String),
    /// 输入云为空或包含无效坐标。
    #[error("invalid surface point cloud")]
    InvalidPointCloud,
    /// 源分辨率无法提供声明的支撑/踏面细节。
    #[error("source resolution {source_m}m cannot support planner resolution {planner_m}m")]
    InsufficientResolution {
        /// 源云实际体素大小。
        source_m: f32,
        /// 规划体素大小。
        planner_m: f32,
    },
    /// 机体/支撑点、姿态或标定偏移非法。
    #[error("invalid body/support pose or calibrated offset")]
    InvalidPose,
    /// 自身水平单元没有满足高度约束的真实支撑。
    #[error("no observed support in the requested column")]
    MissingSupport,
    /// 新进入的精细位置缺少合格观察。
    #[error("近场支撑{cell:?}不可用：{reason}，采样={observed_ns}，检查={checked_ns}")]
    NearFieldObservation {
        /// 固定精度的实际三维格。
        cell: [i32; 3],
        /// 缺失、质量或时效原因。
        reason: &'static str,
        /// 已有真实采样时间，未知为零。
        observed_ns: u64,
        /// 包含制动到达时间的检查时刻。
        checked_ns: u64,
    },
    /// 支撑点离墙或边缘太近。
    #[error("surface does not satisfy the required clearance")]
    InsufficientClearance,
    /// 虚拟禁区或其足迹余量禁止使用该支撑点。
    #[error("support is blocked by the virtual navigation boundary")]
    VirtualObstacle,
    /// 连接经过未知支撑、非法台阶、过陡坡道或受阻轮廓。
    #[error("surface segment is not traversable")]
    UntraversableSegment,
    /// 轮廓/边界不是合法有限多边形。
    #[error("invalid surface boundary or footprint contour")]
    InvalidBoundary,
    /// 起终点没有真实可通行连接。
    #[error("no connected traversable surface path")]
    NoPath,
}

/// 支撑定位结果；单元标识只属于当前快照，跨快照须重新查询。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceLocation {
    /// 支撑面体素坐标。
    pub cell: VoxelKey,
    /// 请求的支撑参考点，保留完整姿态转换后的实际坐标。
    pub support_xyz: [f64; 3],
    /// 关联单元的表面中心。
    pub surface_xyz: [f64; 3],
}

/// 经过真实几何检验的相邻表面连接。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceTransition {
    /// 无离散高度变化。
    Level,
    /// 满足连续高度剖面坡度约束。
    Ramp,
    /// 两侧有足够踏面的上台阶。
    StepUp,
    /// 两侧有足够踏面的下台阶。
    StepDown,
}

/// 前沿观察候选，只表达当前几何，不保存请求/访问/失败历史。
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceFrontier {
    /// 可到达的支撑点。
    pub support_xyz: [f64; 3],
    /// 当前视距内缺少支撑证据的独立列数乘单元面积。
    pub gain: f64,
    /// 沿边界内有向安全支撑图的路径长度（米），包含起点到单元中心的偏移。
    pub travel_cost: f64,
    /// 所属支撑单元，便于同快照内聚类。
    pub cell: VoxelKey,
}

type HeightBounds = im::HashMap<VoxelKey, [f32;2], ahash::RandomState>;

struct SurfaceData {
    // 仅由受限机体查询视图持有；邻接按请求核验，不向路由消费者公开。
    body_query_only: bool,
    body_envelope: Option<Arc<SurfaceBodyEnvelope>>,
    height_bounds: Arc<HeightBounds>,
    frontier_cache: frontiers::CacheSlot,
    body_cache: body::CacheSlot,
    routing_cache: std::sync::Mutex<routing::RoutingCache>,
    graph: Arc<PlannerGraph>,
    regions: Arc<regions::RegionGraph>,
    physical_graph: Arc<PlannerGraph>,
    occupied_columns: Arc<ColumnIz>,
    occupied_voxels: usize,
    virtual_obstacles: Option<Arc<VirtualObstacleMask>>,
    virtual_blocked_cells: Arc<AHashSet<CellId>>,
    config: Config,
    source_voxel_size_m: f32,
    profile_cells: i32,
    candidate_step_cells: i32,
}

struct VirtualObstacleMask {
    origin: [f64; 2],
    resolution: f64,
    width: u32,
    height: u32,
    extent: [f64; 2],
    integral: ImageBuffer<Luma<u32>, Vec<u32>>,
}

impl VirtualObstacleMask {
    fn new(
        origin: [f64; 2],
        resolution: f64,
        width: u32,
        height: u32,
        blocked: &[bool],
    ) -> Result<Self, SurfaceError> {
        let cells = (width as usize).checked_mul(height as usize);
        let extent = [
            origin[0] + f64::from(width) * resolution,
            origin[1] + f64::from(height) * resolution,
        ];
        if origin.iter().any(|value| !value.is_finite())
            || !resolution.is_finite()
            || resolution <= 0.0
            || width == 0
            || height == 0
            || width > MAX_VIRTUAL_MASK_DIMENSION
            || height > MAX_VIRTUAL_MASK_DIMENSION
            || cells != Some(blocked.len())
            || blocked.len() > MAX_VIRTUAL_MASK_CELLS
            || extent.iter().any(|value| !value.is_finite())
            || extent[0] <= origin[0]
            || extent[1] <= origin[1]
        {
            return Err(SurfaceError::InvalidConfig(
                "invalid virtual obstacle mask geometry or payload".into(),
            ));
        }
        let image = GrayImage::from_fn(width, height, |x, y| {
            Luma([u8::from(blocked[y as usize * width as usize + x as usize])])
        });
        Ok(Self {
            origin,
            resolution,
            width,
            height,
            extent,
            integral: integral_image::<_, u32>(&image),
        })
    }

    fn permits_cell(&self, cell: VoxelKey, voxel_size: f64, clearance: f64) -> bool {
        // 单元全覆盖加足迹外包矩形，保守覆盖机器人半宽，细于MLS分辨率的线也不能漏掉。
        let lower = [
            f64::from(cell.0) * voxel_size - clearance,
            f64::from(cell.1) * voxel_size - clearance,
        ];
        let upper = [
            (f64::from(cell.0) + 1.0) * voxel_size + clearance,
            (f64::from(cell.1) + 1.0) * voxel_size + clearance,
        ];
        if lower[0] < self.origin[0]
            || lower[1] < self.origin[1]
            || upper[0] > self.extent[0]
            || upper[1] > self.extent[1]
        {
            return false;
        }
        let left = ((lower[0] - self.origin[0]) / self.resolution).floor() as u32;
        let top = ((lower[1] - self.origin[1]) / self.resolution).floor() as u32;
        let right = (((upper[0] - self.origin[0]) / self.resolution).ceil() as u32)
            .saturating_sub(1)
            .min(self.width - 1);
        let bottom = (((upper[1] - self.origin[1]) / self.resolution).ceil() as u32)
            .saturating_sub(1)
            .min(self.height - 1);
        left <= right
            && top <= bottom
            && sum_image_pixels(&self.integral, left, top, right, bottom)[0] == 0
    }
}

/// 一次地图更新建立一次；克隆仅共享不可变表面和邻接图。
#[derive(Clone)]
pub struct SurfaceSnapshot {
    data: Arc<SurfaceData>,
    observations: Option<Arc<SurfaceObservations>>,
    current_body_support: Option<Arc<body::CurrentBodySupport>>,
    /// 同一不可变快照与精确位姿的检查缓存；任何几何或观察替换均清空。
    body_query_cache: Arc<std::sync::Mutex<body::BodyQueryCache>>,
}

impl std::fmt::Debug for SurfaceSnapshot {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("SurfaceSnapshot")
            .field("cells", &self.data.graph.cells.ids().count())
            .field("voxel_size", &self.data.config.voxel_size)
            .field("source_voxel_size_m", &self.data.source_voxel_size_m)
            .finish()
    }
}

impl SurfaceSnapshot {
    /// 使用已提交的真实点云建立表面；来源身份与清空语义由外层拥有方处理。
    pub fn from_points(
        points: &[(f32, f32, f32)],
        config: Config,
        source_voxel_size_m: f32,
    ) -> Result<Self, SurfaceError> {
        validate_config(&config, source_voxel_size_m)?;
        let columns = update::voxel_columns(points, config.voxel_size)?;
        let heights=update::height_bounds(points, config.voxel_size, source_voxel_size_m);
        update::from_columns(columns, heights, config, source_voxel_size_m,None)
    }

    /// 从同一物理图基底应用当前完整纯虚拟障碍层，覆盖全部楼层。
    /// 全阻断是合法结果；替换为空闲层可以恢复被旧虚拟层阻断的几何。
    pub fn with_virtual_obstacles(
        &self,
        origin: [f64; 2],
        resolution: f64,
        width: u32,
        height: u32,
        blocked: &[bool],
    ) -> Result<SurfaceSnapshot, SurfaceError> {
        let mask = Arc::new(VirtualObstacleMask::new(
            origin, resolution, width, height, blocked,
        )?);
        self.with_virtual_mask(mask)
    }

    fn with_virtual_mask(&self, mask: Arc<VirtualObstacleMask>) -> Result<Self, SurfaceError> {
        if let Some(current) = &self.current_body_support {
            return current.source.with_observations(self.observations.clone())
                .with_virtual_mask(mask)?.with_current_body_support(current.body, current.quaternion);
        }
        let physical = Arc::clone(&self.data.physical_graph);
        let mut graph = physical.as_ref().clone();
        let excluded: AHashSet<_> = graph
            .cells
            .ids()
            .filter(|&id| {
                !mask.permits_cell(
                    graph.cells.coord(id),
                    f64::from(self.config().voxel_size),
                    f64::from(self.config().wall_clearance_m),
                )
            })
            .collect();
        for (id, edges) in graph.cells.iter_edges_mut() {
            edges.retain(|edge| !excluded.contains(&id) && !excluded.contains(&edge.dest));
        }
        let regions=Arc::new(regions::RegionGraph::full(&graph,self.config().voxel_size));
        let result=Self {
            observations: self.observations.clone(),
            current_body_support: None,
            body_query_cache: Default::default(),
            data: Arc::new(SurfaceData {
                body_query_only: false,
                frontier_cache: Default::default(),
                body_cache: Default::default(),
                routing_cache: Default::default(),
                body_envelope: self.data.body_envelope.clone(),
                height_bounds: Arc::clone(&self.data.height_bounds),
                graph: Arc::new(graph),
                regions,
                physical_graph: physical,
                occupied_columns: Arc::clone(&self.data.occupied_columns),
                occupied_voxels: self.data.occupied_voxels,
                virtual_obstacles: Some(mask),
                virtual_blocked_cells: Arc::new(excluded),
                config: self.config().clone(),
                source_voxel_size_m: self.data.source_voxel_size_m,
                profile_cells: self.data.profile_cells,
                candidate_step_cells: self.data.candidate_step_cells,
            }),
        };
        Ok(result)
    }

    /// 生成快照时使用的同一份通行配置。
    pub fn config(&self) -> &Config {
        &self.data.config
    }

    /// 输入云实际分辨率，不能由规划体素反推。
    pub fn source_voxel_size_m(&self) -> f32 {
        self.data.source_voxel_size_m
    }

    /// 查询真实支撑；地面参考点只允许一个规划体素的高度量化误差。
    pub fn locate_support(&self, support_xyz: [f64; 3]) -> Result<SurfaceLocation, SurfaceError> {
        self.locate_near_support(support_xyz, f64::from(self.source_voxel_size_m()))
    }

    /// 按完整机体姿态旋转标定的向下偏移；距离参数必须为正。
    pub fn locate_support_from_body(
        &self,
        body_xyz: [f64; 3],
        body_xyzw: [f64; 4],
        body_to_support_z_m: f64,
    ) -> Result<SurfaceLocation, SurfaceError> {
        let key=[body_xyz[0],body_xyz[1],body_xyz[2],body_xyzw[0],body_xyzw[1],body_xyzw[2],body_xyzw[3],body_to_support_z_m]
            .map(f64::to_bits);
        if let Some(result)=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner()).supports.get(&key).cloned() {
            return result;
        }
        let result=(|| {
            let support = self.support_position_from_body(body_xyz, body_xyzw, body_to_support_z_m)?;
            match self.locate_support(support) {
                Ok(location)=>Ok(location),
                Err(SurfaceError::MissingSupport)=>self.locate_step_contact(body_xyz,body_xyzw,support,body_to_support_z_m),
                Err(error)=>Err(error),
            }
        })();
        let mut cache=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner());
        if cache.supports.len()>=8192 {cache.supports.clear();}
        cache.supports.insert(key,result.clone());
        result
    }

    /// 导航目标高度用于选择支撑层，不作为平面底盘的可控自由度。
    pub fn locate_goal_support_from_body(
        &self,
        body_xyz: [f64; 3],
        body_xyzw: [f64; 4],
        body_to_support_z_m: f64,
    ) -> Result<SurfaceLocation, SurfaceError> {
        let support = self.support_position_from_body(body_xyz, body_xyzw, body_to_support_z_m)?;
        let tolerance = self.body_envelope().map_or(0.0, |model|
            model.max_step_up_m.max(model.max_step_down_m) + model.support_uncertainty_m)
            + f64::from(self.source_voxel_size_m());
        self.locate_near_support(support, tolerance)
    }

    /// 只按标定转换参考点，不宣称该处已被观测；用于远端路径的前瞻方向参考。
    pub fn support_position_from_body(
        &self,
        body_xyz: [f64; 3],
        body_xyzw: [f64; 4],
        body_to_support_z_m: f64,
    ) -> Result<[f64; 3], SurfaceError> {
        if body_xyz.iter().any(|value| !value.is_finite()) {
            return Err(SurfaceError::InvalidPose);
        }
        let offset = rotated_support_offset(body_xyzw, body_to_support_z_m)?;
        let support = std::array::from_fn(|index| body_xyz[index] + offset[index]);
        if support.iter().any(|value| !value.is_finite()) {
            return Err(SurfaceError::InvalidPose);
        }
        Ok(support)
    }

    /// 已明确支撑点和机体姿态时，将相同标定反用于路径机体参考点。
    pub fn body_position_from_support(
        &self,
        support_xyz: [f64; 3],
        body_xyzw: [f64; 4],
        body_to_support_z_m: f64,
    ) -> Result<[f64; 3], SurfaceError> {
        self.locate_support(support_xyz)?;
        let offset = rotated_support_offset(body_xyzw, body_to_support_z_m)?;
        Ok(std::array::from_fn(|index| {
            support_xyz[index] - offset[index]
        }))
    }

    /// 当前表面单元；不可通行单元仍可供前沿的可见性检查使用。
    pub fn cell_keys(&self) -> impl Iterator<Item = VoxelKey> + '_ {
        self.data
            .graph
            .cells
            .ids()
            .map(|id| self.data.graph.cells.coord(id))
    }

    /// 当前单元的可通行邻居；不能跨快照缓存内部稠密编号。
    pub fn neighbors(&self, cell: VoxelKey) -> Vec<VoxelKey> {
        self.data.graph.cells.id(cell).map_or_else(Vec::new, |id| {
            self.data
                .graph
                .cells
                .neighbors(id)
                .iter()
                .filter(|edge| edge.cost.is_finite())
                .map(|edge| self.data.graph.cells.coord(edge.dest))
                .collect()
        })
    }

    /// 读取某条真实可通行邻接的坡道/台阶分类。
    pub fn transition(&self, from: VoxelKey, to: VoxelKey) -> Option<SurfaceTransition> {
        if !self.has_edge(from, to) {
            return None;
        }
        classify_transition(
            &self.data.graph.surface_lookup,
            from,
            to,
            self.config(),
            &self.data.height_bounds,
            self.source_voxel_size_m(),
            self.data.profile_cells,
            self.data.candidate_step_cells,
        )
    }

    /// 验证整个三维连接段；对角跨格要求两侧通道均可通行。
    pub fn validate_segment(&self, from: [f64; 3], to: [f64; 3]) -> Result<(), SurfaceError> {
        let start = self.locate_support(from)?;
        let end = self.locate_support(to)?;
        if start.cell == end.cell {
            return Ok(());
        }
        let dx = i64::from(end.cell.0) - i64::from(start.cell.0);
        let dy = i64::from(end.cell.1) - i64::from(start.cell.1);
        let size = f64::from(self.config().voxel_size);
        let horizontal = [(to[0] - from[0]).abs(), (to[1] - from[1]).abs()];
        let steps = (horizontal[0].max(horizontal[1]) / size).ceil() as i64;
        if dx == 0 && dy == 0 || steps == 0 || steps > MAX_SEGMENT_CELLS {
            return Err(SurfaceError::UntraversableSegment);
        }
        let mut previous = start.cell;
        for sample in 1..=steps * 2 {
            let ratio = sample as f64 / (steps * 2) as f64;
            // 使用真实端点连线穿过的列，而非把两端换成中心后另画一条线。
            // 最后一次直接使用真实末点，避免大数相减把靠近零线的小坐标消成零。
            let point = if sample == steps * 2 { to } else {
                std::array::from_fn(|i| from[i] * (1.0 - ratio) + to[i] * ratio)
            };
            let ix = (point[0] / size).floor() as i32;
            let iy = (point[1] / size).floor() as i32;
            if (ix, iy) == (previous.0, previous.1) {
                continue;
            }
            let height = start.surface_xyz[2] * (1.0 - ratio) + end.surface_xyz[2] * ratio;
            let current = self
                .nearest_column_cell(
                    ix,
                    iy,
                    height,
                    f64::from(self.config().step_threshold_m + self.config().voxel_size),
                )
                .ok_or(SurfaceError::UntraversableSegment)?;
            if !self.cell_is_clear(current) || !self.connected_grid_step(previous, current) {
                return Err(SurfaceError::UntraversableSegment);
            }
            previous = current;
        }
        if previous != end.cell {
            return Err(SurfaceError::UntraversableSegment);
        }
        Ok(())
    }

    /// 为局部前瞻采样真实支撑高度，只验证到所请求比例的连接前缀。
    /// `to` 仅作为已规划方向/高度参考，不遍历远端；当前实测位姿仍须严格定位。
    pub fn sample_segment_support(
        &self,
        from: [f64; 3],
        to: [f64; 3],
        ratio: f64,
    ) -> Result<SurfaceLocation, SurfaceError> {
        self.locate_support(from)?;
        if !ratio.is_finite()
            || !(0.0..=1.0).contains(&ratio)
            || to.iter().any(|value| !value.is_finite())
        {
            return Err(SurfaceError::InvalidPose);
        }
        let desired = if ratio == 0.0 { from } else if ratio == 1.0 { to } else {
            std::array::from_fn(|index| from[index] * (1.0 - ratio) + to[index] * ratio)
        };
        let nearest = self.locate_near_support(
            desired,
            f64::from(self.config().step_threshold_m + self.config().voxel_size),
        )?;
        // 合成目标落在该列真实表面，不能用台阶中间的线性插值高度当作支撑。
        let supported = [desired[0], desired[1], nearest.surface_xyz[2]];
        self.validate_segment(from, supported)?;
        self.locate_support(supported)
    }

    /// 路径的全部端点及连接段使用同一份表面验证。
    pub fn validate_path(&self, path: &[[f64; 3]]) -> Result<(), SurfaceError> {
        let Some(first) = path.first() else {
            return Err(SurfaceError::NoPath);
        };
        self.locate_support(*first)?;
        for pair in path.windows(2) {
            self.validate_segment(pair[0], pair[1])?;
        }
        Ok(())
    }

    /// 验证支撑轮廓边缘及内部真实支撑，不能在轮廓中包住未知坑洞。
    /// 轮廓为地面参考点；内部高度按首个非退化三角形插值并受台阶差限制。
    pub fn validate_contour(&self, contour: &[[f64; 3]]) -> Result<(), SurfaceError> {
        let polygon: Vec<_> = contour.iter().map(|point| [point[0], point[1]]).collect();
        validate_boundary(&polygon)?;
        if contour.len() < 3 || contour.iter().flatten().any(|value| !value.is_finite()) {
            return Err(SurfaceError::InvalidBoundary);
        }
        for index in 0..contour.len() {
            self.validate_segment(contour[index], contour[(index + 1) % contour.len()])?;
        }
        let plane = contour[1..]
            .windows(2)
            .find_map(|pair| plane_from(contour[0], pair[0], pair[1]))
            .ok_or(SurfaceError::InvalidBoundary)?;
        let size = f64::from(self.config().voxel_size);
        let min_x = polygon
            .iter()
            .map(|point| point[0])
            .fold(f64::INFINITY, f64::min);
        let max_x = polygon
            .iter()
            .map(|point| point[0])
            .fold(f64::NEG_INFINITY, f64::max);
        let min_y = polygon
            .iter()
            .map(|point| point[1])
            .fold(f64::INFINITY, f64::min);
        let max_y = polygon
            .iter()
            .map(|point| point[1])
            .fold(f64::NEG_INFINITY, f64::max);
        let (x0, x1, y0, y1) = (
            (min_x / size).floor() as i64,
            (max_x / size).floor() as i64,
            (min_y / size).floor() as i64,
            (max_y / size).floor() as i64,
        );
        if (x1 - x0 + 1)
            .checked_mul(y1 - y0 + 1)
            .is_none_or(|count| count > MAX_CONTOUR_CELLS)
        {
            return Err(SurfaceError::InvalidBoundary);
        }
        for ix in x0..=x1 {
            for iy in y0..=y1 {
                let x = (ix as f64 + 0.5) * size;
                let y = (iy as f64 + 0.5) * size;
                if inside_boundary([x, y], &polygon) {
                    let z = plane[0] * x + plane[1] * y + plane[2];
                    self.locate_near_support(
                        [x, y, z],
                        f64::from(self.config().step_threshold_m) + size,
                    )?;
                }
            }
        }
        Ok(())
    }

    /// 在原有向支撑图上选路；软偏好启用时按公开乘子的累计代价比较，仍执行原共享段校验。
    pub fn plan(&self, start: [f64; 3], goal: [f64; 3]) -> Result<Vec<[f64; 3]>, SurfaceError> {
        let start_location = self.locate_support(start)?;
        let goal_location = self.locate_support(goal)?;
        if self.config().wall_buffer_m>0.0 && self.config().wall_buffer_weight>0.0 {
            let path=self.plan_on_cells(start_location,goal_location,&[])?;
            return self.simplify_flat_path(path,&[]);
        }
        let cells=&self.data.graph.cells;
        let start_id=cells.id(start_location.cell).ok_or(SurfaceError::NoPath)?;
        let goal_id=cells.id(goal_location.cell).ok_or(SurfaceError::NoPath)?;
        let (ids,_)=self.data.regions.path(cells,start_id,goal_id,self.config().voxel_size)?;
        let path=self.path_with_actual_endpoints(ids.into_iter().map(|id|self.cell_position(cells.coord(id))).collect(),start,goal)?;
        self.simplify_flat_path(path,&[])

    }

    /// 只沿边界内的真实邻接规划，凹边界不能被连线捷径穿过。
    pub fn plan_in_boundary(
        &self,
        start: [f64; 3],
        goal: [f64; 3],
        boundary: &[[f64; 2]],
    ) -> Result<Vec<[f64; 3]>, SurfaceError> {
        validate_boundary(boundary)?;
        if boundary.is_empty() {
            return self.plan(start, goal);
        }
        if !inside_boundary([start[0], start[1]], boundary)
            || !inside_boundary([goal[0], goal[1]], boundary)
        {
            return Err(SurfaceError::InvalidBoundary);
        }
        let start_location = self.locate_support(start)?;
        let goal_location = self.locate_support(goal)?;
        let path = self.plan_on_cells(start_location, goal_location, boundary)?;
        if path.windows(2).all(|pair| {
            segment_inside_boundary([pair[0][0], pair[0][1]], [pair[1][0], pair[1][1]], boundary)
        }) {
            self.simplify_flat_path(path, boundary)
        } else {
            Err(SurfaceError::NoPath)
        }
    }

    /// 仅简化同一支撑高度层；复用原规划器的净空代价，保留精确端点与台阶。
    fn simplify_flat_path(
        &self,
        path: Vec<[f64; 3]>,
        boundary: &[[f64; 2]],
    ) -> Result<Vec<[f64; 3]>, SurfaceError> {
        if path.len() <= 2 {
            return Ok(path);
        }
        let cells = path.iter().map(|point| self.locate_support(*point).map(|location| location.cell))
            .collect::<Result<Vec<_>, _>>()?;
        let config = self.config();
        let wall = crate::planner::WallCost {
            clearance_m: config.wall_clearance_m,
            buffer_m: config.wall_buffer_m,
            buffer_weight: config.wall_buffer_weight,
            voxel_size: config.voxel_size,
        };
        let metrics = |from, to| crate::planner::segment_metrics(
            &self.data.graph, from, to, self.data.candidate_step_cells, &wall,
        );
        let Some(penalties) = cells.windows(2).map(|pair| metrics(pair[0], pair[1]).map(|value| value.0))
            .collect::<Option<Vec<_>>>() else {
                // 无法取得原路线软代价时保留已验证路径，不放宽净空偏好。
                return Ok(path);
            };
        let Some(edge_costs)=path.windows(2).map(|pair|self.soft_segment_cost(pair[0],pair[1]))
            .collect::<Option<Vec<_>>>() else {return Ok(path);};
        let mut accumulated=Vec::with_capacity(path.len());accumulated.push(0.0);
        for cost in edge_costs {accumulated.push(accumulated.last().copied().unwrap()+cost);}
        let permits = |from: usize, to: usize| {
            cells[from..=to].iter().all(|cell| cell.2 == cells[from].2)
                && segment_inside_boundary(
                    [path[from][0], path[from][1]], [path[to][0], path[to][1]], boundary,
                )
                && metrics(cells[from], cells[to]).is_some_and(|(penalty, rise)| {
                    rise <= 1e-6 && penalty <= penalties[from..to].iter().copied().fold(1.0_f32, f32::max) + 1e-3
                })
                // 起终点贴墙时两条路线可以有相同峰值，但贴墙长直线的累计代价更高。
                && self.soft_segment_cost(path[from],path[to]).is_some_and(|cost| {
                    let original=accumulated[to]-accumulated[from];
                    cost<=original+1e-9*original.max(1.0)
                })
                // 代价不授予许可，真实端点连线仍通过原完整支撑段核验。
                && self.validate_segment(path[from], path[to]).is_ok()
        };
        if permits(0, path.len() - 1) {
            return Ok(vec![path[0], path[path.len() - 1]]);
        }
        // 全路径直连只尝试一次，后续限定64个节点窗口，避免大图平方级搜索。
        let mut result = vec![path[0]];
        let mut anchor = 0;
        while anchor + 1 < path.len() {
            let limit = (anchor + 64).min(path.len() - 1);
            let next = (anchor + 2..=limit).rev().find(|&next| permits(anchor, next))
                .unwrap_or(anchor + 1);
            result.push(path[next]);
            anchor = next;
        }
        Ok(result)
    }

    /// 按实际线段穿过各格的长度积累有限软代价，避免峰值或端点抽样抹掉居中收益。
    fn soft_segment_cost(&self,from:[f64;3],to:[f64;3])->Option<f64> {
        let delta: [f64;3]=std::array::from_fn(|i|to[i]-from[i]);
        let length=delta[0].hypot(delta[1]).hypot(delta[2]);
        if !length.is_finite() {return None;}
        if length==0.0 || self.config().wall_buffer_m<=0.0 || self.config().wall_buffer_weight<=0.0 {
            return Some(length);
        }
        let size=f64::from(self.config().voxel_size);let mut events=vec![0.0,1.0];
        for axis in 0..2 {
            if delta[axis]==0.0 {continue;}
            let low=(from[axis].min(to[axis])/size).floor() as i64+1;
            let high=(from[axis].max(to[axis])/size).floor() as i64;
            if high-low>MAX_SEGMENT_CELLS {return None;}
            for index in low..=high {
                let t=(index as f64*size-from[axis])/delta[axis];
                if t>0.0 && t<1.0 {events.push(t);}
            }
        }
        events.sort_by(f64::total_cmp);events.dedup();
        let mut cost=0.0;
        for pair in events.windows(2) {
            let t=(pair[0]+pair[1])*0.5;
            let point=std::array::from_fn(|i|from[i]*(1.0-t)+to[i]*t);
            let location=self.locate_near_support(point,
                f64::from(self.config().step_threshold_m+self.config().voxel_size)).ok()?;
            cost+=length*(pair[1]-pair[0])*self.soft_traversal_multiplier(location.cell);
        }
        cost.is_finite().then_some(cost)
    }

    fn plan_on_cells(
        &self,
        start_location: SurfaceLocation,
        goal_location: SurfaceLocation,
        boundary: &[[f64; 2]],
    ) -> Result<Vec<[f64; 3]>, SurfaceError> {
        let start = start_location.support_xyz;
        let goal = goal_location.support_xyz;
        let graph = &self.data.graph;
        let cells = &graph.cells;
        let start_id = cells.id(start_location.cell).ok_or(SurfaceError::NoPath)?;
        let goal_id = cells.id(goal_location.cell).ok_or(SurfaceError::NoPath)?;
        let permits=|from,to| {
            let a=self.cell_position(cells.coord(from));let b=self.cell_position(cells.coord(to));
            segment_inside_boundary([a[0],a[1]],[b[0],b[1]],boundary)
        };
        let (path,_)=if self.config().wall_buffer_m>0.0 && self.config().wall_buffer_weight>0.0 {
            let weights=std::cell::RefCell::new(AHashMap::new());
            let multiplier=|id| {
                if let Some(value)=weights.borrow().get(&id).copied() {return value;}
                let value=self.soft_traversal_multiplier(cells.coord(id));weights.borrow_mut().insert(id,value);value
            };
            regions::weighted_path_edges(cells,start_id,goal_id,self.config().voxel_size,permits,|id,edge| {
                (f64::from(edge.base_cost)*(multiplier(id)+multiplier(edge.dest))*0.5
                    +f64::from(self.config().step_penalty_weight)*f64::from(edge.rise)).max(f64::from(edge.cost))
            })?
        } else {regions::sparse_path_edges(cells,start_id,goal_id,self.config().voxel_size,permits)?};
        self.path_with_actual_endpoints(
            path.into_iter().map(|cell| self.cell_position(cells.coord(cell))).collect(),
            start, goal,
        )
    }

    fn path_with_actual_endpoints(&self, mut path:Vec<[f64;3]>,start:[f64;3],goal:[f64;3])->Result<Vec<[f64;3]>,SurfaceError> {
        if path.is_empty() {return Err(SurfaceError::NoPath);}
        if path.len()==1 {path=vec![start,goal];}
        else {path[0]=start;*path.last_mut().unwrap()=goal;}
        path.dedup();
        self.validate_path(&path)?;
        Ok(path)
    }

    /// 查询边界内从当前支撑面实际可达、且能观察未知支撑的安全停留点。
    /// 空边界表示不额外限制；未知列只计增益，永不成为运动端点。
    pub fn reachable_frontiers(
        &self,
        start: [f64; 3],
        boundary: &[[f64; 2]],
    ) -> Result<Vec<SurfaceFrontier>, SurfaceError> {
        validate_boundary(boundary)?;
        let location = self.locate_support(start)?;
        if !inside_boundary([start[0], start[1]], boundary) {
            return Err(SurfaceError::InvalidBoundary);
        }
        let start_id = self
            .data
            .graph
            .cells
            .id(location.cell)
            .ok_or(SurfaceError::MissingSupport)?;
        let distances = self.frontier_distances(start_id, boundary, None);
        let start_offset = frontier_point_distance(start, self.cell_position(location.cell));
        let index = self.frontier_index(boundary);
        let mut frontiers = Vec::new();
        for (&cell, &gain) in &index.gains {
            let Some(id) = self.data.graph.cells.id(cell) else {
                continue;
            };
            if distances[id as usize].is_finite() {
                frontiers.push(SurfaceFrontier {
                    support_xyz: self.cell_position(cell),
                    gain,
                    travel_cost: distances[id as usize] + start_offset,
                    cell,
                });
            }
        }
        frontiers.sort_by_key(|frontier| frontier.cell);
        Ok(frontiers)
    }

    fn locate_near_support(
        &self,
        support_xyz: [f64; 3],
        tolerance: f64,
    ) -> Result<SurfaceLocation, SurfaceError> {
        let size = f64::from(self.config().voxel_size);
        if support_xyz
            .iter()
            .any(|value| !value.is_finite() || (value / size).abs() >= f64::from(i32::MAX) * 0.5)
        {
            return Err(SurfaceError::InvalidPose);
        }
        let ix = (support_xyz[0] / size).floor() as i32;
        let iy = (support_xyz[1] / size).floor() as i32;
        let cell = self
            .nearest_column_cell(ix, iy, support_xyz[2], tolerance)
            .ok_or(SurfaceError::MissingSupport)?;
        if self.current_body_support.as_ref().is_some_and(|current| {
            current.cells.contains(&cell)
                && !inside_boundary([support_xyz[0], support_xyz[1]], &current.polygon)
        }) {
            return Err(SurfaceError::MissingSupport);
        }
        if self
            .data
            .graph
            .cells
            .id(cell)
            .is_some_and(|id| self.data.virtual_blocked_cells.contains(&id))
            || (self.data.body_query_only && self.data.virtual_obstacles.as_ref().is_some_and(|mask|
                !mask.permits_cell(cell,size,0.0)))
        {
            return Err(SurfaceError::VirtualObstacle);
        }
        if !self.cell_is_clear(cell) {
            return Err(SurfaceError::InsufficientClearance);
        }
        Ok(SurfaceLocation {
            cell,
            support_xyz,
            surface_xyz: self.cell_position(cell),
        })
    }

    fn cell_position(&self, cell: VoxelKey) -> [f64; 3] {
        let point = surface_point_xyz(cell.0, cell.1, cell.2, self.config().voxel_size);
        [f64::from(point.0), f64::from(point.1), f64::from(self.data.height_bounds.get(&cell).map_or(point.2, |bounds|bounds[1]))]
    }

    fn nearest_column_cell(
        &self,
        ix: i32,
        iy: i32,
        height: f64,
        tolerance: f64,
    ) -> Option<VoxelKey> {
        self.data
            .graph
            .surface_lookup
            .get(&(ix, iy))?
            .iter()
            .map(|&iz| {
                (
                    (self.cell_position((ix, iy, iz))[2] - height).abs(),
                    (ix, iy, iz),
                )
            })
            .filter(|(distance, _)| {
                *distance <= tolerance + f64::from(self.config().voxel_size) * 1e-5
            })
            .min_by(|left, right| left.0.total_cmp(&right.0))
            .map(|(_, cell)| cell)
    }

    fn cell_is_clear(&self, cell: VoxelKey) -> bool {
        if self.data.body_query_only {return self.body_query_cell_is_clear(cell);}
        self.data.graph.cells.id(cell).is_some_and(|id| {
            if self.data.virtual_blocked_cells.contains(&id) {
                return false;
            }
            let clearance = self
                .data
                .graph
                .wall_state
                .dist
                .get(id as usize)
                .copied()
                .unwrap_or(0.0);
            penalty_of(
                clearance,
                self.config().wall_clearance_m,
                self.config().wall_buffer_m,
                self.config().wall_buffer_weight,
            )
            .is_finite()
        })
    }

    fn has_geometric_edge(&self, from: VoxelKey, to: VoxelKey) -> bool {
        if self.data.body_query_only {return self.body_query_transition(from,to).is_some();}
        let (Some(from), Some(to)) = (self.data.graph.cells.id(from), self.data.graph.cells.id(to))
        else {
            return false;
        };
        self.data
            .graph
            .cells
            .neighbors(from)
            .iter()
            .any(|edge| edge.dest == to)
    }

    fn has_edge(&self, from: VoxelKey, to: VoxelKey) -> bool {
        if self.data.body_query_only {return self.body_query_edge_is_allowed(from,to);}
        let (Some(from), Some(to)) = (self.data.graph.cells.id(from), self.data.graph.cells.id(to))
        else {
            return false;
        };
        self.data
            .graph
            .cells
            .neighbors(from)
            .iter()
            .any(|edge| edge.dest == to && edge.cost.is_finite())
    }

    fn connected_grid_step(&self, from: VoxelKey, to: VoxelKey) -> bool {
        let (dx, dy) = (to.0 - from.0, to.1 - from.1);
        if dx.abs() + dy.abs() == 1 {
            return self.has_edge(from, to);
        }
        if dx.abs() != 1 || dy.abs() != 1 {
            return false;
        }
        [(to.0, from.1), (from.0, to.1)].into_iter().all(|column| {
            self.data
                .graph
                .surface_lookup
                .get(&column)
                .is_some_and(|heights| {
                    heights.iter().any(|&height| {
                        let middle = (column.0, column.1, height);
                        self.has_edge(from, middle) && self.has_edge(middle, to)
                    })
                })
        })
    }
}

fn validate_config(config: &Config, source_voxel: f32) -> Result<(i32, i32), SurfaceError> {
    config
        .validate()
        .map_err(|error| SurfaceError::InvalidConfig(error.to_string()))?;
    let values = [
        config.voxel_size,
        config.robot_height,
        config.surface_closing_radius,
        config.node_spacing_m,
        config.wall_clearance_m,
        config.wall_buffer_m,
        config.wall_buffer_weight,
        config.step_threshold_m,
        config.max_slope_deg,
        config.min_step_tread_depth_m,
        config.step_penalty_weight,
        config.goal_tolerance,
        config.viz_publish_hz,
    ];
    if values.iter().any(|value| !value.is_finite())
        || !(config.wall_clearance_m+config.wall_buffer_m).is_finite()
        || config.world_frame.trim().is_empty() {
        return Err(SurfaceError::InvalidConfig(
            "non-finite value or missing frame".into(),
        ));
    }
    if config.surface_closing_radius != 0.0 {
        return Err(SurfaceError::InvalidConfig(
            "shared execution surface must not synthesize support by closing holes".into(),
        ));
    }
    if !source_voxel.is_finite()
        || source_voxel <= 0.0
        || source_voxel > config.voxel_size * (1.0 + 1e-5)
        || config.min_step_tread_depth_m + 1e-5 < config.voxel_size * 2.0
    {
        return Err(SurfaceError::InsufficientResolution {
            source_m: source_voxel,
            planner_m: config.voxel_size,
        });
    }
    let profile = (config.min_step_tread_depth_m / config.voxel_size).ceil() as i32;
    let ramp_jump = config.max_slope_deg.to_radians().tan() * config.voxel_size + config.voxel_size;
    let candidate_step = (config.step_threshold_m.max(ramp_jump) / config.voxel_size).ceil() as i32;
    if profile > MAX_PROFILE_CELLS
        || candidate_step > MAX_PROFILE_CELLS
        || config.robot_height / config.voxel_size > 4096.0
        || config.wall_clearance_m / config.voxel_size > MAX_PROFILE_CELLS as f32
    {
        return Err(SurfaceError::InvalidConfig(
            "geometry profile exceeds bounded neighborhood".into(),
        ));
    }
    Ok((profile, candidate_step))
}

fn nearest_z(
    lookup: &SurfaceLookup,
    x: i32,
    y: i32,
    height: i32,
    max_step: i32,
) -> Option<VoxelKey> {
    lookup
        .get(&(x, y))?
        .iter()
        .copied()
        .filter(|candidate| (candidate - height).abs() <= max_step)
        .min_by_key(|candidate| (candidate - height).abs())
        .map(|height| (x, y, height))
}

fn side_profile(
    lookup: &SurfaceLookup,
    start: VoxelKey,
    direction: (i32, i32),
    count: i32,
    max_step: i32,
    bounds: &HeightBounds,
    size: f32,
) -> Vec<f32> {
    let mut cell = start;
    let value=|cell:VoxelKey| bounds.get(&cell).map_or((cell.2 as f32+1.0)*size,|bounds|bounds[1])/size;
    let mut heights = vec![value(cell)];
    for _ in 0..count {
        let Some(next) = nearest_z(
            lookup,
            cell.0 + direction.0,
            cell.1 + direction.1,
            cell.2,
            max_step,
        ) else {break;};
        cell = next;
        heights.push(value(cell));
    }
    heights
}

fn classify_transition(
    lookup: &SurfaceLookup,
    from: VoxelKey,
    to: VoxelKey,
    config: &Config,
    bounds: &HeightBounds,
    source_size: f32,
    profile_cells: i32,
    candidate_step_cells: i32,
) -> Option<SurfaceTransition> {
    let (dx, dy) = (to.0 - from.0, to.1 - from.1);
    let height=|cell:VoxelKey| bounds.get(&cell).map_or((cell.2 as f32+1.0)*config.voxel_size,|bounds|bounds[1]);
    let dz=(height(to)-height(from))/config.voxel_size;
    let quantum=source_size/config.voxel_size;
    if dx.abs() + dy.abs() != 1 {
        return None;
    }
    if dz.abs() < 1e-5 {
        return Some(SurfaceTransition::Level);
    }
    // 同一地面重复观测会形成跨粗格的连续厚层。层顶的一格抖动不能
    // 因窗口远端的台阶而切断眼前平地；必须有真实重叠带，不能补空隙。
    if dz.abs() <= quantum + 1e-5
        && dz.abs() * config.voxel_size <= config.step_threshold_m + 1e-5 {
        let lower = |cell: VoxelKey| {
            let mut low = bounds.get(&cell).map_or(cell.2 as f32 * config.voxel_size, |b| b[0]);
            for step in 1..=candidate_step_cells.max(1) + 2 {
                let Some(z) = cell.2.checked_sub(step) else { break; };
                let Some(b) = bounds.get(&(cell.0, cell.1, z)) else { break; };
                if low - b[1] > source_size + 1e-5 { break; }
                low = low.min(b[0]);
            }
            low
        };
        if height(from).min(height(to)) - lower(from).max(lower(to)) >= source_size - 1e-5 {
            return Some(SurfaceTransition::Ramp);
        }
    }
    // 小于半个源体素且实测局部坡度已合格的连续变化不属于离散台阶。
    // 否则墙外或窗口另一端的地形会反向切断当前平缓表面内的邻接。
    if dz.abs() <= quantum * 0.5 + 1e-5
        && dz.abs() <= config.max_slope_deg.to_radians().tan() + 1e-5
    {
        return Some(SurfaceTransition::Ramp);
    }
    let before = side_profile(
        lookup,
        from,
        (-dx, -dy),
        profile_cells,
        candidate_step_cells, bounds, config.voxel_size,
    );
    let after = side_profile(lookup, to, (dx, dy), profile_cells, candidate_step_cells, bounds, config.voxel_size);
    let heights: Vec<_> = before.iter().rev().chain(after.iter()).copied().collect();
    let max_gradient = config.max_slope_deg.to_radians().tan();
    let flat = |side: &[f32]| side.iter().copied().fold(f32::NEG_INFINITY,f32::max)-side.iter().copied().fold(f32::INFINITY,f32::min) <= quantum*0.5+1e-5;
    // 踏面须有完整实测深度；连续地面则按实际可见剖面检查，不能把墙外
    // 缺失的样本反向侵蚀已观测的狭窄通道。
    let has_treads = before.len() > profile_cells as usize && after.len() > profile_cells as usize
        && flat(&before) && flat(&after);
    // 明确的多体素台阶不能借较大的坡度上限绕过台阶高度限制。
    if has_treads {
        return (dz.abs() * config.voxel_size <= config.step_threshold_m + 1e-5).then_some(
            if dz > 0.0 {
                SurfaceTransition::StepUp
            } else {
                SurfaceTransition::StepDown
            },
        );
    }
    let continuous = (0..heights.len()).all(|left| {
        ((left + 1)..heights.len()).all(|right| {
            // 一个体素只补偿高度量化；跨窗口连续陡坡无法累计借用该误差。
            (heights[right] - heights[left]).abs()
                <= max_gradient * (right - left) as f32 + quantum + 1e-5
        })
    });
    if continuous {
        return Some(SurfaceTransition::Ramp);
    }
    if dz.abs() * config.voxel_size <= config.step_threshold_m + 1e-5 && has_treads {
        Some(if dz > 0.0 {
            SurfaceTransition::StepUp
        } else {
            SurfaceTransition::StepDown
        })
    } else {
        None
    }
}

fn rotated_support_offset(quaternion: [f64; 4], distance: f64) -> Result<[f64; 3], SurfaceError> {
    if !distance.is_finite() || distance <= 0.0 || quaternion.iter().any(|value| !value.is_finite())
    {
        return Err(SurfaceError::InvalidPose);
    }
    let scale = quaternion.into_iter().map(f64::abs).fold(0.0, f64::max);
    if scale == 0.0 {
        return Err(SurfaceError::InvalidPose);
    }
    let [x, y, z, w] = quaternion.map(|value| value / scale);
    let norm = (x * x + y * y + z * z + w * w).sqrt();
    let (x, y, z, w) = (x / norm, y / norm, z / norm, w / norm);
    Ok([
        -distance * 2.0 * (x * z + w * y),
        -distance * 2.0 * (y * z - w * x),
        -distance * (1.0 - 2.0 * (x * x + y * y)),
    ])
}

fn validate_boundary(boundary: &[[f64; 2]]) -> Result<(), SurfaceError> {
    if boundary.is_empty() {
        return Ok(());
    }
    if boundary.len() < 3
        || boundary.len() > 4096
        || boundary.iter().flatten().any(|value| !value.is_finite())
    {
        return Err(SurfaceError::InvalidBoundary);
    }
    let area: f64 = (0..boundary.len())
        .map(|index| {
            let a = boundary[index];
            let b = boundary[(index + 1) % boundary.len()];
            a[0] * b[1] - b[0] * a[1]
        })
        .sum();
    if !area.is_finite() || area.abs() <= f64::EPSILON {
        return Err(SurfaceError::InvalidBoundary);
    }
    let count = if boundary.first() == boundary.last() {
        boundary.len() - 1
    } else {
        boundary.len()
    };
    for left in 0..count {
        let a = boundary[left];
        let b = boundary[(left + 1) % count];
        if a == b {
            return Err(SurfaceError::InvalidBoundary);
        }
        for right in (left + 1)..count {
            if right == left + 1 || (left == 0 && right + 1 == count) {
                continue;
            }
            if boundary_edges_intersect(a, b, boundary[right], boundary[(right + 1) % count]) {
                return Err(SurfaceError::InvalidBoundary);
            }
        }
    }
    Ok(())
}

fn boundary_edges_intersect(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> bool {
    let cross = |p: [f64; 2], q: [f64; 2], r: [f64; 2]| {
        (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    };
    if a[0].min(b[0]) > c[0].max(d[0])
        || c[0].min(d[0]) > a[0].max(b[0])
        || a[1].min(b[1]) > c[1].max(d[1])
        || c[1].min(d[1]) > a[1].max(b[1])
    {
        return false;
    }
    let opposite = |left: f64, right: f64| {
        left.abs() <= 1e-12
            || right.abs() <= 1e-12
            || left.is_sign_positive() != right.is_sign_positive()
    };
    opposite(cross(a, b, c), cross(a, b, d)) && opposite(cross(c, d, a), cross(c, d, b))
}

fn segment_inside_boundary(from: [f64; 2], to: [f64; 2], boundary: &[[f64; 2]]) -> bool {
    if boundary.is_empty() {
        return true;
    }
    if !inside_boundary(from, boundary) || !inside_boundary(to, boundary) {
        return false;
    }
    let direction = [to[0] - from[0], to[1] - from[1]];
    let mut intervals = vec![0.0, 1.0];
    for index in 0..boundary.len() {
        let a = boundary[index];
        let b = boundary[(index + 1) % boundary.len()];
        let edge = [b[0] - a[0], b[1] - a[1]];
        let denominator = direction[0] * edge[1] - direction[1] * edge[0];
        if denominator.abs() <= 1e-12 {
            continue;
        }
        let offset = [a[0] - from[0], a[1] - from[1]];
        let along = (offset[0] * edge[1] - offset[1] * edge[0]) / denominator;
        let across = (offset[0] * direction[1] - offset[1] * direction[0]) / denominator;
        if (0.0..=1.0).contains(&along) && (0.0..=1.0).contains(&across) {
            intervals.push(along);
        }
    }
    intervals.sort_by(f64::total_cmp);
    intervals.windows(2).all(|pair| {
        let midpoint = (pair[0] + pair[1]) * 0.5;
        inside_boundary(
            [
                from[0] + direction[0] * midpoint,
                from[1] + direction[1] * midpoint,
            ],
            boundary,
        )
    })
}

fn inside_boundary(point: [f64; 2], boundary: &[[f64; 2]]) -> bool {
    if boundary.is_empty() {
        return true;
    }
    let mut inside = false;
    for index in 0..boundary.len() {
        let a = boundary[index];
        let b = boundary[(index + 1) % boundary.len()];
        let cross = (point[0] - a[0]) * (b[1] - a[1]) - (point[1] - a[1]) * (b[0] - a[0]);
        if cross.abs() <= 1e-9
            && point[0] >= a[0].min(b[0]) - 1e-9
            && point[0] <= a[0].max(b[0]) + 1e-9
            && point[1] >= a[1].min(b[1]) - 1e-9
            && point[1] <= a[1].max(b[1]) + 1e-9
        {
            return true;
        }
        if (a[1] > point[1]) != (b[1] > point[1])
            && point[0] < (b[0] - a[0]) * (point[1] - a[1]) / (b[1] - a[1]) + a[0]
        {
            inside = !inside;
        }
    }
    inside
}

fn plane_from(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> Option<[f64; 3]> {
    let denominator = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]);
    if denominator.abs() <= 1e-9 {
        return None;
    }
    let x = ((b[2] - a[2]) * (c[1] - a[1]) - (c[2] - a[2]) * (b[1] - a[1])) / denominator;
    let y = ((b[0] - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (b[2] - a[2])) / denominator;
    Some([x, y, a[2] - x * a[0] - y * a[1]])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flat_simplification_preserves_integrated_center_preference_between_wall_endpoints() {
        let mut cfg=config();cfg.voxel_size=0.1;cfg.robot_height=0.35;cfg.wall_clearance_m=0.0;
        cfg.wall_buffer_m=0.6;cfg.wall_buffer_weight=3.0;
        let mut points=Vec::new();
        for x in -22..23 {for y in -6..=5 {for z in if y==-6 || y==5 {0..10}else{0..1} {
            points.push(((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,(z as f32+0.5)*0.1));
        }}}
        let map=SurfaceSnapshot::from_points(&points,cfg,0.1).unwrap();
        let start=[-1.5,-0.3,0.1];let goal=[1.5,-0.3,0.1];
        map.validate_segment(start,goal).unwrap();
        let path=map.plan_in_boundary(start,goal,&[]).unwrap();
        assert!(path.iter().any(|p|p[1].abs()<0.2),"原居中路线不能被同峰值贴墙直线覆盖: {path:?}");
        let cost=path.windows(2).map(|p|map.soft_segment_cost(p[0],p[1]).unwrap()).sum::<f64>();
        let direct=map.soft_segment_cost(start,goal).unwrap();
        assert!(cost<direct,"完整软代价应保留居中收益: {cost} >= {direct}");
        map.validate_path(&path).unwrap();
        let boundary=[[-2.0,-0.48],[2.0,-0.48],[2.0,0.48],[-2.0,0.48]];
        let bounded=map.plan_in_boundary(start,goal,&boundary).unwrap();
        assert!(bounded.iter().any(|p|p[1].abs()<0.2));
    }

    #[test]
    fn overlapping_ground_band_keeps_local_connection_without_bridging_a_separated_step() {
        let mut cfg = config();
        cfg.voxel_size = 0.1;
        cfg.step_threshold_m = 0.15;
        cfg.max_slope_deg = 18.0;
        let mut lookup = SurfaceLookup::default();
        let mut bounds = HeightBounds::default();
        for (y,z,low,high) in [(-2,-5,-0.5,-0.4),(-1,-5,-0.5,-0.4),(0,-5,-0.5,-0.4),
            (1,-4,-0.4,-0.35),(2,-3,-0.3,-0.25),(3,-2,-0.2,-0.15)] {
            lookup.insert((0,y),vec![z]);bounds.insert((0,y,z),[low,high]);
        }
        let from=(0,0,-5);let to=(0,1,-4);
        assert_eq!(classify_transition(&lookup,from,to,&cfg,&bounds,0.05,2,2),None,
            "没有共同高度证据不能跨越分离层");
        bounds.insert((0,1,-5),[-0.5,-0.4]);
        assert_eq!(classify_transition(&lookup,from,to,&cfg,&bounds,0.05,2,2),Some(SurfaceTransition::Ramp));
        cfg.step_threshold_m=0.0;
        assert_ne!(classify_transition(&lookup,from,to,&cfg,&bounds,0.05,2,2),Some(SurfaceTransition::Ramp),
            "不得突破明确的零越阶能力");
    }

    const SIZE: f32 = 0.05;

    fn config() -> Config {
        Config {
            world_frame: "map".into(),
            voxel_size: SIZE,
            robot_height: 0.45,
            surface_closing_radius: 0.0,
            node_spacing_m: 0.3,
            wall_clearance_m: 0.1,
            wall_buffer_m: 0.1,
            wall_buffer_weight: 0.0,
            step_threshold_m: 0.2,
            max_slope_deg: 25.0,
            min_step_tread_depth_m: 0.2,
            step_penalty_weight: 1.0,
            goal_tolerance: 0.1,
            viz_publish_hz: 0.0,
        }
    }

    fn points_for_profile(heights: &[i32], width: i32) -> Vec<(f32, f32, f32)> {
        heights
            .iter()
            .enumerate()
            .flat_map(|(ix, &height)| {
                (0..width).map(move |iy| {
                    (
                        (ix as f32 + 0.5) * SIZE,
                        (iy as f32 + 0.5) * SIZE,
                        (height as f32 + 0.5) * SIZE,
                    )
                })
            })
            .collect()
    }

    fn support(x: i32, y: i32, z: i32) -> [f64; 3] {
        let (x, y, z) = surface_point_xyz(x, y, z, SIZE);
        [f64::from(x), f64::from(y), f64::from(z)]
    }

    #[test]
    fn finer_source_retains_step_height_with_coarse_horizontal_cells() {
        let mut cfg=config();cfg.voxel_size=0.1;cfg.min_step_tread_depth_m=0.3;cfg.step_threshold_m=0.16;
        let mut points=Vec::new();
        for x in 0..80 {for y in 0..24 {points.push(((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,if x<40{0.0125}else{0.1625}));}}
        let map=SurfaceSnapshot::from_points(&points,cfg,0.025).unwrap();
        let low=map.cell_position((20,12,0));let high=map.cell_position((60,12,1));
        assert!((low[2]-0.025).abs()<1e-6);assert!((high[2]-0.175).abs()<1e-6);
        assert!((high[2]-low[2]-0.15).abs()<1e-6,"水平粗格不能把15厘米台阶改成10厘米");
        map.plan(low,high).unwrap();map.plan(high,low).unwrap();
        for (_,edges) in map.data.graph.cells.iter() {assert!(edges.iter().all(|edge|!edge.cost.is_finite() || edge.cost>=0.0));}
    }

    #[test]
    fn fine_height_change_inside_same_coarse_voxel_rebuilds_dependent_geometry() {
        let mut cfg=config();cfg.voxel_size=0.1;cfg.min_step_tread_depth_m=0.3;
        let mut points:Vec<_>=(0..80).flat_map(|x|(0..24).map(move|y|((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.0125))).collect();
        let mut builder=SurfaceMapBuilder::new(cfg.clone());let first=builder.build(&points,0.025).unwrap().snapshot;
        first.reachable_frontiers([1.05,1.05,0.025],&[]).unwrap();
        let point=points.iter_mut().find(|p|voxelize(**p,0.1)==(50,12,0)).unwrap();point.2=0.0625;
        let changed=builder.build(&points,0.025).unwrap();
        assert_eq!(changed.stats.kind,SurfaceBuildKind::Incremental);assert_eq!(changed.stats.changed_surface_cells,0);assert!(!changed.stats.global_cost_refresh);assert!(changed.stats.checked_cells>0);
        assert!((changed.snapshot.cell_position((50,12,0))[2]-0.075).abs()<1e-6);
        let full=SurfaceSnapshot::from_points(&points,cfg,0.025).unwrap();
        assert_eq!(changed.snapshot.reachable_frontiers([1.05,1.05,0.025],&[]).unwrap(),full.reachable_frontiers([1.05,1.05,0.025],&[]).unwrap());
        assert!((first.cell_position((50,12,0))[2]-0.025).abs()<1e-6);
    }

    #[test]
    fn real_stairs_with_treads_are_traversable_both_directions() {
        // 六级15厘米楼梯，每级30厘米踏深，两端有平台。
        let heights: Vec<_> = (0..72)
            .map(|x| {
                if x < 12 {
                    0
                } else if x < 48 {
                    ((x - 12) / 6 + 1) * 3
                } else {
                    18
                }
            })
            .collect();
        let map = SurfaceSnapshot::from_points(&points_for_profile(&heights, 20), config(), SIZE)
            .unwrap();
        assert_eq!(
            map.transition((11, 10, 0), (12, 10, 3)),
            Some(SurfaceTransition::StepUp)
        );
        assert_eq!(
            map.transition((12, 10, 3), (11, 10, 0)),
            Some(SurfaceTransition::StepDown)
        );
        let lower = support(6, 10, 0);
        let upper = support(65, 10, 18);
        for (start, goal) in [(lower, upper), (upper, lower)] {
            let path = map.plan(start, goal).expect("合法楼梯须存在连续支撑路径");
            assert_eq!(path.first(), Some(&start));
            assert_eq!(path.last(), Some(&goal));
            map.validate_path(&path).unwrap();
            for pair in path.windows(2) {
                for ratio in [0.2, 0.5, 0.8] {
                    let sampled = map.sample_segment_support(pair[0], pair[1], ratio).unwrap();
                    assert_eq!(sampled.support_xyz[2], sampled.surface_xyz[2]);
                    map.validate_segment(pair[0], sampled.support_xyz).unwrap();
                }
            }
        }
    }

    #[test]
    fn lookahead_samples_only_the_known_prefix_and_never_moves_actual_pose() {
        let map =
            SurfaceSnapshot::from_points(&points_for_profile(&vec![0; 40], 20), config(), SIZE)
                .unwrap();
        let start = support(10, 10, 0);
        let distant_unknown = [1000.0, start[1], start[2]];
        let ratio = 0.4 / (distant_unknown[0] - start[0]);
        let sampled = map
            .sample_segment_support(start, distant_unknown, ratio)
            .unwrap();
        assert!((sampled.support_xyz[0] - start[0] - 0.4).abs() < 1e-9);
        assert!(map
            .sample_segment_support([start[0], start[1], 2.0], distant_unknown, ratio)
            .is_err());
        assert!(map
            .sample_segment_support(start, distant_unknown, 1.0)
            .is_err());
    }

    #[test]
    fn continuous_ramp_passes_but_steep_ramp_is_not_relabelled_as_stairs() {
        let gentle: Vec<_> = (0..80).map(|x| x / 4).collect();
        let map =
            SurfaceSnapshot::from_points(&points_for_profile(&gentle, 20), config(), SIZE).unwrap();
        let path = map
            .plan(support(10, 10, 2), support(70, 10, 17))
            .expect("约14度坡道应通过25度上限");
        map.validate_path(&path).unwrap();
        assert_eq!(
            map.transition((11, 10, 2), (12, 10, 3)),
            Some(SurfaceTransition::Ramp)
        );

        let steep: Vec<_> = (0..80).collect();
        let mut cfg = config();
        cfg.wall_clearance_m = 0.0;
        let map = SurfaceSnapshot::from_points(&points_for_profile(&steep, 20), cfg, SIZE).unwrap();
        assert!(map.plan(support(10, 10, 10), support(70, 10, 70)).is_err());
        assert!(map
            .validate_segment(support(10, 10, 10), support(70, 10, 70))
            .is_err());
        assert!(map.transition((20, 10, 20), (21, 10, 21)).is_none());
    }

    #[test]
    fn excessive_rise_and_shallow_treads_are_rejected() {
        for heights in [
            (0..72)
                .map(|x| if x < 36 { 0 } else { 6 })
                .collect::<Vec<_>>(),
            (0..72)
                .map(|x| {
                    if x < 12 {
                        0
                    } else if x < 36 {
                        ((x - 12) / 2 + 1) * 3
                    } else {
                        36
                    }
                })
                .collect::<Vec<_>>(),
        ] {
            let mut cfg = config();
            cfg.wall_clearance_m = 0.0;
            let map =
                SurfaceSnapshot::from_points(&points_for_profile(&heights, 20), cfg, SIZE).unwrap();
            assert!(map
                .plan(support(6, 10, heights[6]), support(65, 10, heights[65]))
                .is_err());
        }
        let heights: Vec<_> = (0..72).map(|x| if x < 36 { 0 } else { 3 }).collect();
        let mut cfg = config();
        cfg.max_slope_deg = 80.0;
        cfg.step_threshold_m = 0.1;
        cfg.wall_clearance_m = 0.0;
        let map =
            SurfaceSnapshot::from_points(&points_for_profile(&heights, 20), cfg, SIZE).unwrap();
        assert!(
            map.plan(support(6, 10, 0), support(65, 10, 3)).is_err(),
            "有明确踏面的15厘米台阶不能借80度坡度上限绕过10厘米步高限制"
        );
    }

    #[test]
    fn narrow_edges_and_missing_ground_cannot_carry_a_path() {
        let narrow =
            SurfaceSnapshot::from_points(&points_for_profile(&vec![0; 72], 3), config(), SIZE)
                .unwrap();
        assert!(matches!(
            narrow.locate_support(support(20, 1, 0)),
            Err(SurfaceError::InsufficientClearance)
        ));

        let mut points = points_for_profile(&vec![0; 72], 20);
        points.retain(|point| voxelize(*point, SIZE).0 != 36);
        let map = SurfaceSnapshot::from_points(&points, config(), SIZE).unwrap();
        assert!(map.plan(support(10, 10, 0), support(60, 10, 0)).is_err());
        assert!(map
            .validate_segment(support(10, 10, 0), support(60, 10, 0))
            .is_err());
        assert!(map.locate_support(support(36, 10, 0)).is_err());
    }

    #[test]
    fn source_coarsening_and_synthetic_hole_closing_are_rejected() {
        let points = points_for_profile(&vec![0; 20], 20);
        assert!(matches!(
            SurfaceSnapshot::from_points(&points, config(), SIZE * 2.0),
            Err(SurfaceError::InsufficientResolution { .. })
        ));
        let mut cfg = config();
        cfg.min_step_tread_depth_m = SIZE;
        assert!(matches!(
            SurfaceSnapshot::from_points(&points, cfg, SIZE),
            Err(SurfaceError::InsufficientResolution { .. })
        ));
        let mut cfg = config();
        cfg.surface_closing_radius = 0.1;
        assert!(matches!(
            SurfaceSnapshot::from_points(&points, cfg, SIZE),
            Err(SurfaceError::InvalidConfig(_))
        ));
    }

    #[test]
    fn body_support_uses_full_attitude_and_positive_calibrated_distance() {
        let map =
            SurfaceSnapshot::from_points(&points_for_profile(&vec![0; 40], 40), config(), SIZE)
                .unwrap();
        let expected = support(20, 20, 0);
        let pitch = std::f64::consts::FRAC_PI_6;
        let q = [0.0, (pitch * 0.5).sin(), 0.0, (pitch * 0.5).cos()];
        let body = [
            expected[0] + 0.3 * pitch.sin(),
            expected[1],
            expected[2] + 0.3 * pitch.cos(),
        ];
        let location = map.locate_support_from_body(body, q, 0.3).unwrap();
        for (actual, expected) in location.support_xyz.into_iter().zip(expected) {
            assert!((actual - expected).abs() < 1e-12);
        }
        let recovered = map
            .body_position_from_support(expected, q.map(|value| value * 4.0), 0.3)
            .unwrap();
        for (actual, expected) in recovered.into_iter().zip(body) {
            assert!((actual - expected).abs() < 1e-12);
        }
        assert!(map.locate_support_from_body(body, q, -0.3).is_err());
        assert!(map.locate_support_from_body(body, [0.0; 4], 0.3).is_err());
    }

    #[test]
    fn footprint_interior_cannot_cover_an_unobserved_hole() {
        let mut points = points_for_profile(&vec![0; 40], 40);
        let contour = [
            support(8, 8, 0),
            support(30, 8, 0),
            support(30, 30, 0),
            support(8, 30, 0),
        ];
        let full = SurfaceSnapshot::from_points(&points, config(), SIZE).unwrap();
        full.validate_contour(&contour).unwrap();
        points.retain(|point| voxelize(*point, SIZE) != (20, 20, 0));
        let hole = SurfaceSnapshot::from_points(&points, config(), SIZE).unwrap();
        assert!(hole.validate_contour(&contour).is_err());
    }

    #[test]
    fn regional_surface_costs_follow_detours_boundaries_and_isolate_overlapping_layers() {
        let mut points = points_for_profile(&vec![0; 60], 60);
        points.retain(|point| {
            let cell = voxelize(*point, SIZE);
            !(28..=32).contains(&cell.0) || cell.1 >= 48
        });
        points.extend(points_for_profile(&vec![60; 60], 60));
        let map = SurfaceSnapshot::from_points(&points, config(), SIZE).unwrap();
        let start = support(20, 15, 0);
        let goal = support(40, 15, 0);
        let costs = map.frontier_travel_costs(start, &[goal, support(20, 15, 60)], &[]).unwrap();
        assert!(costs[0] > 3.5, "一米直线必须绕过未知支撑缺口");
        assert!(costs[1].is_infinite(), "上下叠层没有连接时不能混用代价");
        let boundary = [[0.0, 0.0], [3.0, 0.0], [3.0, 2.0], [0.0, 2.0]];
        assert!(map.frontier_travel_costs(start, &[goal], &boundary).unwrap()[0].is_infinite(), "边界切断开口后不能绕出边界");
        let frontiers = map.reachable_frontiers(start, &[]).unwrap();
        let targets: Vec<_> = frontiers.iter().map(|candidate| candidate.support_xyz).collect();
        let costs = map.frontier_travel_costs(start, &targets, &[]).unwrap();
        for (candidate, cost) in frontiers.iter().zip(costs) {
            assert!((candidate.travel_cost - cost).abs() < 1e-9, "初筛与区域间成本必须一致");
        }
    }

    #[test]
    fn frontiers_stay_in_the_reachable_layer_and_inside_requested_boundary() {
        let mut points = points_for_profile(&vec![0; 20], 20);
        points.extend(points_for_profile(&vec![30; 20], 20));
        let map = SurfaceSnapshot::from_points(&points, config(), SIZE).unwrap();
        let frontiers = map.reachable_frontiers(support(10, 10, 0), &[]).unwrap();
        assert!(!frontiers.is_empty());
        for frontier in &frontiers {
            assert_eq!(frontier.cell.2, 0, "无楼层连接时不能发布上层前沿");
            map.plan(support(10, 10, 0), frontier.support_xyz).unwrap();
            assert!(frontier.gain > 0.0);
        }
        let boundary = [[0.2, 0.2], [0.8, 0.2], [0.8, 0.8], [0.2, 0.8]];
        assert!(map
            .reachable_frontiers(support(10, 10, 0), &boundary)
            .unwrap()
            .is_empty());
        assert!(map.plan(support(10, 10, 0), support(10, 10, 30)).is_err());
    }

    #[test]
    fn bounded_flat_route_removes_grid_turns_and_keeps_measured_endpoints() {
        let map = SurfaceSnapshot::from_points(&points_for_profile(&vec![0; 40], 40), config(), SIZE).unwrap();
        let boundary = [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]];
        let start = [0.31, 0.37, 0.0];
        let goal = [1.61, 1.47, 0.0];
        let path = map.plan_in_boundary(start, goal, &boundary).unwrap();
        assert_eq!(path, vec![start, goal]);
        map.validate_path(&path).unwrap();
    }

    #[test]
    fn bounded_route_simplification_preserves_holes_and_stair_heights() {
        let mut points = points_for_profile(&vec![0; 40], 40);
        points.retain(|point| {
            let cell = voxelize(*point, SIZE);
            !(18..=22).contains(&cell.0) || !(18..=22).contains(&cell.1)
        });
        let map = SurfaceSnapshot::from_points(&points, config(), SIZE).unwrap();
        let boundary = [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]];
        let start = support(6, 20, 0);
        let goal = support(34, 20, 0);
        assert!(map.validate_segment(start, goal).is_err());
        let path = map.plan_in_boundary(start, goal, &boundary).unwrap();
        assert!(path.len() > 2);
        map.validate_path(&path).unwrap();

        let heights: Vec<_> = (0..72).map(|x| {
            if x < 12 { 0 } else if x < 48 { ((x - 12) / 6 + 1) * 3 } else { 18 }
        }).collect();
        let stairs = SurfaceSnapshot::from_points(&points_for_profile(&heights, 20), config(), SIZE).unwrap();
        let boundary = [[0.0, 0.0], [3.6, 0.0], [3.6, 1.0], [0.0, 1.0]];
        for (start, goal) in [(support(6, 10, 0), support(65, 10, 18)), (support(65, 10, 18), support(6, 10, 0))] {
            let path = stairs.plan_in_boundary(start, goal, &boundary).unwrap();
            stairs.validate_path(&path).unwrap();
            let levels: std::collections::BTreeSet<_> = path.iter().map(|point| stairs.locate_support(*point).unwrap().cell.2).collect();
            assert_eq!(levels, (0..=6).map(|level| level * 3).collect());
        }
    }

    #[test]
    fn concave_boundary_is_respected_between_centers_and_invalid_polygons_fail() {
        let map =
            SurfaceSnapshot::from_points(&points_for_profile(&vec![0; 40], 40), config(), SIZE)
                .unwrap();
        let boundary = [
            [0.1, 0.1],
            [1.9, 0.1],
            [1.9, 1.9],
            [1.1, 1.9],
            [1.1, 0.8],
            [0.9, 0.8],
            [0.9, 1.9],
            [0.1, 1.9],
        ];
        let start = support(10, 30, 0);
        let goal = support(30, 30, 0);
        let path = map.plan_in_boundary(start, goal, &boundary).unwrap();
        assert!(path.iter().any(|point| point[1] < 0.8));
        assert!(path.windows(2).all(|pair| segment_inside_boundary(
            [pair[0][0], pair[0][1]],
            [pair[1][0], pair[1][1]],
            &boundary
        )));
        let narrow_notch = [
            [0.0, 0.0],
            [2.0, 0.0],
            [2.0, 2.0],
            [1.01, 2.0],
            [1.01, 0.5],
            [0.99, 0.5],
            [0.99, 2.0],
            [0.0, 2.0],
        ];
        assert!(!segment_inside_boundary(
            [0.975, 1.0],
            [1.025, 1.0],
            &narrow_notch
        ));
        let crossing = [[0.0, 0.0], [2.0, 2.0], [2.0, 0.0], [0.0, 1.0]];
        assert!(map.reachable_frontiers(start, &crossing).is_err());
    }

    fn two_floor_virtual_fixture() -> SurfaceSnapshot {
        let mut points = points_for_profile(&vec![0; 100], 40);
        points.extend(points_for_profile(&vec![60; 100], 40));
        SurfaceSnapshot::from_points(&points, config(), SIZE).unwrap()
    }

    #[test]
    fn fine_virtual_line_blocks_all_floors_and_replacement_restores_physical_map() {
        let physical = two_floor_virtual_fixture();
        let (width, height) = (250_u32, 100_u32);
        let mut mask = vec![false; (width * height) as usize];
        // 2厘米细线处于2.40..2.42米，未命中5厘米MLS格的中心2.425米。
        for y in 0..height {
            mask[(y * width + 120) as usize] = true;
        }
        let blocked = physical
            .with_virtual_obstacles([0.0, 0.0], 0.02, width, height, &mask)
            .unwrap();
        for z in [0, 60] {
            let start = support(10, 20, z);
            let goal = support(80, 20, z);
            physical.plan(start, goal).unwrap();
            assert!(blocked.plan(start, goal).is_err());
            assert!(blocked.validate_segment(start, goal).is_err());
            assert!(matches!(
                blocked.locate_support(support(48, 20, z)),
                Err(SurfaceError::VirtualObstacle)
            ));
            assert!(
                blocked.reachable_frontiers(start, &[]).unwrap().is_empty(),
                "禁区与mask范围外不能当作未知增益"
            );
        }
        let restored = blocked
            .with_virtual_obstacles([0.0, 0.0], 0.02, width, height, &vec![false; mask.len()])
            .unwrap();
        for z in [0, 60] {
            restored
                .plan(support(10, 20, z), support(80, 20, z))
                .unwrap();
        }
    }

    #[test]
    fn virtual_obstacles_preserve_detours_and_robot_clearance_without_double_inflation() {
        let physical = two_floor_virtual_fixture();
        let (width, height) = (100_u32, 40_u32);
        let mut mask = vec![false; (width * height) as usize];
        for y in 0..24 {
            mask[(y * width + 48) as usize] = true;
        }
        let constrained = physical
            .with_virtual_obstacles([0.0, 0.0], f64::from(SIZE), width, height, &mask)
            .unwrap();
        for z in [0, 60] {
            let path = constrained
                .plan(support(10, 12, z), support(80, 12, z))
                .unwrap();
            constrained.validate_path(&path).unwrap();
            assert!(
                path.iter().any(|point| point[1] > 1.3),
                "应沿虚拟线端部保留足迹余量绕行"
            );
            assert!(constrained.locate_support(support(46, 12, z)).is_err());
            constrained.locate_support(support(44, 12, z)).unwrap();
        }
    }

    #[test]
    fn virtual_extent_and_all_blocked_masks_fail_closed_but_remain_valid_inputs() {
        let physical = two_floor_virtual_fixture();
        let limited = physical
            .with_virtual_obstacles([1.0, 0.4], 0.1, 20, 12, &vec![false; 240])
            .unwrap();
        for z in [0, 60] {
            assert!(limited.locate_support(support(10, 20, z)).is_err());
            assert!(
                limited.locate_support(support(20, 20, z)).is_err(),
                "范围边缘也须留出机器人半宽"
            );
            limited.locate_support(support(24, 20, z)).unwrap();
        }
        let blocked = physical
            .with_virtual_obstacles([0.0, 0.0], 0.05, 100, 40, &vec![true; 4000])
            .unwrap();
        for z in [0, 60] {
            assert!(blocked.locate_support(support(40, 20, z)).is_err());
            assert!(blocked
                .plan(support(10, 20, z), support(80, 20, z))
                .is_err());
            assert!(blocked
                .reachable_frontiers(support(40, 20, z), &[])
                .is_err());
        }
        assert!(physical
            .with_virtual_obstacles([f64::NAN, 0.0], 0.1, 1, 1, &[false])
            .is_err());
        assert!(physical
            .with_virtual_obstacles([0.0, 0.0], 0.0, 1, 1, &[false])
            .is_err());
        assert!(physical
            .with_virtual_obstacles([0.0, 0.0], 0.1, 2, 1, &[false])
            .is_err());
    }
}
