//! Voxel grid 与 `_voxel_index` 复刻实现；以及 `RollingTerrainGrid` 滚动逻辑。

use crate::config::TerrainMapExtConfig;
use crate::{NavError, Point3, PointCloud, Pose3, TimedPointCloud, Timestamp};

/// Terrain voxel grid 边长（与 C++ 原 `TERRAIN_VOXEL_WIDTH` 对齐）。
pub const TERRAIN_VOXEL_WIDTH: i64 = 41;
/// Terrain voxel grid 半宽（(WIDTH-1)/2 = 20）。
pub const TERRAIN_VOXEL_HALF_WIDTH: i64 = (TERRAIN_VOXEL_WIDTH - 1) / 2;
/// Terrain voxel cell 总数。
pub const TERRAIN_VOXEL_NUM: usize = (TERRAIN_VOXEL_WIDTH * TERRAIN_VOXEL_WIDTH) as usize;

/// Planar voxel grid 边长。
pub const PLANAR_VOXEL_WIDTH: i64 = 101;
/// Planar voxel grid 半宽（(WIDTH-1)/2 = 50）。
pub const PLANAR_VOXEL_HALF_WIDTH: i64 = (PLANAR_VOXEL_WIDTH - 1) / 2;
/// Planar voxel cell 总数。
pub const PLANAR_VOXEL_NUM: usize = (PLANAR_VOXEL_WIDTH * PLANAR_VOXEL_WIDTH) as usize;

/// 单个 voxel 内存储的点。`[x, y, z, time_offset]`。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VoxelPoint {
    /// x 坐标
    pub x: f64,
    /// y 坐标
    pub y: f64,
    /// z 坐标
    pub z: f64,
    /// time_offset = laser_cloud_time - system_init_time
    pub time_offset: f64,
}

/// 复刻 C++/Python 的 `_voxel_index(x, y, voxel_size, half_width)`。
///
/// 算法（严格对齐 Python `int(...)` 截断行为）：
/// 1. `offset = x - y + voxel_size / 2`
/// 2. `index = int(offset / voxel_size) + half_width`（Rust 用 `as i64` 截断，等价 Python `int`）
/// 3. 若 `offset < 0` 再减 1
///
/// # Examples
///
/// ```
/// use nuwax_nav_terrain_map_ext::voxel_index;
/// // x 恰好在 cell 边界上：offset=0.5 → index=0+20=20；offset>0 不修正
/// let i = voxel_index(0.0, 0.0, 1.0, 20);
/// assert_eq!(i, 20);
/// // 负坐标 offset=-0.1：int(-0.1)=0 → index=0+20=20；offset<0 减 1 → 19
/// let i = voxel_index(-0.6, 0.0, 1.0, 20);
/// assert_eq!(i, 19);
/// ```
#[must_use]
pub fn voxel_index(x: f64, y_ref: f64, voxel_size: f64, half_width: i64) -> i64 {
    if voxel_size <= 0.0 {
        return i64::MIN;
    }
    let offset = x - y_ref + voxel_size / 2.0;
    // Rust `as i64` 对正/负浮点都是向 0 截断，等价 Python `int(...)`。
    let mut index = (offset / voxel_size) as i64 + half_width;
    if offset < 0.0 {
        index -= 1;
    }
    index
}

/// 滚动 terrain voxel grid：每 cell 持有 `Vec<VoxelPoint>` + update_num + update_time。
#[derive(Debug, Clone)]
pub struct RollingTerrainGrid {
    /// 每 cell 点列表
    cloud: Vec<Vec<VoxelPoint>>,
    /// 每 cell 点累计更新数（**不随 grid shift 迁移**）
    update_num: Vec<usize>,
    /// 每 cell 上次更新时间（**不随 grid shift 迁移**）
    update_time: Vec<f64>,
    /// x 方向 shift 步数
    shift_x: i64,
    /// y 方向 shift 步数
    shift_y: i64,
}

impl Default for RollingTerrainGrid {
    fn default() -> Self {
        Self::new()
    }
}

impl RollingTerrainGrid {
    /// 构造空的 grid。
    #[must_use]
    pub fn new() -> Self {
        Self {
            cloud: (0..TERRAIN_VOXEL_NUM).map(|_| Vec::new()).collect(),
            update_num: vec![0; TERRAIN_VOXEL_NUM],
            update_time: vec![0.0; TERRAIN_VOXEL_NUM],
            shift_x: 0,
            shift_y: 0,
        }
    }

    /// x shift 步数（用于调试）。
    #[must_use]
    pub fn shift_x(&self) -> i64 {
        self.shift_x
    }

    /// y shift 步数。
    #[must_use]
    pub fn shift_y(&self) -> i64 {
        self.shift_y
    }

    /// 当前 grid 内全部 cell 数。
    #[must_use]
    pub fn cell_count(&self) -> usize {
        TERRAIN_VOXEL_NUM
    }

    /// 访问 cell（只读）。
    #[must_use]
    pub fn cell(&self, idx: usize) -> &[VoxelPoint] {
        &self.cloud[idx]
    }

    /// 访问 cell 的 update_num（只读；测试可验证不随 shift 迁移）。
    #[must_use]
    pub fn cell_update_num(&self, idx: usize) -> usize {
        self.update_num[idx]
    }

    /// 访问 cell 的 update_time（只读）。
    #[must_use]
    pub fn cell_update_time(&self, idx: usize) -> f64 {
        self.update_time[idx]
    }

    /// 推入一个点到指定 cell。
    pub fn push(&mut self, idx: usize, point: VoxelPoint) {
        self.cloud[idx].push(point);
        self.update_num[idx] = self.update_num[idx].saturating_add(1);
    }

    /// 替换指定 cell 的全部点（不清空 update_num/update_time）。
    pub fn replace_cell(&mut self, idx: usize, new_points: Vec<VoxelPoint>) {
        self.cloud[idx] = new_points;
        self.update_num[idx] = 0;
        self.update_time[idx] = -1.0;
    }

    /// 清空指定 cell（点列表、update_num、update_time）。
    pub fn clear_cell(&mut self, idx: usize) {
        self.cloud[idx].clear();
        self.update_num[idx] = 0;
        self.update_time[idx] = 0.0;
    }

    /// 仅设置指定 cell 的 update_time（crate-internal helper，给 `decay.rs` 用）。
    pub(crate) fn set_update_time(&mut self, idx: usize, t: f64) {
        self.update_time[idx] = t;
    }

    /// 按 vehicle 当前位置滚动 grid；窗口内点云按方向平移复用，
    /// 被移出窗口的新边界 voxel 清空。update_num/update_time **不**随点云迁移。
    pub fn roll(&mut self, vehicle_x: f64, vehicle_y: f64, terrain_voxel_size: f64) {
        let mut center_x = terrain_voxel_size * self.shift_x as f64;
        // -X 方向：vehicle 太靠左 → 高 index 向低 index 复制，清空 column 0
        while vehicle_x - center_x < -terrain_voxel_size {
            for ix in (1..TERRAIN_VOXEL_WIDTH).rev() {
                for iy in 0..TERRAIN_VOXEL_WIDTH {
                    let dst = (ix * TERRAIN_VOXEL_WIDTH + iy) as usize;
                    let src = ((ix - 1) * TERRAIN_VOXEL_WIDTH + iy) as usize;
                    self.cloud[dst] = std::mem::take(&mut self.cloud[src]);
                }
            }
            // 清空 column 0
            for iy in 0..TERRAIN_VOXEL_WIDTH {
                self.cloud[iy as usize].clear();
            }
            self.shift_x -= 1;
            center_x = terrain_voxel_size * self.shift_x as f64;
        }
        // +X 方向：vehicle 太靠右 → 低 index 向高 index 复制，清空最后一列
        while vehicle_x - center_x > terrain_voxel_size {
            for ix in 0..(TERRAIN_VOXEL_WIDTH - 1) {
                for iy in 0..TERRAIN_VOXEL_WIDTH {
                    let dst = (ix * TERRAIN_VOXEL_WIDTH + iy) as usize;
                    let src = ((ix + 1) * TERRAIN_VOXEL_WIDTH + iy) as usize;
                    self.cloud[dst] = std::mem::take(&mut self.cloud[src]);
                }
            }
            // 清空最后一列 (index = WIDTH-1)
            let last_col = TERRAIN_VOXEL_WIDTH - 1;
            for iy in 0..TERRAIN_VOXEL_WIDTH {
                let idx = (last_col * TERRAIN_VOXEL_WIDTH + iy) as usize;
                self.cloud[idx].clear();
            }
            self.shift_x += 1;
            center_x = terrain_voxel_size * self.shift_x as f64;
        }
        let mut center_y = terrain_voxel_size * self.shift_y as f64;
        // -Y 方向：vehicle 太靠下 → 高 iy 向低 iy 复制，清空 row 0
        while vehicle_y - center_y < -terrain_voxel_size {
            for iy in (1..TERRAIN_VOXEL_WIDTH).rev() {
                for ix in 0..TERRAIN_VOXEL_WIDTH {
                    let dst = (ix * TERRAIN_VOXEL_WIDTH + iy) as usize;
                    let src = (ix * TERRAIN_VOXEL_WIDTH + (iy - 1)) as usize;
                    self.cloud[dst] = std::mem::take(&mut self.cloud[src]);
                }
            }
            // 清空 row 0
            for ix in 0..TERRAIN_VOXEL_WIDTH {
                let idx = (ix * TERRAIN_VOXEL_WIDTH) as usize;
                self.cloud[idx].clear();
            }
            self.shift_y -= 1;
            center_y = terrain_voxel_size * self.shift_y as f64;
        }
        // +Y 方向：vehicle 太靠上 → 低 iy 向高 iy 复制，清空最后一行
        while vehicle_y - center_y > terrain_voxel_size {
            for iy in 0..(TERRAIN_VOXEL_WIDTH - 1) {
                for ix in 0..TERRAIN_VOXEL_WIDTH {
                    let dst = (ix * TERRAIN_VOXEL_WIDTH + iy) as usize;
                    let src = (ix * TERRAIN_VOXEL_WIDTH + (iy + 1)) as usize;
                    self.cloud[dst] = std::mem::take(&mut self.cloud[src]);
                }
            }
            // 清空最后一行 (iy = WIDTH-1)
            let last_row = TERRAIN_VOXEL_WIDTH - 1;
            for ix in 0..TERRAIN_VOXEL_WIDTH {
                let idx = (ix * TERRAIN_VOXEL_WIDTH + last_row) as usize;
                self.cloud[idx].clear();
            }
            self.shift_y += 1;
            center_y = terrain_voxel_size * self.shift_y as f64;
        }
    }
}

/// 单次 voxel 下采样：对每个 3D bin 只保留 timestamp 最大的点。
///
/// `voxel_size` 单位米。
#[must_use]
pub fn voxel_downsample(points: &[VoxelPoint], voxel_size: f64) -> Vec<VoxelPoint> {
    if voxel_size <= 0.0 || points.is_empty() {
        return points.to_vec();
    }
    let inv = 1.0 / voxel_size;
    // 使用 (i64,i64,i64) 键加速
    let mut seen: std::collections::HashMap<(i64, i64, i64), VoxelPoint> =
        std::collections::HashMap::with_capacity(points.len());
    for &p in points {
        let key = (
            (p.x * inv).floor() as i64,
            (p.y * inv).floor() as i64,
            (p.z * inv).floor() as i64,
        );
        match seen.get(&key) {
            Some(existing) if existing.time_offset >= p.time_offset => {}
            _ => {
                seen.insert(key, p);
            }
        }
    }
    let mut out: Vec<VoxelPoint> = seen.into_values().collect();
    // 保留输入顺序的稳定性：按 z 升序、再 y、再 x 排序（与 Python dict iteration 一致性近似）
    out.sort_by(|a, b| {
        a.x.partial_cmp(&b.x)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a.y.partial_cmp(&b.y).unwrap_or(std::cmp::Ordering::Equal))
            .then(a.z.partial_cmp(&b.z).unwrap_or(std::cmp::Ordering::Equal))
    });
    out
}

/// 累积一帧 cropped scan 到 rolling grid；返回被 push 的 cell 总数。
pub fn ingest_into_grid(
    grid: &mut RollingTerrainGrid,
    cropped: &[VoxelPoint],
    vehicle_x: f64,
    vehicle_y: f64,
    terrain_voxel_size: f64,
) -> usize {
    let mut pushed = 0;
    for p in cropped {
        let ix = voxel_index(p.x, vehicle_x, terrain_voxel_size, TERRAIN_VOXEL_HALF_WIDTH);
        let iy = voxel_index(p.y, vehicle_y, terrain_voxel_size, TERRAIN_VOXEL_HALF_WIDTH);
        if (0..TERRAIN_VOXEL_WIDTH).contains(&ix) && (0..TERRAIN_VOXEL_WIDTH).contains(&iy) {
            let idx = (ix * TERRAIN_VOXEL_WIDTH + iy) as usize;
            grid.push(idx, *p);
            pushed += 1;
        }
    }
    pushed
}

/// `TerrainMapExtProcessor`：高层 API；调用 crop/decay/ground/connectivity/merge 模块。
#[derive(Debug)]
pub struct TerrainMapExtProcessor {
    /// 配置
    pub config: TerrainMapExtConfig,
    /// rolling grid
    grid: RollingTerrainGrid,
    /// vehicle 当前位姿
    odom: Option<Pose3>,
    /// 最近一次 local_terrain（来自 terrain_map topic）
    local_terrain: Vec<Point3>,
    /// system_init_time
    system_init_time: Option<f64>,
    /// 最近 laser_cloud_time
    last_laser_time: Option<f64>,
    /// 下一次是否 clearing
    clearing: bool,
}

impl TerrainMapExtProcessor {
    /// 构造。
    #[must_use]
    pub fn new(config: TerrainMapExtConfig) -> Self {
        Self {
            config,
            grid: RollingTerrainGrid::new(),
            odom: None,
            local_terrain: Vec::new(),
            system_init_time: None,
            last_laser_time: None,
            clearing: false,
        }
    }

    /// 触发一次 manual clearing（会清掉 `clearing_distance` 内的点）。
    pub fn request_clearing(&mut self) {
        self.clearing = true;
    }

    /// 更新里程计。
    pub fn update_odometry(&mut self, pose: Pose3) {
        self.odom = Some(pose);
    }

    /// 接收 fresh local terrain（来自 `terrain_map` topic）。
    pub fn update_local_terrain(&mut self, cloud: &[Point3]) {
        self.local_terrain = cloud.to_vec();
    }

    /// 接收 `registered_scan` 输入并裁剪；非空时记录 system_init_time。
    /// # Errors
    /// 当 `points.len() != times.len()` 时返回 `InvalidInput`。
    pub fn ingest_registered_scan(&mut self, cloud: TimedPointCloud<'_>) -> Result<(), NavError> {
        if cloud.points.len() != cloud.times.len() {
            return Err(NavError::InvalidInput {
                field: "TimedPointCloud",
                reason: "points.len() != times.len()".into(),
            });
        }
        let pose = self.odom.ok_or(NavError::InvalidInput {
            field: "odometry",
            reason: "no odometry received yet".into(),
        })?;
        let laser_time = cloud.times.last().copied().unwrap_or(0.0);
        if self.system_init_time.is_none() {
            self.system_init_time = Some(laser_time);
        }
        self.last_laser_time = Some(laser_time);
        // 把扫描时间也缓存；不立刻处理（与 Python 实现一致：scan 进入后 process_tick 才计算）
        // 为了保持 ingest API 纯净，裁剪并写入 grid 在 process_tick 完成。
        let cropped = crate::crop::crop_scan(cloud.points, &pose, &self.config);
        // 滚动 grid（vehicle 位置变化时平移旧点）
        let vx = pose.position.x;
        let vy = pose.position.y;
        self.grid.roll(vx, vy, self.config.terrain_voxel_size);
        let time_offset = laser_time - self.system_init_time.unwrap_or(0.0);
        let voxel_points: Vec<VoxelPoint> = cropped
            .into_iter()
            .zip(cloud.points.iter())
            .filter_map(|(kept, p)| {
                if kept {
                    Some(VoxelPoint {
                        x: p.x,
                        y: p.y,
                        z: p.z,
                        time_offset,
                    })
                } else {
                    None
                }
            })
            .collect();
        ingest_into_grid(
            &mut self.grid,
            &voxel_points,
            vx,
            vy,
            self.config.terrain_voxel_size,
        );
        Ok(())
    }

    /// 跑一次处理 tick：触发 voxel 下采样 + 衰减 + ground + connectivity + 输出。
    /// # Errors
    /// 在没收到 odometry 时返回 `InvalidInput`。
    pub fn process_tick(&mut self, now: Timestamp) -> Result<Option<PointCloud>, NavError> {
        let pose = self.odom.ok_or(NavError::InvalidInput {
            field: "odometry",
            reason: "process_tick without odometry".into(),
        })?;
        let vx = pose.position.x;
        let vy = pose.position.y;
        let vz = pose.position.z;
        let laser_time = self.last_laser_time.ok_or(NavError::InvalidInput {
            field: "ingest_registered_scan",
            reason: "no scan received yet".into(),
        })?;
        let time_offset = laser_time - self.system_init_time.unwrap_or(0.0);

        // 1. voxel 下采样/过滤
        crate::decay::decay_and_downsampling(
            &mut self.grid,
            &self.config,
            time_offset,
            vx,
            vy,
            vz,
            self.clearing,
        );
        self.clearing = false;

        // 2. 取中央 21x21 cell 范围
        let lo = TERRAIN_VOXEL_HALF_WIDTH - 10;
        let hi = TERRAIN_VOXEL_HALF_WIDTH + 11;
        let mut terrain_cloud: Vec<VoxelPoint> = Vec::new();
        for ix in lo..hi {
            for iy in lo..hi {
                let idx = (ix * TERRAIN_VOXEL_WIDTH + iy) as usize;
                terrain_cloud.extend_from_slice(self.grid.cell(idx));
            }
        }

        // 3. planar ground estimation
        let (planar_elev, planar_conn) =
            crate::ground::estimate_ground(&terrain_cloud, vx, vy, vz, &self.config);

        // 4. 输出过滤
        let out = crate::merge::build_output(
            &terrain_cloud,
            &planar_elev,
            &planar_conn,
            &self.local_terrain,
            &self.config,
            vx,
            vy,
            vz,
            now,
        );
        Ok(out)
    }

    /// 测试 helper：直接访问 rolling grid。
    #[must_use]
    pub fn grid(&self) -> &RollingTerrainGrid {
        &self.grid
    }

    /// 测试 helper：可变访问 rolling grid。
    pub fn grid_mut(&mut self) -> &mut RollingTerrainGrid {
        &mut self.grid
    }

    /// 测试 helper：system_init_time。
    #[must_use]
    pub fn system_init_time(&self) -> Option<f64> {
        self.system_init_time
    }
}
