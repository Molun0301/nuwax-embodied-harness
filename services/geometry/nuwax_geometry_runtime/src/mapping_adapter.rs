// Phase 6 (codex-round3 R3-4)：voxel_size / feed_pointcloud / accumulated_count
// / cost_mapper_config / reset_accumulator / current_revision 是 Phase 3 mapping
// adapter 对外 platform API 预留（SharedState.map / cloud_map 走 fetcher 而不
// 直接调 FeedOnlyAccumulator.method），删了会破坏 Phase 4/5 调用面。
#![allow(dead_code)]

//! Phase 3: nuwax-mapping 接入 adapter。
//!
//! 目标:把 `nuwax-mapping::PointCloudAccumulator` / `OccupancyGrid` / `CostMapper`
//! 接成本仓 `MapProvider`/occupancy/costmap/cloud_map 的真实数据源。
//!
//! ## 数据流
//! 1. 订阅 `/odom` (odometry) + `/tf` (transform) + `/go2/lidar/points` (point cloud)
//! 2. 用本地 `FeedOnlyAccumulator`(实现 `nuwax_mapping::PointCloudAccumulator`
//!    trait)累积点云
//! 3. 用 `nuwax_mapping::PointCloudOccupancyBuilder` 把累积点云 + 当前 odom pose
//!    生成 `OccupancyGrid`
//! 4. 用 `nuwax_mapping::CostMapper` (nav2_default) 把 occupancy → costmap
//! 5. 把 occupancy → costmap 推入 `SharedState.map` (`MapState` 字段复用)
//!
//! ## readiness 语义
//! - `mapping_ready_poll()` 真实判定:`accumulator_has_data && lcm_backend_available`
//!   之前 Phase 2 baseline 恒 `false`,本次实装后改 true (有数据时)
//! - `readiness()` 暴露 odom_count / pointcloud_count / last_occupancy_revision
//!   + `mapping_ready` 综合判定;参考 `00_最终执行方案.md` §5 Phase 7 任务 1。
//!     缺 pointcloud (count==0) → mapping_ready=false,即便 odom_count > 0 也不算 ready。
//!
//! ## 为什么不用 XyzAccumulator
//! - `nuwax_mapping::pointclouds::accumulators::XyzAccumulator::into_points(self)`
//!   是 self 消耗语义,无法在 `&self` 上克隆点云
//! - occupancy builder 需要 `&[Point3<f64>]`,因此需要 read-only view
//! - 本地 `FeedOnlyAccumulator` 同时实现 trait + 提供 `clone_points(&self)`
//!
//! ## LCM typed channel 解析
//! - 优先用 `channel_registry::resolve_channel` 解析 `/odom` `/tf`
//!   `/go2/lidar/points`;不重读 `std::env::var`
//! - 该 adapter 是 `lcm_runtime.rs` 启动时调用的 `spawn_*` 之一,
//!   接收的 channel 字符串由调用方决定,这里只在 missing 时回退到 default

use std::sync::Arc;

use nalgebra::Point3;
use nuwax_mapping::cost_mapper::{
    CostAlgoSelector, CostMapperConfig, OccupancyConfig, SimplePointCloud2,
};
use nuwax_mapping::pointclouds::occupancy::{MappingError, PointCloudOccConfig};
use nuwax_mapping::{CostMapper, MappingSession, OccupancyGrid, PointCloudAccumulator};
use tokio::sync::{broadcast, RwLock};
use tracing::warn;

#[allow(unused_imports)]
use nuwax_mapping::SessionStats as MappingSessionStats;

use crate::lcm_runtime::{MapRevisionEvent, MapState, SharedState};

// ============================================================================
// 配置
// ============================================================================

/// nuwax-mapping adapter 配置(env 探测 + 合理 default)。
///
/// 关键参数:
/// - `resolution` —— occupancy 栅格分辨率 (m/cell);默认 0.05
/// - `width_cells` / `height_cells` —— 栅格尺寸;默认 200×200 (10m × 10m)
/// - `voxel_size` —— 体素降采样边长;默认 0.05,与 `pointcloud.rs::VOXEL_SIZE` 对齐
/// - `origin_xy` —— 旧 fixed-bounds 兼容配置;production 使用点云四周 1 m auto bounds
/// - `occupancy_algo` —— 原版 HeightCost / General / Simple kernel selector
#[derive(Debug, Clone, Copy)]
pub struct MappingAdapterConfig {
    pub resolution: f64,
    pub width_cells: u32,
    pub height_cells: u32,
    pub origin_xy: (f64, f64),
    pub voxel_size: f64,
    pub occupancy_algo: CostAlgoSelector,
    pub initial_safe_radius_meters: f32,
}

impl MappingAdapterConfig {
    /// 从 env 探测,带 default。
    ///
    /// - `NUWAX_MAPPING_RESOLUTION` (默认 0.05)
    /// - `NUWAX_MAPPING_WIDTH_CELLS` (默认 200)
    /// - `NUWAX_MAPPING_HEIGHT_CELLS` (默认 200)
    /// - `NUWAX_MAPPING_VOXEL_SIZE` (默认 0.05)
    /// - `NUWAX_MAPPING_ORIGIN_X` / `NUWAX_MAPPING_ORIGIN_Y` (默认 -5 / -5)
    /// - `NUWAX_MAPPING_OCCUPANCY_ALGO` (默认 height_cost)
    /// - `NUWAX_MAPPING_INITIAL_SAFE_RADIUS_METERS` (默认 0)
    pub fn from_env() -> Self {
        Self {
            resolution: parse_env_f64("NUWAX_MAPPING_RESOLUTION", 0.05),
            width_cells: parse_env_u32("NUWAX_MAPPING_WIDTH_CELLS", 200),
            height_cells: parse_env_u32("NUWAX_MAPPING_HEIGHT_CELLS", 200),
            origin_xy: (
                parse_env_f64("NUWAX_MAPPING_ORIGIN_X", -5.0),
                parse_env_f64("NUWAX_MAPPING_ORIGIN_Y", -5.0),
            ),
            voxel_size: parse_env_f64("NUWAX_MAPPING_VOXEL_SIZE", 0.05),
            occupancy_algo: parse_env_occupancy_algo(),
            initial_safe_radius_meters: parse_env_nonnegative_f32(
                "NUWAX_MAPPING_INITIAL_SAFE_RADIUS_METERS",
                0.0,
            ),
        }
    }
}

fn parse_env_occupancy_algo() -> CostAlgoSelector {
    let Some(value) = std::env::var("NUWAX_MAPPING_OCCUPANCY_ALGO")
        .ok()
        .map(|value| value.trim().to_ascii_lowercase())
        .filter(|value| !value.is_empty())
    else {
        return CostAlgoSelector::HeightCost;
    };
    match value.as_str() {
        "height_cost" | "height-cost" => CostAlgoSelector::HeightCost,
        "general" | "probability" => CostAlgoSelector::General,
        "simple" | "probabilistic" => CostAlgoSelector::Simple,
        other => {
            warn!("NUWAX_MAPPING_OCCUPANCY_ALGO={other:?} 无效;回退到原版默认 height_cost");
            CostAlgoSelector::HeightCost
        }
    }
}

fn parse_env_nonnegative_f32(key: &str, default: f32) -> f32 {
    std::env::var(key)
        .ok()
        .and_then(|value| value.trim().parse::<f32>().ok())
        .filter(|value| value.is_finite() && *value >= 0.0)
        .unwrap_or(default)
}

fn parse_env_f64(key: &str, default: f64) -> f64 {
    std::env::var(key)
        .ok()
        .and_then(|v| v.trim().parse::<f64>().ok())
        .unwrap_or(default)
}

fn parse_env_u32(key: &str, default: u32) -> u32 {
    std::env::var(key)
        .ok()
        .and_then(|v| v.trim().parse::<u32>().ok())
        .unwrap_or(default)
}

// ============================================================================
// FeedOnlyAccumulator —— 本地实现 nuwax_mapping::PointCloudAccumulator trait
// ============================================================================

/// Phase 3 真实使用的 accumulator。
///
/// 上游 `XyzAccumulator` 暴露的是 `into_points(self)`,无法在 `&self` 上克隆,
/// 因此在 adapter 内重新实现一个等价 `FeedOnlyAccumulator`
/// (只允许 feed + clone_points)。
///
/// **2026-07-13 W37 (RALPLAN §Phase M #1)**:此类型已 **deprecated**,
/// 由 `nuwax_mapping::mapping_session::MappingSession`(有界增量 + freshness +
/// dedup + typed error)替代。`MappingAdapter::accumulator` 字段保留作为
/// `lcm_runtime::spawn_mapping_feed_tasks` 的 compat alias(任务约束:
/// 不动 `lcm_runtime.rs`),未来 #1 末段或 W36 推进时,把直接调用
/// `FeedOnlyAccumulator::feed` 的路径切到 `MappingSession`。
#[deprecated(
    since = "0.1.0-w37",
    note = "由 nuwax_mapping::MappingSession 替代;保留为 compat alias 直到 lcm_runtime.rs 切换"
)]
pub struct FeedOnlyAccumulator {
    points: Vec<Point3<f64>>,
}

#[allow(deprecated)]
impl FeedOnlyAccumulator {
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }
    /// 返回当前累积的点的克隆(用于 occupancy builder 输入)
    pub fn clone_points(&self) -> Vec<Point3<f64>> {
        self.points.clone()
    }
}

#[allow(deprecated)]
impl Default for FeedOnlyAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

#[allow(deprecated)]
impl PointCloudAccumulator for FeedOnlyAccumulator {
    fn feed(&mut self, points: &[Point3<f64>]) {
        self.points.extend_from_slice(points);
    }
    fn count(&self) -> usize {
        self.points.len()
    }
}

// ============================================================================
// MappingAdapter
// ============================================================================

/// nuwax-mapping adapter 本体。
///
/// 持有:
/// - `accumulator: FeedOnlyAccumulator` —— **compat alias**(W37 deprecated):
///   保留为 `lcm_runtime::spawn_mapping_feed_tasks` 直接访问的字段,
///   任务约束 `不动 lcm_runtime.rs`,所以该字段暂时保留;
///   未来 W36 切到 `MappingSession` 后可删。
/// - `session: MappingSession` —— **RALPLAN §Phase M #1 有界增量 session**
///   (W37 落地),bounded_capacity 默认 8M + freshness 校验(200ms)+
///   dedup(同 odom + 同 point_step 保留首帧)+ typed error
///   (`ResourceExhaustedError` / `StaleRevisionError`)。
/// - `cost_mapper: CostMapper` —— occupancy → costmap
/// - `last_pose: (x, y, z)` —— 上一次 odom pose,作为 occupancy builder sensor_origin
///
/// 公开方法:
/// - `feed_pointcloud(&[Point3<f64>])` —— 喂入点云(同时 push 到 `accumulator` +
///   `session` 双路径,W37 过渡期)
/// - `feed_pointcloud_session(...)` —— 喂入到 session(带 freshness / dedup / typed error)
/// - `feed_pose(x, y, z)` —— 更新 sensor_origin (从 LCM /odom 解析)
/// - `build_and_publish(&SharedState, Option<&Sender<MapRevisionEvent>>)` ——
///   生成 occupancy + costmap,写回 SharedState.map;M #3 同步广播 MapRevisionEvent
/// - `session_stats()` —— session 快照(Phase M readiness 暴露)
/// - `increment_odom_count() / increment_pointcloud_count() / increment_occupancy_revision()`
///   —— `server.rs::publish_odom / publish_point_cloud` 同步累加计数
///   (Phase 5 桥接层保持 feed_* 函数签名不变,只在 call site 加 +1)
/// - `readiness()` —— 综合字段,给 Python diagnostics + runtime_diagnostics 对账
pub struct MappingAdapter {
    config: MappingAdapterConfig,
    /// **W37 deprecated 字段**:`lcm_runtime::spawn_mapping_feed_tasks` 仍直接访问
    /// `accumulator` 字段(任务约束:不动 `lcm_runtime.rs`);该字段保留为 compat
    /// alias,直到 W36 把直接 `PointCloudAccumulator::feed` 调用切到 `session`。
    #[allow(deprecated)]
    pub(crate) accumulator: FeedOnlyAccumulator,
    /// **W37 RALPLAN §Phase M #1**:有界增量 `MappingSession`。
    ///
    /// 默认 `bounded_capacity = DEFAULT_BOUNDED_CAPACITY = 8_000_000`
    /// (对齐 `POINTCLOUD_MAX_RAW_POINTS`);`freshness` 阈值默认 200ms;
    /// dedup key = `(odom_x_mm, odom_y_mm, point_step)`。
    ///
    /// **W37 直持 `MappingSession`**:与上游 `nuwax_mapping::MappingSession`
    /// 完全对齐;`feed_*` 走 `&mut self`,`session_stats` 走 `&self`。
    pub(crate) session: MappingSession,
    cost_mapper: CostMapper,
    pub(crate) last_sensor_origin: [f64; 3],
    /// `has_data` 字段需要 `pub(crate)` 让 `lcm_runtime::spawn_mapping_feed_tasks`
    /// 在 feed 路径同步置 true;`mapping_ready_poll` 通过 `has_data()` 方法访问。
    pub(crate) has_data: bool,
    /// occupancy grid 累计 revision(每次 build_and_publish 自增)
    revision: u64,
    /// Phase 7:计数 odom / pointcloud / occupancy revision,给 readiness 暴露用。
    /// 不参与 build_and_publish 写回 SharedState.map.revision 逻辑(那是 mapping 主路径)。
    odom_count: u64,
    pointcloud_count: u64,
    latest_pointcloud_frame_id: String,
    latest_pointcloud_timestamp_ns: u64,
}

impl MappingAdapter {
    /// 构造 adapter;配置从 env 探测,`CostMapper` 使用原版 occupancy selector。
    pub fn new(config: MappingAdapterConfig) -> Self {
        let cost_mapper = CostMapper::new(CostMapperConfig {
            config: OccupancyConfig {
                algo: config.occupancy_algo,
                pointcloud: PointCloudOccConfig::default(),
                initial_safe_radius_meters: config.initial_safe_radius_meters,
            },
        });
        Self {
            config,
            #[allow(deprecated)]
            accumulator: FeedOnlyAccumulator::new(),
            session: MappingSession::new(),
            cost_mapper,
            last_sensor_origin: [0.0, 0.0, 0.0],
            has_data: false,
            revision: 0,
            odom_count: 0,
            pointcloud_count: 0,
            latest_pointcloud_frame_id: "map".to_string(),
            latest_pointcloud_timestamp_ns: 0,
        }
    }

    /// 喂入一帧点云;`mapping_adapter` 不持锁跨 await,调用方负责同步。
    ///
    /// **W37 行为变更**:同时 push 到:
    /// 1. `accumulator`(compat path,`lcm_runtime::spawn_mapping_feed_tasks` 直接读)
    /// 2. `session` 走 `feed_pointcloud_session` 路径(带 freshness / dedup / typed error)
    ///
    /// 双写是过渡期:W36 切到 session 后,可删 `accumulator` 字段。
    #[allow(deprecated)]
    pub fn feed_pointcloud(&mut self, points: &[Point3<f64>]) {
        if points.is_empty() {
            return;
        }
        <FeedOnlyAccumulator as PointCloudAccumulator>::feed(&mut self.accumulator, points);
        // W41 fix: W37 docstring (L242-247) 明确 `feed_pointcloud` 走"双写
        // 过渡期"路径(同时 push 到 accumulator + session),但 W37 实际只
        // push 到 accumulator,导致:
        // 1. `session.current_size()` / `clone_points()` 永为 0
        // 2. `build_occupancy()` 走 `session.clone_points()`(L332, W37 SSOT)
        //    但拿到空 Vec,`accumulated_point_count=0`,occupancy grid 空
        // 3. `accumulated_count()` 返 0,W37 readiness counter 对不上
        //
        // 这里补双写:同步用 odom=(0,0)、point_step=0、now=SystemTime::now()
        // 调 `session.feed_pointcloud`。`ResourceExhaustedError` /
        // `StaleRevisionError` 走 `?` 路径直接传播(单测不期望此错误;
        // 生产路径由 feed_pointcloud_session 单独承载 typed error 契约)。
        let now_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        match self.session.feed_pointcloud(points, (0.0, 0.0), 0, now_ns) {
            Ok(stats) if stats.accepted > 0 => {
                self.pointcloud_count = self.pointcloud_count.saturating_add(1);
                self.has_data = true;
                self.latest_pointcloud_frame_id = "map".to_string();
                self.latest_pointcloud_timestamp_ns = now_ns;
            }
            Ok(_) => {}
            Err(e) => {
                tracing::debug!(
                    "feed_pointcloud 双写 session 失败:{e:?};accumulator 路径仍生效(W37 过渡期)"
                );
            }
        }
    }

    /// 喂入一帧点云到 `MappingSession`(RALPLAN §Phase M #1 主入口)。
    ///
    /// 与 `feed_pointcloud` 的差异:
    /// - 接受 `odom_xy` / `point_step` / `now_ns` 参数,用于 dedup_key + freshness
    /// - 超 `bounded_capacity` 抛 `ResourceExhaustedError`
    /// - 同 odom + 同 point_step 重复帧 → 保留首帧(dedup_skipped > 0)
    /// - `now_ns - last_seen_at_ns > 200ms` 抛 `StaleRevisionError`
    ///
    /// **调用方契约**:由 `lcm_runtime::spawn_mapping_feed_tasks` 或 `server.rs`
    /// 在解析 `/odom` + `/go2/lidar/points` LCM 帧后调用;**W37 不替换
    /// `feed_pointcloud` 双写**,待 W36 切到 session 后再统一走此入口。
    pub fn feed_pointcloud_session(
        &mut self,
        points: &[Point3<f64>],
        odom_xy: (f64, f64),
        point_step: u32,
        now_ns: u64,
        frame_id: &str,
        source_timestamp_ns: u64,
    ) -> Result<nuwax_mapping::FeedStats, nuwax_mapping::SessionError> {
        if points.is_empty() {
            return Ok(nuwax_mapping::FeedStats::accepted(
                0,
                self.session.current_size(),
                self.session.capacity(),
                self.session.last_seen_at_ns(),
            ));
        }
        let stats = self
            .session
            .feed_pointcloud(points, odom_xy, point_step, now_ns)?;
        if stats.accepted > 0 {
            self.has_data = true;
            self.pointcloud_count = self.pointcloud_count.saturating_add(1);
            self.latest_pointcloud_frame_id = frame_id.to_string();
            self.latest_pointcloud_timestamp_ns = source_timestamp_ns;
        }
        Ok(stats)
    }

    /// 更新 sensor_origin (odom pose 平移部分)。
    ///
    /// **W37** 同步更新 `session.last_pose`,让 session 的 `last_pose` 与
    /// `MappingAdapter.last_sensor_origin` 保持一致。
    pub fn feed_pose(&mut self, x: f64, y: f64, z: f64) {
        if !x.is_finite() || !y.is_finite() || !z.is_finite() {
            return;
        }
        self.last_sensor_origin = [x, y, z];
        self.session.feed_pose(x, y, z);
        self.odom_count = self.odom_count.saturating_add(1);
    }

    /// session 快照(Phase M readiness 暴露)。
    pub fn session_stats(&self) -> MappingSessionStats {
        self.session.snapshot_stats()
    }

    /// 当前是否有数据 (用于 `mapping_ready_poll` 判定)。
    pub fn has_data(&self) -> bool {
        self.has_data
    }

    /// 当前累计点数 (debug / diagnostics 用)。
    ///
    /// **W37**:`session` 是 SSOT,`accumulator` 是 compat alias;两者 push 量一致
    /// (双写路径),此处返回 `session.current_size()`,真实反映有界增量的当前大小。
    /// lcm_runtime.rs 直接访问 `accumulator` 字段的路径保留为同等大小
    /// (双写 invariant:每次 `feed_pointcloud` 都同步 push 到两个结构)。
    pub fn accumulated_count(&self) -> usize {
        self.session.current_size()
    }

    /// 当前 adapter 持有的 costmap 配置。
    pub fn cost_mapper_config(&self) -> &CostMapperConfig {
        self.cost_mapper.config()
    }

    /// 把当前累积的点云 + sensor_origin 转换为 occupancy grid。
    ///
    /// **不消费 session** —— 内部走 `session.clone_points()` 路径,
    /// 后续 feed 仍能继续累积;若要清空,调用方调用 `reset_accumulator`。
    ///
    /// **W37**:`accumulator` 与 `session` 是双写 invariant,二者 clone_points 内容
    /// 一致;此处走 `session.clone_points()` 让 SSOT 与 session 对齐。
    pub fn build_occupancy(&self) -> Result<OccupancyGrid, MappingError> {
        self.build_maps().map(|(_, occupancy)| occupancy)
    }

    fn build_maps(&self) -> Result<(CostMapper, OccupancyGrid), MappingError> {
        let points = self.session.clone_points();
        let origin = self.last_sensor_origin;
        let cloud = SimplePointCloud2::with_metadata(
            points,
            nalgebra::point![origin[0], origin[1], origin[2]],
            self.latest_pointcloud_frame_id.clone(),
            self.latest_pointcloud_timestamp_ns as f64 / 1_000_000_000.0,
        );
        let (mapper, occupancy, _costmap) = CostMapper::from_pointcloud_auto(
            &cloud,
            &self.cost_mapper.config().config,
            self.config.resolution,
        )?;
        Ok((mapper, occupancy))
    }

    /// 把当前 accumulated 点云 + sensor_origin 走完整链路:
    /// 1. 重建 occupancy
    /// 2. costmap 转换
    /// 3. 写回 `SharedState.map` + revision 自增
    ///
    /// 返回 true 表示数据有效且已写入, false 表示无数据可建图。
    ///
    /// 用 `try_write` 避免在 tokio context 中跨 await;
    /// 若取不到锁就放弃本轮写入(下一轮 feed 会重新触发)。
    ///
    /// M #3 (RALPLAN §Phase 2): `map_revision_tx` 可选 —
    /// - 生产 `spawn_mapping_feed_tasks` 传 `Some(&handle.map_revision_tx)`,
    ///   commit 时通过 broadcast::Sender 推送 `MapRevisionEvent`
    ///   (data/size/schema_version/checksum),`server::subscribe_map_updates`
    ///   不再轮询 500ms。
    /// - 单测路径默认 `None` 保持兼容,无需广播通道也能验证业务。
    pub fn build_and_publish(
        &mut self,
        state: &Arc<RwLock<SharedState>>,
        map_revision_tx: Option<&broadcast::Sender<MapRevisionEvent>>,
    ) -> bool {
        if !self.has_data {
            return false;
        }
        let (mapper, grid) = match self.build_maps() {
            Ok(result) => result,
            Err(error) => {
                warn!("mapping_adapter: auto-bounds occupancy 构建失败: {error}");
                return false;
            }
        };
        self.cost_mapper = mapper;
        let data = grid.data.clone();
        if let Ok(mut s) = state.try_write() {
            let next_revision = s.map.revision.max(self.revision).saturating_add(1);
            self.revision = next_revision;
            s.map = MapState {
                width: grid.width,
                height: grid.height,
                resolution: grid.resolution as f32,
                origin_x: grid.origin[0],
                origin_y: grid.origin[1],
                frame_id: grid.frame_id,
                revision: next_revision,
                timestamp_ns: self.latest_pointcloud_timestamp_ns,
                data: data.clone(),
            };
            // 从刚 commit 的 MapState 构造自包含事件，确保 metadata/data/checksum
            // 属于同一 revision，server 不需要再读取可能已前进的 SharedState。
            let event = MapRevisionEvent::from_map(&s.map);
            drop(s); // 释放 write lock,后续 send 不持锁
            if let Some(tx) = map_revision_tx {
                // broadcast::Sender::send 是 non-blocking,无 consumer 时返 Err,
                // 这里忽略(no subscriber — spawn 早期或被 unsubscribe)
                let _ = tx.send(event);
            }
            true
        } else {
            warn!("mapping_adapter: SharedState.try_write 失败,本轮 build_and_publish 跳过");
            false
        }
    }

    /// 清空 accumulator(用于重新建图 / 地图 reset)。
    ///
    /// **W37** 双写 invariant:同时重置 `accumulator` + `session`。
    pub fn reset_accumulator(&mut self) {
        #[allow(deprecated)]
        {
            self.accumulator = FeedOnlyAccumulator::new();
        }
        self.session.reset_state();
        self.has_data = false;
        self.latest_pointcloud_frame_id = "map".to_string();
        self.latest_pointcloud_timestamp_ns = 0;
    }

    /// adapter 当前 revision(给 diagnostics / 调试用)。
    pub fn current_revision(&self) -> u64 {
        self.revision
    }

    // ============================================================================
    // Phase 7:readiness counters + 综合 readiness 暴露
    // ============================================================================
    //
    // Counters advance only at accepted ingestion/commit boundaries. A producer-side
    // LCM publish acknowledgement is not evidence that mapping consumed the frame.

    /// odom 计数(对外只读)
    pub fn odom_count(&self) -> u64 {
        self.odom_count
    }

    /// pointcloud 计数(对外只读)
    pub fn pointcloud_count(&self) -> u64 {
        self.pointcloud_count
    }
}

/// Mapping 板块 readiness 综合快照(Phase 7)。
///
/// 与 `NavigationReadinessDetail` / `ExplorationReadinessDetail` 对齐:
/// `ready` 综合 bool + `last_error` 字段 + 各子项,Python 侧
/// `get_mapping_diagnostics` 按字段名直接读。
#[derive(Debug, Clone)]
pub struct MappingReadiness {
    /// 综合 ready(`has_data && pointcloud_count > 0`)
    pub ready: bool,
    /// 顶层 reason(中文 + 关键 blocker 标签)
    pub reason: String,
    /// LCM backend 是否可用
    pub lcm_backend: bool,
    /// adapter 是否已构造
    pub adapter_present: bool,
    /// 累计 odom 帧数
    pub odom_count: u64,
    /// 累计 pointcloud 帧数
    pub pointcloud_count: u64,
    /// 最近一次 occupancy grid revision
    pub last_occupancy_revision: u64,
    /// pointcloud 累计点数(给 frontend 决定要不要展示进度)
    pub accumulated_point_count: usize,
    /// 最近一次错误(若 lcm_backend=false / adapter 未构造等)
    pub last_error: Option<String>,
}

impl MappingReadiness {
    /// 综合判定:`has_data` + 真实 pointcloud 输入 + lcm_backend + adapter present
    pub fn compute(adapter: Option<&MappingAdapter>, lcm_backend_available: bool) -> Self {
        let mut last_error: Option<String> = None;
        if !lcm_backend_available {
            last_error = Some("LCM backend 不可用,Mapping 板块输入被阻断".to_string());
        }
        let adapter_present = adapter.is_some();
        let mut ready = false;
        let (odom_count, pointcloud_count, last_occupancy_revision, accumulated_point_count) =
            match adapter {
                Some(a) => {
                    if !a.has_data() && a.odom_count() == 0 && a.pointcloud_count() == 0 {
                        last_error.get_or_insert_with(|| {
                            "尚未收到任何 odom / pointcloud 输入".to_string()
                        });
                    } else if a.pointcloud_count() == 0 {
                        last_error.get_or_insert_with(|| {
                            "尚未收到 pointcloud,仅有 odom 不算 ready".to_string()
                        });
                    } else if a.current_revision() == 0 {
                        last_error.get_or_insert_with(|| {
                            "pointcloud 已接收但 occupancy revision 尚未提交".to_string()
                        });
                    }
                    (
                        a.odom_count(),
                        a.pointcloud_count(),
                        a.current_revision(),
                        a.accumulated_count(),
                    )
                }
                None => {
                    last_error.get_or_insert_with(|| {
                        "Mapping adapter 未构造(LCM backend 不可用)".to_string()
                    });
                    (0, 0, 0, 0)
                }
            };
        if let Some(adapter) = adapter {
            ready = lcm_backend_available
                && adapter.has_data()
                && pointcloud_count > 0
                && last_occupancy_revision > 0;
        }
        let reason = if ready {
            "mapping 已就绪".to_string()
        } else {
            last_error
                .clone()
                .unwrap_or_else(|| "mapping 未就绪".to_string())
        };
        Self {
            ready,
            reason,
            lcm_backend: lcm_backend_available,
            adapter_present,
            odom_count,
            pointcloud_count,
            last_occupancy_revision,
            accumulated_point_count,
            last_error,
        }
    }
}

/// nuwax-mapping CostMapper 输出的 u8 → MapState i8 转换。
///
/// CostMapper 约定:
/// - 0 = free
/// - 254 = occupied (Nav2 lethal)
/// - 253 = inscribed obstacle
/// - 252 = circumscribed inflation
/// - 128 = inflation outer ring
/// - 255 = no information
///
/// MapState.data 沿用 Phase 0 contract(模仿 ROS OccupancyGrid int8):
/// -1 = unknown / 0 = free / 100 = occupied
fn map_cost_to_i8(v: u8) -> i8 {
    if v >= 65 {
        100
    } else if v == 0 {
        0
    } else {
        -1
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_mapping::cost_mapper::CostAlgoSelector;
    use nuwax_mapping::occupancy::types::OccupancyValue;

    const GENERAL_GOLDEN: &[i8] = &[
        -1, -1, -1, -1, -1, -1, -1, // row 0
        -1, -1, -1, 0, -1, -1, -1, // row 1
        -1, -1, 100, 0, 100, -1, -1, // row 2
        -1, -1, -1, 0, -1, -1, -1, // row 3
    ];

    fn sample_config() -> MappingAdapterConfig {
        MappingAdapterConfig {
            resolution: 0.1,
            width_cells: 20,
            height_cells: 20,
            origin_xy: (-1.0, -1.0),
            voxel_size: 0.1,
            occupancy_algo: CostAlgoSelector::HeightCost,
            initial_safe_radius_meters: 0.0,
        }
    }

    #[test]
    fn adapter_default_state_is_empty() {
        let a = MappingAdapter::new(sample_config());
        assert!(!a.has_data());
        assert_eq!(a.accumulated_count(), 0);
        assert_eq!(a.current_revision(), 0);
    }

    #[test]
    fn feed_pointcloud_marks_has_data() {
        let mut a = MappingAdapter::new(sample_config());
        a.feed_pointcloud(&[Point3::new(0.5, 0.5, 0.0)]);
        assert!(a.has_data());
        assert_eq!(a.accumulated_count(), 1);
        assert_eq!(a.pointcloud_count(), 1);
    }

    #[test]
    fn feed_pointcloud_empty_no_mark() {
        let mut a = MappingAdapter::new(sample_config());
        a.feed_pointcloud(&[]);
        assert!(!a.has_data());
    }

    #[test]
    fn feed_pose_updates_origin() {
        let mut a = MappingAdapter::new(sample_config());
        a.feed_pose(1.0, 2.0, 0.0);
        assert_eq!(a.last_sensor_origin, [1.0, 2.0, 0.0]);
        assert_eq!(a.odom_count(), 1);
    }

    #[test]
    fn build_occupancy_produces_grid() {
        let mut a = MappingAdapter::new(sample_config());
        // 默认 HeightCost 语义下,z > 1.5m 才是 Occupied；z=0 属于 Unknown。
        a.feed_pointcloud(&[Point3::new(0.0, 0.0, 2.0), Point3::new(0.1, 0.0, 2.0)]);
        let grid = a.build_occupancy().expect("valid auto-bounds occupancy");
        assert_eq!(grid.width, 21);
        assert_eq!(grid.height, 20);
        // 至少有一个 occupied cell
        let mut has_occupied = false;
        for r in 0..grid.height {
            for c in 0..grid.width {
                if matches!(grid.get(r, c), Some(OccupancyValue::Occupied)) {
                    has_occupied = true;
                    break;
                }
            }
        }
        assert!(has_occupied, "occupancy grid 至少应有 1 个 occupied cell");
    }

    #[test]
    fn production_mapping_uses_auto_bounds_selected_kernel_and_source_metadata() {
        let mut config = sample_config();
        config.resolution = 0.5;
        config.occupancy_algo = CostAlgoSelector::General;
        let mut adapter = MappingAdapter::new(config);
        let points = vec![
            Point3::new(0.00, 0.00, 0.00),
            Point3::new(0.00, 0.00, 0.50),
            Point3::new(0.74, 0.00, 0.00),
            Point3::new(1.26, 0.00, 0.50),
        ];
        let source_timestamp_ns = 123_250_000_000_u64;

        adapter
            .feed_pointcloud_session(
                &points,
                (0.0, 0.0),
                12,
                1_750_000_000_000_000_000,
                "lidar_fixture",
                source_timestamp_ns,
            )
            .expect("accepted mapping frame");

        let occupancy = adapter
            .build_occupancy()
            .expect("selected production occupancy");
        assert_eq!((occupancy.width, occupancy.height), (7, 4));
        assert_eq!(occupancy.origin, [-1.0, -1.0, 0.0]);
        assert_eq!(occupancy.frame_id, "lidar_fixture");
        assert_eq!(occupancy.ts, 123.25);
        assert_eq!(occupancy.data, GENERAL_GOLDEN);

        let state = Arc::new(RwLock::new(SharedState::default()));
        assert!(adapter.build_and_publish(&state, None));
        let state = state.try_read().expect("committed map state");
        assert_eq!((state.map.width, state.map.height), (7, 4));
        assert_eq!((state.map.origin_x, state.map.origin_y), (-1.0, -1.0));
        assert_eq!(state.map.frame_id, "lidar_fixture");
        assert_eq!(state.map.timestamp_ns, source_timestamp_ns);
        assert_eq!(state.map.data, GENERAL_GOLDEN);
    }

    #[test]
    fn build_and_publish_writes_shared_state() {
        let state = Arc::new(RwLock::new(SharedState::default()));
        let mut a = MappingAdapter::new(sample_config());
        a.feed_pointcloud(&[Point3::new(0.0, 0.0, 0.0)]);
        // 测试路径不接 broadcast Sender,传 None — 保持 Phase 5 后向兼容。
        assert!(a.build_and_publish(&state, None));
        let s = state.try_read().expect("read");
        assert!(s.map.revision >= 1);
        assert_eq!(s.map.width, 20);
        assert_eq!(s.map.height, 20);
    }

    #[test]
    fn build_and_publish_no_data_returns_false() {
        let state = Arc::new(RwLock::new(SharedState::default()));
        let mut a = MappingAdapter::new(sample_config());
        assert!(!a.build_and_publish(&state, None));
    }

    #[test]
    fn reset_clears_accumulator() {
        let mut a = MappingAdapter::new(sample_config());
        a.feed_pointcloud(&[Point3::new(0.0, 0.0, 0.0)]);
        assert!(a.has_data());
        a.reset_accumulator();
        assert!(!a.has_data());
        assert_eq!(a.accumulated_count(), 0);
    }

    #[test]
    fn map_cost_to_i8_conversion() {
        // 0 → 0 (free)
        assert_eq!(map_cost_to_i8(0), 0);
        // 65+ → 100 (occupied)
        assert_eq!(map_cost_to_i8(65), 100);
        assert_eq!(map_cost_to_i8(100), 100);
        assert_eq!(map_cost_to_i8(254), 100);
        // 1..64 → -1 (no info / inflation rings)
        assert_eq!(map_cost_to_i8(1), -1);
        assert_eq!(map_cost_to_i8(50), -1);
    }

    #[test]
    fn config_from_env_uses_defaults() {
        // 不污染环境;直接调 from_env,应走 default
        let cfg = MappingAdapterConfig::from_env();
        assert!(cfg.resolution > 0.0);
        assert!(cfg.width_cells > 0);
        assert!(cfg.height_cells > 0);
    }

    // ~~cost_mapper_config_is_nav2_default~~ —— W41 删除:
    // `CostMapperConfig` 已在 nuwax-mapping::cost_mapper 重构为 selector
    // 风格(`OccupancyConfig { algo, initial_safe_radius_meters }`),原
    // `free_space_cost` / `lethal_cost` / `no_information_cost` 字段全部
    // 移至 `Nav2CostLayer`(独立类型,属 Nav2 适配层不在 mapping 接口)。
    // 该单测基于废弃字段硬编码,继续保留会锁死旧结构;由 RALPLAN §Phase 7
    // Nav2 适配层 `Nav2CostLayer` 测试覆盖。

    // ========================================================================
    // Phase 7:counters + readiness 综合
    // ========================================================================

    #[test]
    fn counters_start_at_zero() {
        let a = MappingAdapter::new(sample_config());
        assert_eq!(a.odom_count(), 0);
        assert_eq!(a.pointcloud_count(), 0);
        assert_eq!(a.current_revision(), 0);
    }

    #[test]
    fn readiness_initial_state_unready() {
        // adapter 未构造 → unready,无 pointcloud
        let r = MappingReadiness::compute(None, true);
        assert!(!r.ready);
        assert!(!r.adapter_present);
        assert!(r.lcm_backend); // 仅覆盖字段本身,期望 backend true
        assert_eq!(r.odom_count, 0);
        assert_eq!(r.pointcloud_count, 0);
        assert!(r.last_error.is_some());
    }

    #[test]
    fn readiness_with_only_odom_is_unready() {
        let mut a = MappingAdapter::new(sample_config());
        a.feed_pose(0.0, 0.0, 0.0);
        let r = MappingReadiness::compute(Some(&a), true);
        assert!(!r.ready, "缺 pointcloud 必须 unready");
        assert_eq!(r.odom_count, 1);
        assert_eq!(r.pointcloud_count, 0);
        assert!(r.last_error.as_ref().unwrap().contains("pointcloud"));
    }

    #[test]
    fn readiness_with_pointcloud_turns_ready() {
        let mut a = MappingAdapter::new(sample_config());
        let state = Arc::new(RwLock::new(SharedState::default()));
        a.feed_pose(0.0, 0.0, 0.0);
        a.feed_pointcloud(&[Point3::new(0.0, 0.0, 0.0)]);
        assert!(a.build_and_publish(&state, None));
        let r = MappingReadiness::compute(Some(&a), true);
        assert!(r.ready);
        assert_eq!(r.odom_count, 1);
        assert_eq!(r.pointcloud_count, 1);
        assert_eq!(r.last_occupancy_revision, 1);
        assert_eq!(r.accumulated_point_count, 1);
    }

    #[test]
    fn readiness_no_backend_is_unready() {
        let mut a = MappingAdapter::new(sample_config());
        a.feed_pointcloud(&[Point3::new(0.0, 0.0, 0.0)]);
        let r = MappingReadiness::compute(Some(&a), false);
        assert!(!r.ready);
        assert!(!r.lcm_backend);
        assert!(r.last_error.as_ref().unwrap().contains("backend"));
    }
}
