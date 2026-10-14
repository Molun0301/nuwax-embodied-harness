//! `mapping_session.rs` — Phase M #1 有界增量 `MappingSession`
//!
//! 替换 `nuwax-geometry-runtime/src/mapping_adapter.rs::FeedOnlyAccumulator`:
//! - 默认 `bounded_capacity = 8_000_000`(`POINTCLOUD_MAX_RAW_POINTS` 默认值)
//! - 点预算满时滚动淘汰最旧点，持续接收新数据
//! - freshness 只拒绝比消息时间水位倒退超过容差的旧帧
//! - 通用 dedup 按时间戳、内容和采集位姿；已准入帧按来源身份，共享 4096 帧历史
//! - `make_contiguous_points` 为同步 occupancy builder 提供零拷贝切片
//!
//! ## typed error 与 W12 Python 对应
//! - `ResourceExhaustedError` ↔ Python `ResourceExhaustedError`
//! - `StaleRevisionError` ↔ Python `StaleRevisionError`
//! - 与 P0B #4 的 stable errors 一致(`RESOURCE_EXHAUSTED` / `STALE_REVISION`)
//!
//! ## 实现注意
//! - 不持有外部时钟；消息时间戳由调用方从 PointCloud2 header 传入
//! - `VecDeque` 维护点预算，`HashSet + VecDeque` 维护固定长度的重放身份历史

use std::collections::{hash_map::DefaultHasher, HashSet, VecDeque};
use std::fmt;
use std::hash::{Hash, Hasher};

use nalgebra::Point3;

use crate::pointclouds::PointCloudAccumulator;

// ============================================================================
// 配置常量
// ============================================================================

/// `bounded_capacity` 默认值 —— 对齐 `POINTCLOUD_MAX_RAW_POINTS = 8_000_000`。
///
/// 调用方可通过 `MappingSession::with_capacity` 覆盖;默认 8M 是 sanity cap,
/// 防止恶意 / 异常上游 LCM 帧把 adapter 撑爆。
pub const DEFAULT_BOUNDED_CAPACITY: usize = 8_000_000;

/// 消息时间允许的逆序容差。比最高水位落后超过 200ms 才视为 stale。
pub const DEFAULT_STALE_THRESHOLD_NS: u64 = 200 * 1_000_000;

/// 最近重放帧的去重历史上限。该上限只按帧计数，与点预算相互独立。
pub const DEFAULT_DEDUP_HISTORY_FRAMES: usize = 4_096;

/// odom 坐标量化精度 —— 1mm。对齐 W12 Python `_odom_key` 的 `* 1000.0`。
///
/// 用于 dedup_key 的 mm 量化;同 mm 网格内的 odom 视为同一坐标,避免浮点误差导致
/// dedup miss。
pub const ODOM_QUANTIZE_MM: i64 = 1000;

// ============================================================================
// typed error
// ============================================================================

/// `MappingSession` 错误基类(对应 W12 Python `SessionError`)。
#[derive(Debug)]
pub enum SessionError {
    /// 配置为零容量，无法保留任何输入点。
    ResourceExhausted {
        /// 配置的最大点数。
        capacity: usize,
        /// 本次 feed 前已经累积的点数。
        current_size: usize,
        /// 本次批次尝试写入的点数。
        attempted_points: usize,
    },
    /// 消息时间戳比已接收最高水位倒退超过容差。
    StaleRevision {
        /// 已接收消息时间戳的最高水位。
        last_seen_at_ns: u64,
        /// 被判定为 stale 的当前帧时间。
        now_ns: u64,
        /// 允许的最大逆序幅度。
        threshold_ns: u64,
    },
    /// 输入 frame 无法通过已建立的刚体变换进入目标 map frame。
    FrameTransformUnavailable {
        /// 当前输入声明的源 frame。
        source_frame_id: String,
        /// mapping session 的全局输出 frame。
        target_frame_id: String,
        /// 稳定、可诊断的拒绝原因。
        reason: String,
    },
}

impl fmt::Display for SessionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ResourceExhausted {
                capacity,
                current_size,
                attempted_points,
            } => write!(
                f,
                "MappingSession 资源耗尽:capacity={capacity} current_size={current_size} attempted={attempted_points}"
            ),
            Self::StaleRevision {
                last_seen_at_ns,
                now_ns,
                threshold_ns,
            } => write!(
                f,
                "MappingSession revision 陈旧:last={last_seen_at_ns}ns now={now_ns}ns threshold={threshold_ns}ns"
            ),
            Self::FrameTransformUnavailable {
                source_frame_id,
                target_frame_id,
                reason,
            } => write!(
                f,
                "MappingSession frame transform 不可用:source={source_frame_id} target={target_frame_id} reason={reason}"
            ),
        }
    }
}

impl std::error::Error for SessionError {}

/// typed alias:`RESOURCE_EXHAUSTED`(与 P0B §4 stable errors 对齐)
pub type ResourceExhaustedError = SessionError;

/// typed alias:`STALE_REVISION`(与 P0B §4 stable errors 对齐)
pub type StaleRevisionError = SessionError;

// ============================================================================
// FeedStats —— 单次 feed_pointcloud 的结果统计
// ============================================================================

/// `feed_pointcloud` 单次调用的统计结果。
///
/// 对齐 W12 Python `tools/mapping/w8_mapping_session.py::FeedStats` 字段命名:
/// - `accepted`:实际入 bounded_ring 的点数
/// - `dedup_skipped`:dedup_set 命中跳过的点数
/// - `current_size`:bounded_ring 当前元素数
/// - `capacity`:capacity 上限
/// - `last_seen_at_ns`:更新后的 last_seen_at_ns
/// - `published`:本次是否触发了 publish 路径(此处为 always false,
///   publish 由 `build_and_publish` 显式调用,与 feed 解耦)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FeedStats {
    /// 本次实际接收的点数。
    pub accepted: usize,
    /// 本次因 dedup 跳过的点数。
    pub dedup_skipped: usize,
    /// 为给新点腾出预算而移除的旧点，以及超大批次中未保留的最旧输入点。
    pub evicted: usize,
    /// feed 完成后的累积点数。
    pub current_size: usize,
    /// session 配置的最大点数。
    pub capacity: usize,
    /// feed 完成后的 freshness 水位。
    pub last_seen_at_ns: u64,
    /// 本次 feed 是否同时触发发布。
    pub published: bool,
}

impl FeedStats {
    /// dedup 全命中时的便捷构造(零 accepted,全 skipped)。
    pub fn dedup_only(
        skipped: usize,
        current_size: usize,
        capacity: usize,
        last_seen_at_ns: u64,
    ) -> Self {
        Self {
            accepted: 0,
            dedup_skipped: skipped,
            evicted: 0,
            current_size,
            capacity,
            last_seen_at_ns,
            published: false,
        }
    }

    /// 正常 accept 后的便捷构造。
    pub fn accepted(
        accepted: usize,
        current_size: usize,
        capacity: usize,
        last_seen_at_ns: u64,
    ) -> Self {
        Self {
            accepted,
            dedup_skipped: 0,
            evicted: 0,
            current_size,
            capacity,
            last_seen_at_ns,
            published: false,
        }
    }

    /// 接收新点并滚动淘汰旧点时的结果。
    pub fn accepted_with_eviction(
        accepted: usize,
        evicted: usize,
        current_size: usize,
        capacity: usize,
        last_seen_at_ns: u64,
    ) -> Self {
        Self {
            accepted,
            dedup_skipped: 0,
            evicted,
            current_size,
            capacity,
            last_seen_at_ns,
            published: false,
        }
    }
}

// ============================================================================
// SessionStats —— snapshot 形态的诊断快照
// ============================================================================

/// `snapshot_stats` 返回的诊断快照(Phase M #1 readiness 暴露)。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SessionStats {
    /// session 配置的最大点数。
    pub capacity: usize,
    /// 当前累积点数。
    pub current_size: usize,
    /// 最近观察到的非零时间戳。
    pub last_seen_at_ns: u64,
    /// 最近一次传入的 sensor origin。
    pub last_pose: [f64; 3],
    /// 当前 dedup key 数量。
    pub dedup_set_size: usize,
    /// 成功接收点数据的单调代次，供上层避免重复全量重建。
    pub generation: u64,
    /// session 生命周期内累计滚动淘汰的点数。
    pub evicted_point_count: u64,
}

/// canonical 生产路径已准入帧的不可变来源身份。
///
/// 这是可信生产方元数据，不是身份认证或准入证明。构造前，调用方必须已将身份绑定到
/// 不可变帧数据，并执行 runtime generation / source sequence 的准入边界检查。
/// 同一身份搭配不同内容属于调用方违约；`MappingSession` 不扫描点云来检测该行为。
/// 只有身份仍在有界历史中时才保证去重。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct AdmittedFrameKey {
    backend: i32,
    runtime_generation: i64,
    source_sequence: i64,
}

impl AdmittedFrameKey {
    /// 只校验三个身份标量均为正数，不读取或复制帧数据。
    ///
    /// 正数校验不能证明准入；调用方必须遵守 [`AdmittedFrameKey`] 的不可变帧与
    /// 生产方代次、序列边界契约。
    pub fn from_admitted_source(
        backend: i32,
        runtime_generation: i64,
        source_sequence: i64,
    ) -> Option<Self> {
        if backend <= 0 || runtime_generation <= 0 || source_sequence <= 0 {
            return None;
        }
        Some(Self {
            backend,
            runtime_generation,
            source_sequence,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) enum FrameDedupKey {
    Legacy((i64, i64, u32, u64, u64)),
    Admitted(AdmittedFrameKey),
}

#[cfg(test)]
std::thread_local! {
    static POINTS_FINGERPRINT_CALLS: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

// ============================================================================
// MappingSession —— 核心有界增量 session
// ============================================================================

/// 有界增量 mapping session(Phase M #1 #1)。
///
/// `current_window` 是唯一点数据容器；容量压力滚动淘汰旧点。freshness 使用消息
/// 时间最高水位，dedup 使用固定长度的完整帧身份历史。
#[derive(Clone)]
pub struct MappingSession {
    /// `bounded_capacity` —— 与 `POINTCLOUD_MAX_RAW_POINTS` 默认对齐 8M
    pub(crate) bounded_capacity: usize,
    /// `current_window` —— 当前累积的 `Point3<f64>`(顺序保留,支持 clone_points)
    pub(crate) current_window: VecDeque<Point3<f64>>,
    /// 最近接收的帧身份集合，只用于识别消息重放。
    pub(crate) dedup_set: HashSet<FrameDedupKey>,
    /// 与 `dedup_set` 同步的 FIFO，用于执行固定上限淘汰。
    pub(crate) dedup_order: VecDeque<FrameDedupKey>,
    pub(crate) dedup_history_capacity: usize,
    /// `freshness` —— 已接收消息时间戳的最高水位。
    pub(crate) freshness: u64,
    /// 当前 sensor_origin (从 odom 平移部分)
    pub(crate) last_pose: [f64; 3],
    /// freshness 阈值(纳秒),默认 200ms
    pub(crate) stale_threshold_ns: u64,
    generation: u64,
    evicted_point_count: u64,
}

impl MappingSession {
    /// 构造默认 `MappingSession`(capacity = `DEFAULT_BOUNDED_CAPACITY` = 8M)。
    pub fn new() -> Self {
        Self::with_capacity(DEFAULT_BOUNDED_CAPACITY)
    }

    /// 构造指定 `bounded_capacity` 的 `MappingSession`。
    ///
    /// `capacity = 0` 是合法但无意义的边界(capacity=0 任何 feed 必抛
    /// `ResourceExhausted`);此处不校验,允许测试 capacity=0 路径。
    pub fn with_capacity(bounded_capacity: usize) -> Self {
        Self {
            bounded_capacity,
            current_window: VecDeque::with_capacity(bounded_capacity.min(1024)),
            dedup_set: HashSet::new(),
            dedup_order: VecDeque::with_capacity(DEFAULT_DEDUP_HISTORY_FRAMES),
            dedup_history_capacity: DEFAULT_DEDUP_HISTORY_FRAMES,
            freshness: 0,
            last_pose: [0.0, 0.0, 0.0],
            stale_threshold_ns: DEFAULT_STALE_THRESHOLD_NS,
            generation: 0,
            evicted_point_count: 0,
        }
    }

    /// 显式指定 freshness 阈值(覆盖 `DEFAULT_STALE_THRESHOLD_NS = 200ms`)。
    pub fn with_stale_threshold_ns(mut self, stale_threshold_ns: u64) -> Self {
        self.stale_threshold_ns = stale_threshold_ns;
        self
    }

    /// 覆盖按帧计数的重放去重历史上限。
    pub fn with_dedup_history_capacity(mut self, capacity: usize) -> Self {
        self.dedup_history_capacity = capacity;
        self.dedup_order = VecDeque::with_capacity(capacity.min(1024));
        self
    }

    /// 当前 `bounded_capacity` 上限。
    pub fn capacity(&self) -> usize {
        self.bounded_capacity
    }

    /// 当前 `current_window` 元素数。
    pub fn current_size(&self) -> usize {
        self.current_window.len()
    }

    /// 最近一次 feed 的 `now_ns`(单调时钟);`0` 表示尚未 feed 过。
    pub fn last_seen_at_ns(&self) -> u64 {
        self.freshness
    }

    /// 当前 sensor_origin (odom 平移部分)。
    pub fn last_pose(&self) -> [f64; 3] {
        self.last_pose
    }

    /// dedup_set 元素数(诊断用)。
    pub fn dedup_set_size(&self) -> usize {
        self.dedup_set.len()
    }

    /// 成功接收点数据的单调代次。
    pub fn generation(&self) -> u64 {
        self.generation
    }

    /// 当前累积点的克隆(供 occupancy builder 输入;`&self` 上 read-only)。
    ///
    /// 与 `FeedOnlyAccumulator::clone_points` 同语义,不消费 session。
    pub fn clone_points(&self) -> Vec<Point3<f64>> {
        self.current_window.iter().copied().collect()
    }

    /// 返回连续内存中的当前滚动窗口，供同步构建器零拷贝读取。
    pub fn make_contiguous_points(&mut self) -> &[Point3<f64>] {
        self.current_window.make_contiguous()
    }

    /// 构造 dedup_key —— 量化 odom_x/odom_y 到 mm,point_step 原值。
    ///
    /// 与 W12 Python `_odom_key` 1:1 对齐(`x_mm = round(x * 1000)`).
    pub fn dedup_key(odom_xy: (f64, f64), point_step: u32) -> (i64, i64, u32) {
        let x_mm = (odom_xy.0 * ODOM_QUANTIZE_MM as f64).round() as i64;
        let y_mm = (odom_xy.1 * ODOM_QUANTIZE_MM as f64).round() as i64;
        (x_mm, y_mm, point_step)
    }

    fn points_fingerprint(points: &[Point3<f64>]) -> u64 {
        #[cfg(test)]
        POINTS_FINGERPRINT_CALLS.with(|calls| calls.set(calls.get() + 1));

        let mut hasher = DefaultHasher::new();
        points.len().hash(&mut hasher);
        for point in points {
            point.x.to_bits().hash(&mut hasher);
            point.y.to_bits().hash(&mut hasher);
            point.z.to_bits().hash(&mut hasher);
        }
        hasher.finish()
    }

    fn frame_dedup_key(
        points: &[Point3<f64>],
        odom_xy: (f64, f64),
        point_step: u32,
        message_timestamp_ns: u64,
    ) -> FrameDedupKey {
        let (x_mm, y_mm, point_step) = Self::dedup_key(odom_xy, point_step);
        FrameDedupKey::Legacy((
            x_mm,
            y_mm,
            point_step,
            message_timestamp_ns,
            Self::points_fingerprint(points),
        ))
    }

    fn remember_dedup_key(&mut self, key: FrameDedupKey) {
        if self.dedup_history_capacity == 0 || !self.dedup_set.insert(key) {
            return;
        }
        self.dedup_order.push_back(key);
        while self.dedup_order.len() > self.dedup_history_capacity {
            if let Some(expired) = self.dedup_order.pop_front() {
                self.dedup_set.remove(&expired);
            }
        }
    }

    fn append_with_rolling_capacity(&mut self, points: &[Point3<f64>]) -> (usize, usize) {
        if points.is_empty() || self.bounded_capacity == 0 {
            return (0, points.len());
        }

        let accepted = points.len().min(self.bounded_capacity);
        let dropped_from_batch = points.len() - accepted;
        let retained_start = points.len() - accepted;
        let existing_budget = self.bounded_capacity - accepted;
        let evicted_existing = self.current_window.len().saturating_sub(existing_budget);
        self.current_window.drain(..evicted_existing);
        self.current_window
            .extend(points[retained_start..].iter().copied());
        let evicted = evicted_existing.saturating_add(dropped_from_batch);
        self.evicted_point_count = self
            .evicted_point_count
            .saturating_add(u64::try_from(evicted).unwrap_or(u64::MAX));
        self.generation = self.generation.saturating_add(1);
        (accepted, evicted)
    }

    /// 无消息时间戳的兼容入口。它不做重放去重，但仍执行滚动点预算。
    pub fn feed_unstamped(&mut self, points: &[Point3<f64>]) -> FeedStats {
        let (accepted, evicted) = self.append_with_rolling_capacity(points);
        FeedStats::accepted_with_eviction(
            accepted,
            evicted,
            self.current_window.len(),
            self.bounded_capacity,
            self.freshness,
        )
    }

    /// feed 一帧点云。
    ///
    /// 1. 拒绝比消息时间最高水位倒退超过容差的旧帧。
    /// 2. 过滤同时间戳、同内容、同位姿的消息重放。
    /// 3. 滚动淘汰最旧点，为新批次腾出预算。
    /// 4. 更新消息时间水位和有界 dedup 历史。
    ///
    /// 参数:
    /// - `points`:点云切片
    /// - `odom_xy`:(odom_x, odom_y) 元组,用于 dedup_key
    /// - `point_step`:ROS PointCloud2 `point_step` 字段,用于 dedup_key
    /// - `now_ns`:消息源时间戳(ns)；`0` 表示未提供，跳过 freshness 校验
    pub fn feed_pointcloud(
        &mut self,
        points: &[Point3<f64>],
        odom_xy: (f64, f64),
        point_step: u32,
        now_ns: u64,
    ) -> Result<FeedStats, SessionError> {
        self.feed_keyed_pointcloud(points, now_ns, || {
            Self::frame_dedup_key(points, odom_xy, point_step, now_ns)
        })
    }

    /// 使用生产方已准入的来源身份接收不可变点云。
    ///
    /// 本入口不计算点云 fingerprint。相同身份在不同接收时间仍属于重放；不同身份即使
    /// 点云相同也属于不同采集。调用方必须遵守 [`AdmittedFrameKey`] 的准入契约。
    ///
    /// `received_at_ns` 使用 session 原有的 freshness 水位（`0` 跳过校验）。freshness
    /// 先于去重，所以过旧重放仍可能返回 [`SessionError::StaleRevision`]。两种入口共享
    /// 有界重放历史和滚动点预算；历史淘汰或 reset 后，未过期身份可以再次接收。
    pub fn feed_admitted_pointcloud(
        &mut self,
        points: &[Point3<f64>],
        key: AdmittedFrameKey,
        received_at_ns: u64,
    ) -> Result<FeedStats, SessionError> {
        self.feed_keyed_pointcloud(points, received_at_ns, || FrameDedupKey::Admitted(key))
    }

    fn feed_keyed_pointcloud(
        &mut self,
        points: &[Point3<f64>],
        now_ns: u64,
        make_key: impl FnOnce() -> FrameDedupKey,
    ) -> Result<FeedStats, SessionError> {
        // freshness 衡量消息时间是否倒退，而不是相邻到达间隔。
        if now_ns > 0
            && self.freshness > now_ns
            && self.freshness - now_ns > self.stale_threshold_ns
        {
            return Err(SessionError::StaleRevision {
                last_seen_at_ns: self.freshness,
                now_ns,
                threshold_ns: self.stale_threshold_ns,
            });
        }

        // 延迟构造身份，保证 stale 输入不执行兼容入口的点云 fingerprint。
        let key = make_key();
        if self.dedup_set.contains(&key) {
            return Ok(FeedStats::dedup_only(
                points.len(),
                self.current_window.len(),
                self.bounded_capacity,
                self.freshness,
            ));
        }

        if self.bounded_capacity == 0 && !points.is_empty() {
            return Err(SessionError::ResourceExhausted {
                capacity: self.bounded_capacity,
                current_size: self.current_window.len(),
                attempted_points: points.len(),
            });
        }

        let (accepted, evicted) = self.append_with_rolling_capacity(points);
        self.remember_dedup_key(key);
        if now_ns > 0 {
            self.freshness = self.freshness.max(now_ns);
        }

        Ok(FeedStats::accepted_with_eviction(
            accepted,
            evicted,
            self.current_window.len(),
            self.bounded_capacity,
            self.freshness,
        ))
    }

    /// 更新 sensor_origin (odom pose 平移部分)。
    ///
    /// 不动 `freshness`;`feed_pose` 与 `feed_pointcloud` 是两条独立通道。
    pub fn feed_pose(&mut self, x: f64, y: f64, z: f64) {
        self.last_pose = [x, y, z];
    }

    /// publish 占位接口 —— 与 W2 §B.1 "本接口只回报 published 标志,真实发布由上层订阅"
    /// 对齐。
    ///
    /// 当前返回 `Ok(false)`:session 不内嵌发布通道;`MappingAdapter::build_and_publish`
    /// 才是真正的 publish 入口。Phase M #1 不在本接口重复实现 publish。
    pub fn build_and_publish(&mut self) -> Result<bool, SessionError> {
        // 当前实现无 publish 动作 —— real publish 在 MappingAdapter::build_and_publish
        // (基于 MappingSession 连续切片 + nuwax_mapping::CostMapper)。
        Ok(false)
    }

    /// snapshot 形态的诊断快照(对应 `MappingReadiness` 字段)。
    pub fn snapshot_stats(&self) -> SessionStats {
        SessionStats {
            capacity: self.bounded_capacity,
            current_size: self.current_window.len(),
            last_seen_at_ns: self.freshness,
            last_pose: self.last_pose,
            dedup_set_size: self.dedup_set.len(),
            generation: self.generation,
            evicted_point_count: self.evicted_point_count,
        }
    }

    /// 重置 session 状态(用于重新建图 / 地图 reset)。
    ///
    /// 清空 `current_window` / `accumulator` / `dedup_set`,重置 `freshness` / `last_pose`。
    /// `bounded_capacity` / `stale_threshold_ns` 保留(配置类字段)。
    pub fn reset_state(&mut self) {
        self.current_window.clear();
        self.dedup_set.clear();
        self.dedup_order.clear();
        self.freshness = 0;
        self.last_pose = [0.0, 0.0, 0.0];
        self.generation = 0;
        self.evicted_point_count = 0;
    }
}

impl Default for MappingSession {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// PointCloudAccumulator trait 实现 —— 兼容 nuwax-geometry-runtime 调用
// ============================================================================
//
// `MappingAdapter::accumulator: Arc<MappingSession>` 仍能让
// `<MappingSession as PointCloudAccumulator>::feed(&mut adapter.accumulator, points)`
// 编译通过;但 `Arc<T>` 的 `feed` 需要 `&mut Arc<T>` 然后 `Arc::get_mut` 取 `&mut T` ——
// 单 strong ref 路径可用,否则调用方必须改 `Arc::make_mut`(本 crate 后续若需要
// 可扩展 trait 默认实现)。
//
// 当前 trait 实现仅覆盖直接持有 `&mut MappingSession` 的场景
// (单测 / 未来 feed pipeline);`Arc<MappingSession>` 路径由 MappingAdapter::feed_pointcloud
// 显式调用 `Arc::make_mut` 解决,不在 trait 层暴露。

impl PointCloudAccumulator for MappingSession {
    fn feed(&mut self, points: &[Point3<f64>]) {
        let _ = self.feed_unstamped(points);
    }

    fn count(&self) -> usize {
        self.current_window.len()
    }
}

// ============================================================================
// 单元测试(≥3 RED test)
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(x: f64, y: f64, z: f64) -> Point3<f64> {
        Point3::new(x, y, z)
    }

    // ---- Test 1:capacity 是滚动预算，零容量才不可接收 ----

    #[test]
    fn feed_pointcloud_rolls_when_capacity_is_full() {
        let mut s = MappingSession::with_capacity(3);
        s.feed_pointcloud(&[pt(0.0, 0.0, 0.0)], (0.0, 0.0), 32, 1_000_000_000)
            .expect("first push ok");
        s.feed_pointcloud(&[pt(1.0, 0.0, 0.0)], (1.0, 0.0), 32, 1_001_000_000)
            .expect("second push ok");
        s.feed_pointcloud(&[pt(2.0, 0.0, 0.0)], (2.0, 0.0), 32, 1_002_000_000)
            .expect("third push ok");
        let stats = s
            .feed_pointcloud(&[pt(3.0, 0.0, 0.0)], (3.0, 0.0), 32, 1_003_000_000)
            .expect("full sessions must roll forward");
        assert_eq!(stats.accepted, 1);
        assert_eq!(stats.evicted, 1);
        assert_eq!(s.current_size(), 3);
        assert_eq!(s.clone_points()[0], pt(1.0, 0.0, 0.0));
    }

    #[test]
    fn zero_capacity_returns_resource_exhausted_without_mutation() {
        let mut s = MappingSession::with_capacity(0);
        let err = s
            .feed_pointcloud(&[pt(0.0, 0.0, 0.0)], (0.0, 0.0), 32, 1_000_000_000)
            .expect_err("zero capacity cannot retain a point");

        assert!(matches!(
            err,
            SessionError::ResourceExhausted {
                capacity: 0,
                current_size: 0,
                attempted_points: 1,
            }
        ));
        assert_eq!(s.current_size(), 0);
        assert_eq!(s.dedup_set_size(), 0);
        assert_eq!(s.last_seen_at_ns(), 0);
    }

    #[test]
    fn feed_pointcloud_accepts_batch_ending_exactly_at_capacity() {
        let mut s = MappingSession::with_capacity(3);
        s.feed_pointcloud(&[pt(0.0, 0.0, 0.0)], (0.0, 0.0), 32, 1_000_000_000)
            .expect("first point fits");
        let stats = s
            .feed_pointcloud(
                &[pt(1.0, 0.0, 0.0), pt(2.0, 0.0, 0.0)],
                (1.0, 0.0),
                32,
                1_001_000_000,
            )
            .expect("batch ending exactly at capacity must fit");

        assert_eq!(stats.accepted, 2);
        assert_eq!(stats.current_size, 3);
        assert_eq!(s.current_size(), s.capacity());
    }

    // ---- Test 2:只拒绝落后消息水位超过 200ms 的旧帧 ----

    #[test]
    fn feed_pointcloud_new_revision_after_long_gap_is_accepted() {
        let mut s = MappingSession::new();
        s.feed_pointcloud(&[pt(0.0, 0.0, 0.0)], (0.0, 0.0), 32, 1_000_000_000)
            .expect("first feed ok");
        let new_now = 1_000_000_000 + 300_000_000;
        let stats = s
            .feed_pointcloud(&[pt(0.5, 0.0, 0.0)], (0.5, 0.0), 48, new_now)
            .expect("a newer message cannot become stale merely because input is low-rate");
        assert_eq!(stats.accepted, 1);
        assert_eq!(s.last_seen_at_ns(), new_now);
    }

    #[test]
    fn stale_out_of_order_frame_does_not_move_freshness_watermark() {
        let mut s = MappingSession::new();
        let newest = 1_000_000_000;
        let stale = newest - DEFAULT_STALE_THRESHOLD_NS - 1;

        s.feed_pointcloud(&[pt(0.0, 0.0, 0.0)], (0.0, 0.0), 32, newest)
            .expect("first frame accepted");
        s.feed_pointcloud(&[pt(1.0, 0.0, 0.0)], (1.0, 0.0), 32, stale)
            .expect_err("old replay must be rejected");

        assert_eq!(s.last_seen_at_ns(), newest);
        assert_eq!(s.current_size(), 1);
    }

    // ---- Test 3:完整消息重放去重 ----

    #[test]
    fn feed_pointcloud_dedup_same_odom_step_keeps_first() {
        let mut s = MappingSession::new();
        let pts = [pt(1.0, 2.0, 3.0), pt(4.0, 5.0, 6.0)];
        let stats_a = s
            .feed_pointcloud(&pts, (1.0, 2.0), 32, 1_000_000_000)
            .expect("first feed ok");
        assert_eq!(stats_a.accepted, 2);
        assert_eq!(stats_a.dedup_skipped, 0);
        assert_eq!(stats_a.current_size, 2);

        // 同时间戳、同内容、同 odom + 同 point_step → 判定消息重放。
        let stats_b = s
            .feed_pointcloud(&pts, (1.0, 2.0), 32, 1_000_000_000)
            .expect("second feed should not error (dedup returns ok)");
        assert_eq!(stats_b.accepted, 0);
        assert_eq!(stats_b.dedup_skipped, 2);
        assert_eq!(stats_b.current_size, 2, "size 不变,保留首帧");
        assert!(!stats_b.published);
    }

    // ---- 附加:test dedup_key 量化精度 ----

    #[test]
    fn dedup_key_quantizes_odom_to_mm() {
        let k1 = MappingSession::dedup_key((1.0, 2.0), 32);
        let k2 = MappingSession::dedup_key((1.0001, 2.0001), 32);
        // 0.0001 m = 0.1 mm < 1mm → 量化后同一 key
        assert_eq!(k1, k2, "0.1mm 差异应量化为同 key");

        let k3 = MappingSession::dedup_key((1.002, 2.0), 32);
        assert_ne!(k1, k3, "2mm 差异应量化为不同 key");
    }

    // ---- 附加:test 同 odom 不同 point_step 不 dedup ----

    #[test]
    fn different_point_step_is_not_dedup() {
        let mut s = MappingSession::new();
        let pts = [pt(1.0, 2.0, 3.0)];
        s.feed_pointcloud(&pts, (1.0, 2.0), 32, 1_000_000_000)
            .expect("first feed ok");
        let stats_b = s
            .feed_pointcloud(&pts, (1.0, 2.0), 48, 1_001_000_000)
            .expect("different point_step 不 dedup");
        assert_eq!(stats_b.accepted, 1);
        assert_eq!(stats_b.current_size, 2);
    }

    // ---- 附加:test snapshot_stats ----

    #[test]
    fn snapshot_stats_reflects_current_state() {
        let mut s = MappingSession::with_capacity(100);
        s.feed_pose(1.5, 2.5, 0.0);
        s.feed_pointcloud(&[pt(0.0, 0.0, 0.0)], (0.0, 0.0), 32, 1_000_000_000)
            .expect("feed ok");
        let snap = s.snapshot_stats();
        assert_eq!(snap.capacity, 100);
        assert_eq!(snap.current_size, 1);
        assert_eq!(snap.last_seen_at_ns, 1_000_000_000);
        assert_eq!(snap.last_pose, [1.5, 2.5, 0.0]);
        assert_eq!(snap.dedup_set_size, 1);
    }

    // ---- 附加:test now_ns = 0 跳过 freshness 校验 ----

    #[test]
    fn now_ns_zero_skips_freshness_check() {
        let mut s = MappingSession::new();
        // 连续两次 feed 都用 now_ns=0,不会触发 stale 校验
        s.feed_pointcloud(&[pt(0.0, 0.0, 0.0)], (0.0, 0.0), 32, 0)
            .expect("first feed ok (now_ns=0 跳过 freshness)");
        s.feed_pointcloud(&[pt(1.0, 0.0, 0.0)], (1.0, 0.0), 32, 0)
            .expect("second feed ok (now_ns=0 跳过 freshness)");
        // 但 staleness 校验只在 `freshness > 0 && now_ns > 0` 时才生效,
        // 第一次 feed 不会更新 freshness(因为 now_ns=0),所以这里 current_size=2 但
        // freshness 仍 = 0
        assert_eq!(s.current_size(), 2);
        assert_eq!(s.last_seen_at_ns(), 0);
    }

    // ---- 附加:test clone_points 是 read-only ----

    #[test]
    fn clone_points_does_not_consume_session() {
        let mut s = MappingSession::new();
        s.feed_pointcloud(&[pt(1.0, 2.0, 3.0)], (0.0, 0.0), 32, 1_000_000_000)
            .expect("feed ok");
        let snap1 = s.clone_points();
        let snap2 = s.clone_points();
        assert_eq!(snap1.len(), 1);
        assert_eq!(snap2.len(), 1);
        assert_eq!(s.current_size(), 1, "clone 不消耗 session");
    }

    // ---- 附加:test PointCloudAccumulator trait 兼容 ----

    #[test]
    fn point_cloud_accumulator_trait_feed_works() {
        let mut s = MappingSession::with_capacity(100);
        // trait 默认入口:不传 freshness / dedup / point_step,等价于一次
        // "best-effort" push
        <MappingSession as PointCloudAccumulator>::feed(
            &mut s,
            &[pt(1.0, 2.0, 3.0), pt(4.0, 5.0, 6.0)],
        );
        assert_eq!(s.count(), 2);
        assert_eq!(s.current_size(), 2);
    }

    #[test]
    fn low_frequency_new_frames_are_not_stale() {
        let mut s = MappingSession::new();
        s.feed_pointcloud(&[pt(0.0, 0.0, 0.0)], (0.0, 0.0), 32, 1_000_000_000)
            .expect("first frame accepted");

        let stats = s
            .feed_pointcloud(&[pt(1.0, 0.0, 0.0)], (1.0, 0.0), 32, 1_500_000_000)
            .expect("a newer frame remains fresh even after a 500ms input gap");

        assert_eq!(stats.accepted, 1);
        assert_eq!(s.last_seen_at_ns(), 1_500_000_000);
    }

    #[test]
    fn out_of_order_message_older_than_tolerance_is_rejected() {
        let mut s = MappingSession::new();
        s.feed_pointcloud(&[pt(0.0, 0.0, 0.0)], (0.0, 0.0), 32, 1_000_000_000)
            .expect("first frame accepted");

        let err = s
            .feed_pointcloud(&[pt(1.0, 0.0, 0.0)], (1.0, 0.0), 32, 700_000_000)
            .expect_err("a frame 300ms behind the message-time watermark is stale");

        assert!(matches!(
            err,
            SessionError::StaleRevision {
                last_seen_at_ns: 1_000_000_000,
                now_ns: 700_000_000,
                threshold_ns: DEFAULT_STALE_THRESHOLD_NS,
            }
        ));
        assert_eq!(s.current_size(), 1);
        assert_eq!(s.last_seen_at_ns(), 1_000_000_000);
    }

    #[test]
    fn stationary_robot_accepts_new_samples_with_new_message_timestamps() {
        let mut s = MappingSession::new();
        let first = s
            .feed_pointcloud(&[pt(0.0, 0.0, 0.0)], (2.0, 3.0), 32, 1_000_000_000)
            .expect("first frame accepted");
        let second = s
            .feed_pointcloud(&[pt(1.0, 0.0, 0.0)], (2.0, 3.0), 32, 1_100_000_000)
            .expect("new sample at a stationary pose must not be deduplicated forever");

        assert_eq!(first.accepted, 1);
        assert_eq!(second.accepted, 1);
        assert_eq!(second.dedup_skipped, 0);
        assert_eq!(s.current_size(), 2);
    }

    #[test]
    fn dedup_history_is_bounded_and_expired_replays_can_be_reprocessed() {
        let mut s = MappingSession::new().with_dedup_history_capacity(2);
        let frame = [pt(0.0, 0.0, 0.0)];
        for (x, timestamp) in [(0.0, 1_000), (1.0, 2_000), (2.0, 3_000)] {
            s.feed_pointcloud(&frame, (x, 0.0), 32, timestamp)
                .expect("unique frame accepted");
        }
        assert_eq!(s.dedup_set_size(), 2);

        let replay = s
            .feed_pointcloud(&frame, (0.0, 0.0), 32, 1_000)
            .expect("expired replay identity may be processed again");
        assert_eq!(replay.accepted, 1);
        assert_eq!(s.dedup_set_size(), 2);
    }

    #[test]
    fn capacity_rolls_forward_and_keeps_newest_points() {
        let mut s = MappingSession::with_capacity(3);
        s.feed_pointcloud(
            &[pt(0.0, 0.0, 0.0), pt(1.0, 0.0, 0.0)],
            (0.0, 0.0),
            32,
            1_000_000_000,
        )
        .expect("initial batch accepted");

        let stats = s
            .feed_pointcloud(
                &[pt(2.0, 0.0, 0.0), pt(3.0, 0.0, 0.0)],
                (1.0, 0.0),
                32,
                1_100_000_000,
            )
            .expect("capacity pressure must evict old points instead of wedging the session");

        assert_eq!(stats.accepted, 2);
        assert_eq!(stats.current_size, 3);
        assert_eq!(
            s.clone_points(),
            vec![pt(1.0, 0.0, 0.0), pt(2.0, 0.0, 0.0), pt(3.0, 0.0, 0.0)]
        );
    }
}

#[cfg(test)]
include!("mapping_session_hash_baseline_tests.rs");

#[cfg(test)]
#[path = "mapping_session_admitted_dedup_tests.rs"]
mod admitted_dedup_tests;
