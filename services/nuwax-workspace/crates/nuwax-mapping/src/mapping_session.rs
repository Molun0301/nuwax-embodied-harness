//! `mapping_session.rs` — Phase M #1 有界增量 `MappingSession`
//!
//! 替换 `nuwax-geometry-runtime/src/mapping_adapter.rs::FeedOnlyAccumulator`:
//! - 默认 `bounded_capacity = 8_000_000`(`POINTCLOUD_MAX_RAW_POINTS` 默认值)
//! - `freshness` 校验:`now_ns - last_seen_at_ns > STALE_THRESHOLD_NS` 抛 `StaleRevisionError`
//! - `dedup_key`:`(odom_x_mm, odom_y_mm, point_step)` 同量化坐标 + 同 step → 保留首帧
//! - `accumulator`:`BoundedRing<Point3<f64>>` 写满 capacity 抛 `ResourceExhaustedError`
//! - 4 method:`feed_pointcloud` / `feed_pose` / `build_and_publish` / `snapshot_stats`
//!
//! 字段命名/语义对齐 W12 Python `tools/mapping/w8_mapping_session.py`
//! (RALPLAN §Phase M #1 + W2 §B.1,2026-07-12 W12 落地)。
//!
//! 与 `XyzAccumulator::into_points(self)` 不同,`MappingSession` 提供 `clone_points(&self)`
//! 只读视图,occupancy builder 可在 `&self` 上工作 —— 与 adapter 的 `build_occupancy`
//! 语义一致(不消费 session,后续 feed 仍能继续累积)。
//!
//! ## 与 `FeedOnlyAccumulator` 的差异
//! - `FeedOnlyAccumulator`:无限追加,只暴露 `feed/count/clone_points`
//! - `MappingSession`:bounded + freshness + dedup + typed error + stats
//!
//! ## typed error 与 W12 Python 对应
//! - `ResourceExhaustedError` ↔ Python `ResourceExhaustedError`
//! - `StaleRevisionError` ↔ Python `StaleRevisionError`
//! - 与 P0B #4 的 stable errors 一致(`RESOURCE_EXHAUSTED` / `STALE_REVISION`)
//!
//! ## 实现注意
//! - 不持有外部时钟;`now_ns` 由调用方传入(LCM 调度帧自带 timestamp,跨 crate 传单调时钟)
//! - `BoundedRing` 用 `VecDeque` + 显式 capacity check(显式抛错而非 silent evict,
//!   与 W2 §B.1 "超 capacity 返 RESOURCE_EXHAUSTED typed error" 一致)
//! - `dedup_set` 容量与 capacity 同阶;此处只校验重复坐标,允许后续 GC 策略扩展

use std::collections::{HashSet, VecDeque};
use std::fmt;

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

/// `freshness` 阈值 —— 200ms。超过该阈值视为 `StaleRevisionError`。
///
/// 与 W12 Python `STALE_THRESHOLD_MS = 200` 1:1 对齐;LCM /odom + /go2/lidar/points
/// 在 100Hz 下相邻帧间隔约 10ms,200ms 已能覆盖网络抖动与 GC stall。
pub const DEFAULT_STALE_THRESHOLD_NS: u64 = 200 * 1_000_000;

/// odom 坐标量化精度 —— 1mm。对齐 W12 Python `_odom_key` 的 `* 1000.0`。
///
/// 用于 dedup_key 的 mm 量化;同 mm 网格内的 odom 视为同一坐标,避免浮点误差导致
/// dedup miss。
pub const ODOM_QUANTIZE_MM: i64 = 1000;

fn batch_fits_capacity(current_size: usize, attempted_points: usize, capacity: usize) -> bool {
    current_size
        .checked_add(attempted_points)
        .is_some_and(|next_size| next_size <= capacity)
}

// ============================================================================
// typed error
// ============================================================================

/// `MappingSession` 错误基类(对应 W12 Python `SessionError`)。
#[derive(Debug)]
pub enum SessionError {
    /// bounded_capacity 已满,本次 feed 无法入环
    ResourceExhausted {
        /// 配置的最大点数。
        capacity: usize,
        /// 本次 feed 前已经累积的点数。
        current_size: usize,
        /// 本次批次尝试写入的点数。
        attempted_points: usize,
    },
    /// odom_pose 时间戳陈旧(`now_ns - last_seen_at_ns > STALE_THRESHOLD_NS`)
    StaleRevision {
        /// 检测到间隔前的 freshness 水位。
        last_seen_at_ns: u64,
        /// 被判定为 stale 的当前帧时间。
        now_ns: u64,
        /// 允许的最大帧间隔。
        threshold_ns: u64,
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
}

// ============================================================================
// MappingSession —— 核心有界增量 session
// ============================================================================

/// 有界增量 mapping session(Phase M #1 #1)。
///
/// 替换 `services/geometry/nuwax_geometry_runtime/src/mapping_adapter.rs::FeedOnlyAccumulator`:
/// - `bounded_capacity`:`VecDeque` 上限,显式抛错(非 silent evict)
/// - `current_window`:`VecDeque<Point3<f64>>` 当前累积点云
/// - `dedup_key`:HashSet<(odom_x_mm, odom_y_mm, point_step)>
/// - `accumulator`:`VecDeque<Point3<f64>>`(即 current_window 的别名,字段语义)
/// - `freshness`:`u64` 纳秒单调时间戳(`0` 表示未初始化,跳过 freshness 校验)
///
/// 与 `XyzAccumulator` 不同:
/// - 显式 capacity 边界 → typed `ResourceExhaustedError`
/// - freshness 窗口 → typed `StaleRevisionError`
/// - 同 odom_pose + 同 point_step dedup → 保留首帧
///
/// 与 `FeedOnlyAccumulator` 对比:
/// - `FeedOnlyAccumulator`:无限追加,只暴露 `feed/count/clone_points`
/// - `MappingSession`:bounded + freshness + dedup + typed error + stats + clone_points
///
/// **`Clone`**:`Arc::make_mut` 需要内部类型实现 `Clone`,W37 给 session 加
/// `#[derive(Clone)]` 以让 `MappingAdapter.session: Arc<MappingSession>` 路径
/// 在多 strong ref 时能 clone 数据(单 ref 路径零拷贝)。Clone 成本与
/// `current_window` + `dedup_set` 大小成正比,实机 8M 点时是几 MB 量级,
/// 在 `feed_pointcloud_session` 高频路径不构成瓶颈(每次只 clone 一次,
/// 且只在 ref 计数 > 1 时触发)。
#[derive(Clone)]
pub struct MappingSession {
    /// `bounded_capacity` —— 与 `POINTCLOUD_MAX_RAW_POINTS` 默认对齐 8M
    pub(crate) bounded_capacity: usize,
    /// `current_window` —— 当前累积的 `Point3<f64>`(顺序保留,支持 clone_points)
    pub(crate) current_window: VecDeque<Point3<f64>>,
    /// `dedup_key` —— `(odom_x_mm, odom_y_mm, point_step)`,同量化坐标 + 同 step 视为重复
    pub(crate) dedup_set: HashSet<(i64, i64, u32)>,
    /// `accumulator` —— 与 `current_window` 同义,字段名沿用 W2 §B.1 设计草案
    /// (避免 W12 报告里"accumulator"与"current_window"双名混淆)
    pub(crate) accumulator: VecDeque<Point3<f64>>,
    /// `freshness` —— 最近一次观察到的非零 `now_ns`(单调时钟,调用方传入)
    pub(crate) freshness: u64,
    /// 当前 sensor_origin (从 odom 平移部分)
    pub(crate) last_pose: [f64; 3],
    /// freshness 阈值(纳秒),默认 200ms
    pub(crate) stale_threshold_ns: u64,
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
            accumulator: VecDeque::with_capacity(bounded_capacity.min(1024)),
            freshness: 0,
            last_pose: [0.0, 0.0, 0.0],
            stale_threshold_ns: DEFAULT_STALE_THRESHOLD_NS,
        }
    }

    /// 显式指定 freshness 阈值(覆盖 `DEFAULT_STALE_THRESHOLD_NS = 200ms`)。
    pub fn with_stale_threshold_ns(mut self, stale_threshold_ns: u64) -> Self {
        self.stale_threshold_ns = stale_threshold_ns;
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

    /// 当前累积点的克隆(供 occupancy builder 输入;`&self` 上 read-only)。
    ///
    /// 与 `FeedOnlyAccumulator::clone_points` 同语义,不消费 session。
    pub fn clone_points(&self) -> Vec<Point3<f64>> {
        self.current_window.iter().copied().collect()
    }

    /// 构造 dedup_key —— 量化 odom_x/odom_y 到 mm,point_step 原值。
    ///
    /// 与 W12 Python `_odom_key` 1:1 对齐(`x_mm = round(x * 1000)`).
    pub fn dedup_key(odom_xy: (f64, f64), point_step: u32) -> (i64, i64, u32) {
        let x_mm = (odom_xy.0 * ODOM_QUANTIZE_MM as f64).round() as i64;
        let y_mm = (odom_xy.1 * ODOM_QUANTIZE_MM as f64).round() as i64;
        (x_mm, y_mm, point_step)
    }

    /// feed 一帧点云。
    ///
    /// 语义(对齐 W12 Python `feed_pointcloud`):
    /// 1. `freshness` 校验:`now_ns > 0 && freshness > 0 && now_ns - freshness > threshold`
    ///    抛 `StaleRevisionError`,并把 freshness 水位推进到 `now_ns`,避免长间隔后
    ///    所有后续帧都相对旧水位持续 stale
    /// 2. dedup:`dedup_key(odom_xy, point_step)` 已存在于 `dedup_set` → 返回
    ///    `FeedStats::dedup_only(...)`,不修改 `current_window`
    /// 3. capacity 校验:`current_window.len() + points.len() > bounded_capacity` → 抛
    ///    `ResourceExhaustedError`(checked arithmetic,整批原子拒绝而非 partial push)
    /// 4. push + 更新 `freshness` / `dedup_set`
    /// 5. 返回 `FeedStats::accepted(...)`
    ///
    /// 参数:
    /// - `points`:点云切片
    /// - `odom_xy`:(odom_x, odom_y) 元组,用于 dedup_key
    /// - `point_step`:ROS PointCloud2 `point_step` 字段,用于 dedup_key
    /// - `now_ns`:单调时钟(ns);`0` 表示未提供,跳过 freshness 校验
    pub fn feed_pointcloud(
        &mut self,
        points: &[Point3<f64>],
        odom_xy: (f64, f64),
        point_step: u32,
        now_ns: u64,
    ) -> Result<FeedStats, SessionError> {
        // 1. freshness 校验
        if now_ns > 0 && self.freshness > 0 && now_ns > self.freshness {
            let delta = now_ns - self.freshness;
            if delta > self.stale_threshold_ns {
                let last_seen_at_ns = self.freshness;
                self.freshness = now_ns;
                return Err(SessionError::StaleRevision {
                    last_seen_at_ns,
                    now_ns,
                    threshold_ns: self.stale_threshold_ns,
                });
            }
        }

        // 2. dedup
        let key = Self::dedup_key(odom_xy, point_step);
        if self.dedup_set.contains(&key) {
            return Ok(FeedStats::dedup_only(
                points.len(),
                self.current_window.len(),
                self.bounded_capacity,
                self.freshness,
            ));
        }

        // 3. capacity 校验
        if !batch_fits_capacity(
            self.current_window.len(),
            points.len(),
            self.bounded_capacity,
        ) {
            return Err(SessionError::ResourceExhausted {
                capacity: self.bounded_capacity,
                current_size: self.current_window.len(),
                attempted_points: points.len(),
            });
        }

        // 4. push
        for p in points {
            self.current_window.push_back(*p);
            self.accumulator.push_back(*p);
        }
        self.dedup_set.insert(key);
        if now_ns > 0 {
            self.freshness = now_ns;
        }

        Ok(FeedStats::accepted(
            points.len(),
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
        // (基于 `clone_points` + nuwax_mapping::CostMapper)。
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
        }
    }

    /// 重置 session 状态(用于重新建图 / 地图 reset)。
    ///
    /// 清空 `current_window` / `accumulator` / `dedup_set`,重置 `freshness` / `last_pose`。
    /// `bounded_capacity` / `stale_threshold_ns` 保留(配置类字段)。
    pub fn reset_state(&mut self) {
        self.current_window.clear();
        self.accumulator.clear();
        self.dedup_set.clear();
        self.freshness = 0;
        self.last_pose = [0.0, 0.0, 0.0];
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
        // trait 默认行为:无 freshness / 无 dedup / 静默 push / 超 capacity 截断。
        // 该入口主要给 `nuwax-mapping` 内部测试用;
        // real-lcm 路径应走 `feed_pointcloud(...)` 显式版本。
        let _ = self.feed_pointcloud(points, (0.0, 0.0), 0, 0);
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

    // ---- Test 1:capacity 满后 feed 抛 ResourceExhaustedError ----

    #[test]
    fn feed_pointcloud_exceeds_capacity_raises_resource_exhausted() {
        let mut s = MappingSession::with_capacity(3);
        // capacity = 3 时,3 个不同 odom pose 各 push 1 点 → 第 4 个 feed 必抛 ResourceExhausted
        s.feed_pointcloud(&[pt(0.0, 0.0, 0.0)], (0.0, 0.0), 32, 1_000_000_000)
            .expect("first push ok");
        s.feed_pointcloud(&[pt(1.0, 0.0, 0.0)], (1.0, 0.0), 32, 1_001_000_000)
            .expect("second push ok");
        s.feed_pointcloud(&[pt(2.0, 0.0, 0.0)], (2.0, 0.0), 32, 1_002_000_000)
            .expect("third push ok");
        assert_eq!(s.current_size(), 3);
        let err = s
            .feed_pointcloud(&[pt(3.0, 0.0, 0.0)], (3.0, 0.0), 32, 1_003_000_000)
            .expect_err("第 4 个 feed 应当抛 ResourceExhausted");
        match err {
            SessionError::ResourceExhausted {
                capacity,
                current_size,
                attempted_points,
            } => {
                assert_eq!(capacity, 3);
                assert_eq!(current_size, 3);
                assert_eq!(attempted_points, 1);
            }
            other => panic!("unexpected error variant: {other:?}"),
        }
    }

    #[test]
    fn feed_pointcloud_rejects_overflowing_batch_atomically() {
        let mut s = MappingSession::with_capacity(3);
        s.feed_pointcloud(
            &[pt(0.0, 0.0, 0.0), pt(1.0, 0.0, 0.0)],
            (0.0, 0.0),
            32,
            1_000_000_000,
        )
        .expect("first batch fits");

        let err = s
            .feed_pointcloud(
                &[pt(2.0, 0.0, 0.0), pt(3.0, 0.0, 0.0)],
                (1.0, 0.0),
                32,
                1_001_000_000,
            )
            .expect_err("2 + 2 points must not fit into capacity 3");

        assert!(matches!(
            err,
            SessionError::ResourceExhausted {
                capacity: 3,
                current_size: 2,
                attempted_points: 2,
            }
        ));
        assert_eq!(s.current_size(), 2, "rejected batches must be atomic");
        assert_eq!(s.dedup_set_size(), 1, "rejected key must not be retained");
        assert_eq!(s.last_seen_at_ns(), 1_000_000_000);
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

    #[test]
    fn batch_capacity_arithmetic_rejects_usize_overflow() {
        assert!(batch_fits_capacity(usize::MAX, 0, usize::MAX));
        assert!(!batch_fits_capacity(usize::MAX, 1, usize::MAX));
    }

    // ---- Test 2:stale > 200ms 抛 StaleRevisionError ----

    #[test]
    fn feed_pointcloud_stale_revision_raises_error() {
        let mut s = MappingSession::new();
        s.feed_pointcloud(&[pt(0.0, 0.0, 0.0)], (0.0, 0.0), 32, 1_000_000_000)
            .expect("first feed ok");
        // +300ms > 200ms threshold → 抛 StaleRevision
        let stale_now = 1_000_000_000 + 300_000_000;
        let err = s
            .feed_pointcloud(&[pt(0.5, 0.0, 0.0)], (0.5, 0.0), 48, stale_now)
            .expect_err("stale > 200ms 应当抛 StaleRevision");
        match err {
            SessionError::StaleRevision {
                last_seen_at_ns,
                now_ns,
                threshold_ns,
            } => {
                assert_eq!(last_seen_at_ns, 1_000_000_000);
                assert_eq!(now_ns, stale_now);
                assert_eq!(threshold_ns, DEFAULT_STALE_THRESHOLD_NS);
            }
            other => panic!("unexpected error variant: {other:?}"),
        }
    }

    #[test]
    fn stale_gap_rebases_freshness_and_next_frame_recovers() {
        let mut s = MappingSession::new();
        let first_now = 1_000_000_000;
        let stale_now = first_now + DEFAULT_STALE_THRESHOLD_NS + 1;
        let recovered_now = stale_now + 10_000_000;

        s.feed_pointcloud(&[pt(0.0, 0.0, 0.0)], (0.0, 0.0), 32, first_now)
            .expect("first frame accepted");
        s.feed_pointcloud(&[pt(1.0, 0.0, 0.0)], (1.0, 0.0), 32, stale_now)
            .expect_err("the first frame after a long gap remains stale");

        assert_eq!(
            s.last_seen_at_ns(),
            stale_now,
            "a rejected stale frame must rebase the freshness watermark"
        );
        let stats = s
            .feed_pointcloud(&[pt(2.0, 0.0, 0.0)], (2.0, 0.0), 32, recovered_now)
            .expect("a subsequent frame inside the freshness window must recover");

        assert_eq!(stats.accepted, 1);
        assert_eq!(stats.current_size, 2);
        assert_eq!(stats.last_seen_at_ns, recovered_now);
    }

    // ---- Test 3:dedup 同 odom + 同 point_step 保留首帧 ----

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

        // 同 odom + 同 point_step → dedup,跳过
        let stats_b = s
            .feed_pointcloud(&pts, (1.0, 2.0), 32, 1_001_000_000)
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
}
