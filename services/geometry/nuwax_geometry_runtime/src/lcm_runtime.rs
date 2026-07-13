// Phase 6 (codex-round3 R3-4)：build_handle_for_tests 是 #[cfg(test)] 内部
// 用的 helper,dead_code 是预期行为;needless_range_loop 与 too_many_arguments
// 是 typed_codec 长 payload 拼接的合理选择,不强行收敛。
#![allow(dead_code, clippy::needless_range_loop, clippy::too_many_arguments)]

//! LCM-first geometry runtime state loop。
//!
//! 该模块替换原 `ros2_bridge.rs`：不再订阅 ROS2 topic，而是通过
//! `nuwax_transport::LcmTransport` 订阅 LCM typed channel，将 typed frame
//! 累积到 `SharedState`，供 gRPC facade（`server.rs`）查询/订阅。
//!
//! ## backend 策略(2026-07-08 fake-lcm 物理删除)
//! - 唯一 backend:`LcmTransport::new_real(&lcm_url)`,基于 `nuwax-transport/real-lcm` feature。
//! - 通过 `NUWAX_LCM_BACKEND` 环境变量显式选择：仅接受 `real-lcm` / `real` / 空(默认 real-lcm);
//!   其它值(包括 `fake-lcm` / `fake`)→ 启动期 `process::exit(1)` + `LCM_BACKEND_UNSUPPORTED`。
//!   **严禁静默 fallback**(fake backend 已物理删除,v3 F2 决策)。
//! - 不允许回退 ROS2 / r2r / DDS / RMW。
//!
//! ## channel 命名约定
//! 与 `00_执行方案.md` §2.3 一致：
//! - `/odom#nav_msgs.Odometry`
//! - `/tf#tf2_msgs.TFMessage`
//! - `/map/occupancy#nav_msgs.OccupancyGrid`
//! - `/go2/lidar/points#sensor_msgs.PointCloud2`
//! - `/nav/state#nuwax_msgs.NavigationState`
//! - `/explore/frontiers#nuwax_msgs.FrontierList`
//!
//! 这里的 `nav_msgs/Odometry` 等名称是 LCM typed message 命名空间字符串，
//! 不引入 ROS2 package 依赖。

use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::Arc;

use nuwax_codec::lcm::schema;
use nuwax_protocol::{LcmMessageType, LcmTopic};
use nuwax_tf::{make_transform, TfTree, TransformStamped};
use nuwax_transport::LcmTransport;
use thiserror::Error;
use tokio::sync::{broadcast, RwLock};
use tracing::{error, info, warn};

use crate::mapping_adapter::{MappingAdapter, MappingAdapterConfig};
use crate::occupancy_shm::OccupancyGridShmWriter;
use crate::pointcloud::CloudMapShmWriter;

// Phase 4 (channel registry) + Phase 7 (readiness cache): 统一在这里导入,
// 避免每个函数 body 内部重复 `use crate::channel_registry::...`。
//
// 同时 `use crate::channel_registry;` 让 module 名进 scope,内部多处以
// `channel_registry::channel_for(...)` 限定调用,无需每次都写完整 `crate::` 路径。
use crate::channel_registry;
use crate::channel_registry::ChannelKey;

// ============================================================================
// 状态定义（与原 ros2_bridge.rs 同结构，便于 server.rs 不感知 backend 切换）
// ============================================================================

/// Phase 1 fail-fast 错误：上游 dimos `nuwax-lcm-sys`/`nuwax-lcm` 尚不存在
///（`nuwax-transport/real-lcm` feature 仍为空桩），所以 `real-lcm` backend
/// 启动期直接拒绝；2026-07-08 fake-lcm 物理删除后无任何 fallback。错误码 `LCM_BACKEND_UNAVAILABLE`
/// 必须在 stderr 中可见供 start 脚本与运维 grep 识别。
#[derive(Debug, Error)]
#[error(
    "LCM_BACKEND_UNAVAILABLE: NUWAX_LCM_BACKEND=real-lcm 不可用 —— 上游 dimos \
     nuwax-transport/real-lcm feature 仍为空桩 (Rust LCM binding 未实装 + \
     fingerprint 未切 MurmurHash64A)。详见 feedback_phase1.md。"
)]
pub struct RealLcmBackendUnavailable;

#[derive(Clone, Default)]
pub struct PoseState {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub qx: f64,
    pub qy: f64,
    pub qz: f64,
    pub qw: f64,
    pub frame_id: String,
    pub timestamp_ns: u64,
}

#[derive(Clone, Default)]
pub struct MapState {
    pub width: u32,
    pub height: u32,
    pub resolution: f32,
    pub origin_x: f64,
    pub origin_y: f64,
    pub frame_id: String,
    pub revision: u64,
    pub timestamp_ns: u64,
    pub data: Vec<i8>,
}

#[derive(Clone, Default)]
pub struct TransformEntry {
    pub parent: String,
    pub child: String,
    pub tx: f64,
    pub ty: f64,
    pub tz: f64,
    pub rx: f64,
    pub ry: f64,
    pub rz: f64,
    pub rw: f64,
    /// 该 TF 边的 stamp（纳秒，LCM 时间域）。0 表示无戳（latched/静态边），永不过期。
    pub stamp_ns: u64,
}

/// 动态 /tf 兼容镜像存活时长（纳秒）。stamp>0 且超过 TTL 的边在合成位姿前被裁剪，
/// 避免老版本 RTAB-Map/LCM 重启遗留陈旧 map→odom 污染 BFS。stamp==0 不过期。
/// 主查询路径使用 [`TfTree`] 的同一 10 秒历史窗口。
const TF_ENTRY_TTL_NS: u64 = 10_000_000_000;

#[derive(Clone, Default)]
pub struct NavState {
    pub status: String,
    pub message: String,
    pub goal_id: String,
    pub distance_remaining: f32,
    pub timestamp_ns: u64,
}

#[derive(Clone, Default)]
pub struct FrontierEntry {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub qx: f64,
    pub qy: f64,
    pub qz: f64,
    pub qw: f64,
    pub frame_id: String,
    pub size_cells: u32,
    pub heading: f32,
}

pub struct SharedState {
    pub pose: PoseState,
    pub map: MapState,
    /// Canonical 10-second TF history used by pose synthesis and gRPC frame inventory.
    pub tf_tree: Arc<TfTree>,
    /// Latest-edge compatibility mirror for older internal tests and diagnostics.
    pub transforms: Vec<TransformEntry>,
    pub nav_state: NavState,
    pub cloud_map_revision: u64,
    pub frontiers: Vec<FrontierEntry>,
    pub frontiers_revision: u64,
}

impl Default for SharedState {
    fn default() -> Self {
        Self {
            pose: PoseState::default(),
            map: MapState::default(),
            tf_tree: Arc::new(TfTree::new()),
            transforms: Vec::new(),
            nav_state: NavState {
                status: "IDLE".into(),
                message: "空闲".into(),
                ..Default::default()
            },
            cloud_map_revision: 0,
            frontiers: Vec::new(),
            frontiers_revision: 0,
        }
    }
}

impl SharedState {
    /// Return the latest transform for every child retained by the canonical TF tree.
    pub fn latest_transform_entries(&self) -> Vec<TransformEntry> {
        self.tf_tree
            .frames()
            .into_iter()
            .filter_map(|frame| self.tf_tree.lookup(&frame).ok())
            .filter_map(transform_to_entry)
            .collect()
    }
}

#[derive(Clone, Debug)]
pub struct CloudMapNotification {
    pub revision: u64,
    pub point_count: u32,
    pub voxel_size: f32,
}

/// RALPLAN §Phase M #3: occupancy grid revision 事件。
///
/// `mapping_adapter::MappingAdapter::build_and_publish` 在 commit 新 revision
/// 后构造本结构并通过 `broadcast::Sender<MapRevisionEvent>` 广播;
/// `server::subscribe_map_updates` 接收后替代旧的 500ms 轮询,真正事件驱动
/// 推送 `MapRevisionEvent` 给 Python 端 / WS-SSE 消费者。
///
/// 字段对齐 W29 `MapStreamEventBroker.StreamSnapshot.parsed` schema:
/// - `revision` —— occupancy grid 累计 revision(u64,单调递增)
/// - `data`     —— costmap 字节(`i8 as u8`,长度 = `width * height`)
/// - `size`     —— `data.len()`(u64,等价于 wire 上 rbuf.data_size)
/// - `schema_version` —— `NativeMapManifest` schema 版本(当前 v1)
/// - `checksum` —— SHA-256(`data`)的 hex(由 `map_checksum::sha256_hex` 算)
/// - `last_seen_ms` —— commit 时 `now_ns / 1_000_000`(i64,毫秒,给前端排序)
#[derive(Clone, Debug)]
pub struct MapRevisionEvent {
    pub revision: u64,
    pub width: u32,
    pub height: u32,
    pub resolution: f32,
    pub origin_x: f64,
    pub origin_y: f64,
    pub frame_id: String,
    pub timestamp_ns: u64,
    pub data: Vec<u8>,
    pub size: u64,
    pub schema_version: u32,
    pub checksum: String,
    pub last_seen_ms: i64,
}

impl MapRevisionEvent {
    pub fn from_map(map: &MapState) -> Self {
        let data: Vec<u8> = map.data.iter().map(|&value| value as u8).collect();
        Self {
            revision: map.revision,
            width: map.width,
            height: map.height,
            resolution: map.resolution,
            origin_x: map.origin_x,
            origin_y: map.origin_y,
            frame_id: map.frame_id.clone(),
            timestamp_ns: map.timestamp_ns,
            size: data.len() as u64,
            checksum: crate::map_checksum::sha256_hex(&data),
            last_seen_ms: (map.timestamp_ns / 1_000_000) as i64,
            schema_version: 1,
            data,
        }
    }

    pub fn to_map_state(&self) -> MapState {
        MapState {
            width: self.width,
            height: self.height,
            resolution: self.resolution,
            origin_x: self.origin_x,
            origin_y: self.origin_y,
            frame_id: self.frame_id.clone(),
            revision: self.revision,
            timestamp_ns: self.timestamp_ns,
            data: self.data.iter().map(|&value| value as i8).collect(),
        }
    }
}

/// 几何 runtime 句柄。`server.rs` 通过此句柄读 SharedState、订阅 cloud_map_tx
/// 以及通过 transport.publish 推送 odom / point cloud(real backend 直透 liblcm)。
#[derive(Clone)]
pub struct GeometryRuntimeHandle {
    pub state: Arc<RwLock<SharedState>>,
    pub cloud_map_tx: broadcast::Sender<CloudMapNotification>,
    pub nav_state_tx: broadcast::Sender<NavState>,
    pub frontiers_tx: broadcast::Sender<Vec<FrontierEntry>>,
    pub map_revision_tx: broadcast::Sender<MapRevisionEvent>,
    pub transport: LcmTransport,
    /// canonical LCM backend 名（HealthCheck / 日志 / diagnostics 用）：
    /// 2026-07-08 fake-lcm 物理删除后,字段值固定为 `real-lcm`;
    /// 用户请求 real-lcm 但 liblcm 不可用时由上层 `set transport_backend_name` 改写为
    /// `real-lcm-unavailable`。
    pub transport_backend_name: &'static str,
    /// backend 是否真正可用。当 `false` 时 server.rs 的 HealthCheck 必须
    /// 返回 `healthy=false` + `LCM_BACKEND_UNAVAILABLE` message；
    /// 该 flag 当前唯一触发源是 `LcmBackendKind::Real`（fail-fast 设计）。
    pub lcm_backend_available: bool,
    /// Phase 6 / Rework-9: Python `LcmMotionCommandBridge` 注入的 cmd_vel sink ready 信号。
    /// - `true`  — Python 侧 LcmMotionCommandBridge 已订阅 `/cmd_vel`,
    ///   watchdog 存活,sink 可写。
    /// - `false` — bridge 未启动 / sink 未注入 / watchdog 失败 /
    ///   providers 不实现 MotionControl / bridge.start() 抛异常。
    ///
    /// **生产 spawn 路径默认 `false`** —— 必须由 Python
    /// `LcmMotionCommandBridge.start()` 成功后通过 gateway 写回 `true`,
    /// 否则 `SetNavigationGoal` 在 `compute_readiness` 因
    /// `cmd_vel_sink_ready=false` 被拒绝(参见
    /// `REJECT_REASON_CMD_VEL_SINK_DOWN`)。test helper 路径保留默认 `true`
    /// 以兼容旧测试用例(显式 `set_cmd_vel_sink_ready(false)`
    /// 验证 readiness 拒绝分支;2026-07-08 fake-lcm 物理删除后 test helper
    /// 已改走 `LcmTransport::new_real`,见 `build_handle_for_tests`)。
    pub cmd_vel_sink_ready: Arc<std::sync::atomic::AtomicBool>,
    /// Phase 5: 探索 runtime 句柄(由 `spawn_geometry_runtime` 注入)。
    /// 负责 start / stop / state / diagnostics 四类 RPC。
    /// 持有 `Arc<tokio::sync::Mutex<...>>` 内部,允许 `Clone` 句柄
    /// 在 gRPC 任务间安全共享。
    pub explore: crate::explore_runtime::ExploreRuntimeHandle,
    /// Phase 4: 导航 runtime 状态(由 `spawn_geometry_runtime` / 单元测试
    /// 构造时初始化)。`server.rs` 通过它调 `set_goal` / `cancel` /
    /// `compute_readiness` 真实执行链。内部仅读 `state.pose.timestamp_ns`
    /// 与 `state.map.revision`,不持有 LCM 订阅线程。
    pub nav: crate::nav_runtime::NavRuntimeState,
    /// Phase 3: mapping adapter 句柄。
    /// - `None` —— backend 不可用 / LCM fail-fast 路径不构造 adapter
    /// - `Some` —— adapter 已注入,`mapping_ready_poll` 通过该字段判定
    ///   类型为 `Arc<parking_lot::Mutex<MappingAdapter>>`:
    ///   - `Arc` 允许多订阅任务共享
    ///   - `parking_lot::Mutex` 走同步路径,避免 async 持锁跨 await
    pub mapping_adapter: Option<Arc<parking_lot::Mutex<MappingAdapter>>>,
    /// Phase 3: occupancy grid SHM sidecar 写入器。
    ///
    /// - `None` —— backend 不可用 / LCM fail-fast 路径不构造
    /// - `Some` —— `get_occupancy_grid_shm` 走 SHM 直读
    pub occupancy_shm: Option<Arc<parking_lot::Mutex<OccupancyGridShmWriter>>>,
    /// 与 `occupancy_shm` 同时创建的权威路径；gRPC 返回的 `ShmRef.key`
    /// 必须使用该值，不能在请求时重新读取可能已变化的环境变量。
    pub occupancy_shm_path: Option<String>,
}

impl GeometryRuntimeHandle {
    pub fn subscribe_map_revisions(&self) -> broadcast::Receiver<MapRevisionEvent> {
        self.map_revision_tx.subscribe()
    }

    /// Phase 6: 读 cmd_vel sink ready flag。
    /// 由 Python `LcmMotionCommandBridge` 注入:
    /// - bridge 启动 + sink 已注入 + watchdog 存活 → `true`
    /// - bridge 未启动 / sink 未注入 / watchdog 失败 → `false`
    ///   2026-07-08 fake-lcm 物理删除后 test helper 改走 real backend,
    ///   默认 `true` 仍保留以兼容旧测试断言(显式 `set_cmd_vel_sink_ready(false)`
    ///   验证 readiness 拒绝分支)。
    pub fn cmd_vel_sink_ready(&self) -> bool {
        self.cmd_vel_sink_ready
            .load(std::sync::atomic::Ordering::Acquire)
    }

    /// Phase 6: Python LcmMotionCommandBridge 调用的 setter,
    /// 用于把外部 cmd_vel sink ready 状态写到 handle。
    pub fn set_cmd_vel_sink_ready(&self, ready: bool) {
        self.cmd_vel_sink_ready
            .store(ready, std::sync::atomic::Ordering::Release);
    }

    /// mapping 板块 readiness。
    /// - backend 不可用 → false
    /// - adapter 未构造 → false
    /// - adapter 已构造 + 有点云数据 → true
    /// - adapter 已构造 + 无数据 → false (等待 LCM 输入)
    pub fn mapping_ready_poll(&self) -> bool {
        if !self.lcm_backend_available {
            return false;
        }
        match &self.mapping_adapter {
            Some(a) => a.lock().has_data(),
            None => false,
        }
    }

    /// navigation 板块 readiness：必须有可用 backend，且最近收到真实
    /// stack 的 `/nav/state` heartbeat/state。
    pub fn nav_ready_poll(&self) -> bool {
        self.lcm_backend_available && self.nav.stack_is_live()
    }

    /// exploration 板块 readiness。
    /// Phase 7:从 `ExploreRuntimeHandle::readiness_cache` 取真实聚合
    /// (`backend_available && controller.is_active() &&
    /// frontier_source_ready && costmap_input_ready`),不再依赖
    /// "handle 存在即 true" 的假 ready。
    ///
    /// 该方法在 sync context(diagnostics 聚合路径)被调;走缓存快照避免
    /// await 内层 `tokio::Mutex`,cache 由 `ingest_costmap / start / stop
    /// / tick` 在变更状态后刷新,以及 `refresh_readiness_cache_with_backend`
    /// 在 backend 切换时刷新。
    pub fn explore_ready_poll(&self) -> bool {
        if !self.lcm_backend_available {
            return false;
        }
        // backend_available = lcm_backend_available,先用 sync 版本尝试刷新 cache;
        // 抢不到锁则退到 cached_readiness(降级,不假装 ready)。
        self.explore
            .refresh_readiness_cache_with_backend(self.lcm_backend_available);
        self.explore.cached_readiness().ready
    }
}

// ============================================================================
// 内部：tf2 坐标链合成（map→odom→base_link）。算法与原 ros2_bridge.rs 一致，
// 保证 server.rs 语义不因 backend 切换而漂移。
// ============================================================================

type Mat4 = [[f64; 4]; 4];

fn identity4() -> Mat4 {
    let mut m = [[0.0f64; 4]; 4];
    m[0][0] = 1.0;
    m[1][1] = 1.0;
    m[2][2] = 1.0;
    m[3][3] = 1.0;
    m
}

fn quat_to_rot(x: f64, y: f64, z: f64, w: f64) -> [[f64; 3]; 3] {
    let norm = (x * x + y * y + z * z + w * w).sqrt();
    if norm <= 1e-12 {
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    }
    let (x, y, z, w) = (x / norm, y / norm, z / norm, w / norm);
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

fn mat_mul(a: &Mat4, b: &Mat4) -> Mat4 {
    let mut o = [[0.0f64; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            let mut s = 0.0;
            for k in 0..4 {
                s += a[i][k] * b[k][j];
            }
            o[i][j] = s;
        }
    }
    o
}

fn entry_to_transform(entry: &TransformEntry) -> Option<TransformStamped> {
    let parent = normalize_frame(&entry.parent);
    let child = normalize_frame(&entry.child);
    let values = [
        entry.tx, entry.ty, entry.tz, entry.rx, entry.ry, entry.rz, entry.rw,
    ];
    let quaternion_norm_sq =
        entry.rx * entry.rx + entry.ry * entry.ry + entry.rz * entry.rz + entry.rw * entry.rw;
    if parent.is_empty()
        || child.is_empty()
        || !values.into_iter().all(f64::is_finite)
        || quaternion_norm_sq <= f64::EPSILON
    {
        return None;
    }
    let mut transform = make_transform(
        parent,
        child,
        [entry.tx, entry.ty, entry.tz],
        [entry.rx, entry.ry, entry.rz, entry.rw],
    );
    transform.stamp_secs = i64::try_from(entry.stamp_ns / 1_000_000_000).ok()?;
    transform.stamp_nanos = u32::try_from(entry.stamp_ns % 1_000_000_000).ok()?;
    Some(transform)
}

fn transform_to_entry(transform: TransformStamped) -> Option<TransformEntry> {
    let stamp_secs = u64::try_from(transform.stamp_secs).ok()?;
    let stamp_ns = stamp_secs
        .checked_mul(1_000_000_000)?
        .checked_add(u64::from(transform.stamp_nanos))?;
    let translation = transform.transform.translation.vector;
    let quaternion = transform.transform.rotation.quaternion();
    Some(TransformEntry {
        parent: transform.parent,
        child: transform.child,
        tx: translation.x,
        ty: translation.y,
        tz: translation.z,
        rx: quaternion.i,
        ry: quaternion.j,
        rz: quaternion.k,
        rw: quaternion.w,
        stamp_ns,
    })
}

fn entry_to_matrix(e: &TransformEntry) -> Mat4 {
    let r = quat_to_rot(e.rx, e.ry, e.rz, e.rw);
    let mut m = identity4();
    for i in 0..3 {
        for j in 0..3 {
            m[i][j] = r[i][j];
        }
    }
    m[0][3] = e.tx;
    m[1][3] = e.ty;
    m[2][3] = e.tz;
    m
}

fn rigid_inverse(m: &Mat4) -> Mat4 {
    let mut o = identity4();
    for (i, row) in o.iter_mut().enumerate().take(3) {
        for (j, col) in row.iter_mut().enumerate().take(3) {
            *col = m[j][i];
        }
    }
    for i in 0..3 {
        o[i][3] = -(o[i][0] * m[0][3] + o[i][1] * m[1][3] + o[i][2] * m[2][3]);
    }
    o
}

fn normalize_frame(f: &str) -> String {
    f.trim().trim_start_matches('/').to_string()
}

fn resolve_transform(transforms: &[TransformEntry], source: &str, target: &str) -> Option<Mat4> {
    let source = normalize_frame(source);
    let target = normalize_frame(target);
    if source.is_empty() || target.is_empty() {
        return None;
    }
    if source == target {
        return Some(identity4());
    }

    let mut adjacency: HashMap<String, Vec<(String, Mat4)>> = HashMap::new();
    for e in transforms {
        let parent = normalize_frame(&e.parent);
        let child = normalize_frame(&e.child);
        let m = entry_to_matrix(e); // child→parent
        adjacency
            .entry(child.clone())
            .or_default()
            .push((parent.clone(), m));
        adjacency
            .entry(parent)
            .or_default()
            .push((child, rigid_inverse(&m)));
    }

    let mut visited: HashSet<String> = HashSet::new();
    visited.insert(source.clone());
    let mut queue: VecDeque<(String, Mat4)> = VecDeque::new();
    queue.push_back((source, identity4()));
    while let Some((frame, mat)) = queue.pop_front() {
        if frame == target {
            return Some(mat);
        }
        if let Some(edges) = adjacency.get(&frame) {
            for (next, next_mat) in edges {
                if visited.contains(next) {
                    continue;
                }
                visited.insert(next.clone());
                queue.push_back((next.clone(), mat_mul(next_mat, &mat)));
            }
        }
    }
    None
}

fn mat_to_quat(m: &Mat4) -> (f64, f64, f64, f64) {
    let trace = m[0][0] + m[1][1] + m[2][2];
    if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        (
            (m[2][1] - m[1][2]) / s,
            (m[0][2] - m[2][0]) / s,
            (m[1][0] - m[0][1]) / s,
            0.25 * s,
        )
    } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
        let s = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
        (
            0.25 * s,
            (m[0][1] + m[1][0]) / s,
            (m[0][2] + m[2][0]) / s,
            (m[2][1] - m[1][2]) / s,
        )
    } else if m[1][1] > m[2][2] {
        let s = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
        (
            (m[0][1] + m[1][0]) / s,
            0.25 * s,
            (m[1][2] + m[2][1]) / s,
            (m[0][2] - m[2][0]) / s,
        )
    } else {
        let s = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
        (
            (m[0][2] + m[2][0]) / s,
            (m[1][2] + m[2][1]) / s,
            0.25 * s,
            (m[0][1] - m[1][0]) / s,
        )
    }
}

/// (x, y, z, qx, qy, qz, qw, normalized_base_frame)
type PoseWithFrame = (f64, f64, f64, f64, f64, f64, f64, String);

fn resolve_map_pose(
    transforms: &[TransformEntry],
    map_frame: &str,
    base_frames: &[&str],
) -> Option<PoseWithFrame> {
    for base in base_frames {
        if let Some(m) = resolve_transform(transforms, base, map_frame) {
            let (qx, qy, qz, qw) = mat_to_quat(&m);
            return Some((
                m[0][3],
                m[1][3],
                m[2][3],
                qx,
                qy,
                qz,
                qw,
                normalize_frame(base),
            ));
        }
    }
    None
}

fn resolve_map_pose_tree(
    tree: &TfTree,
    map_frame: &str,
    base_frames: &[&str],
) -> Option<PoseWithFrame> {
    let map_frame = normalize_frame(map_frame);
    for base in base_frames {
        let base = normalize_frame(base);
        let Ok(transform) = tree.chain(&base, &map_frame) else {
            continue;
        };
        let translation = transform.translation.vector;
        let quaternion = transform.rotation.quaternion();
        return Some((
            translation.x,
            translation.y,
            translation.z,
            quaternion.i,
            quaternion.j,
            quaternion.k,
            quaternion.w,
            base,
        ));
    }
    None
}

// ============================================================================
// LCM payload 解析辅助（generated-schema 解码）
// ============================================================================
//
// `nuwax-transport` 已验证并剥离 generated fingerprint，所以这些函数接收
// `_encode_one()` body。为复用 `nuwax-codec` 的 top-level generated decoder，
// 仅在进程内重新拼回对应 fingerprint；不再维护第二套 little-endian layout。

fn decode_generated<T>(
    fingerprint: [u8; 8],
    body: &[u8],
    decode: impl FnOnce(&[u8]) -> Result<T, nuwax_codec::CodecError>,
) -> Option<T> {
    let mut encoded = Vec::with_capacity(fingerprint.len() + body.len());
    encoded.extend_from_slice(&fingerprint);
    encoded.extend_from_slice(body);
    decode(&encoded).ok()
}

fn header_stamp_ns(header: &schema::Header) -> Option<u64> {
    if header.stamp_sec < 0 || !(0..1_000_000_000).contains(&header.stamp_nsec) {
        return None;
    }
    (header.stamp_sec as u64)
        .checked_mul(1_000_000_000)?
        .checked_add(header.stamp_nsec as u64)
}

fn parse_odom_payload(payload: &[u8]) -> Option<PoseState> {
    crate::typed_codec::decode_odometry_payload(payload)
}

/// 单次 /map/occupancy frame 允许的 width/height 上限(防御性)。
///
/// 2000×2000 = 4M cells = 4 MiB(0.05m/cell 覆盖 100×100 m),已是物理合理
/// 上限。超过则上游 occupancy 帧字段异常,直接拒绝。width * height
/// 决定后续 `data` 字节数,这一项炸掉会直接 175 GB。
const OCCUPANCY_MAX_DIMENSION: u32 = 8192;

/// 单次 /explore/frontiers frame 允许的最大 frontier 条目数(防御性上限)。
///
/// 与 `TF_MAX_ENTRIES_PER_FRAME` 同源:防止异常帧的 count 字段触发
/// Vec::with_capacity 申请上百 GB。
const FRONTIERS_MAX_ENTRIES_PER_FRAME: usize = 16384;

/// 单次 /tf frame 允许的最大 transform 条目数(防御性上限)。
///
/// 理论合法值取决于上游 SLAM 输出,但物理合理范围是 < 1e4;一旦上游帧
/// 因 uninit memory / 协议异常把 count 字段解析成大数,`Vec::with_capacity`
/// 会触发数百 GB 的 mmap/alloc,直接炸掉 7 GiB 内存的 Jetson/Go2 实机。
/// 这里把上限钉死在 65536,既覆盖任何现实 SLAM 节点,又让异常帧
/// 仅触发 warn + 跳过(不阻塞其它 channel)。
const TF_MAX_ENTRIES_PER_FRAME: usize = 65536;

fn parse_tf_payload(payload: &[u8]) -> Option<Vec<TransformEntry>> {
    let transforms = crate::typed_codec::decode_tf_payload(payload)?;
    if transforms.len() > TF_MAX_ENTRIES_PER_FRAME {
        warn!(
            "/tf payload count={} 超过 sanity cap={};按异常帧跳过",
            transforms.len(),
            TF_MAX_ENTRIES_PER_FRAME
        );
        return None;
    }
    Some(transforms)
}

fn parse_occupancy_payload(payload: &[u8]) -> Option<MapState> {
    let message = decode_generated(
        schema::OCCUPANCY_GRID_FINGERPRINT,
        payload,
        schema::decode_occupancy_grid,
    )?;
    let width = u32::try_from(message.info.width).ok()?;
    let height = u32::try_from(message.info.height).ok()?;
    // 防御性 sanity cap:见 OCCUPANCY_MAX_DIMENSION 注释。
    if width == 0
        || height == 0
        || width > OCCUPANCY_MAX_DIMENSION
        || height > OCCUPANCY_MAX_DIMENSION
    {
        warn!(
            "/map/occupancy payload width={} height={} 超过 sanity cap={};按异常帧跳过",
            width, height, OCCUPANCY_MAX_DIMENSION
        );
        return None;
    }
    // data 必须等于 width*height;否则说明上游帧截断/异常,拒绝
    // 防止后续 Vec<i8> 申请非预期容量。
    let expected_len = (width as usize).saturating_mul(height as usize);
    if message.data.len() != expected_len {
        warn!(
            "/map/occupancy payload data_len={} 与 width*height={} 不一致;按异常帧跳过",
            message.data.len(),
            expected_len
        );
        return None;
    }
    Some(MapState {
        width,
        height,
        resolution: message.info.resolution,
        origin_x: message.info.origin.position[0],
        origin_y: message.info.origin.position[1],
        frame_id: message.header.frame_id.clone(),
        revision: 0,
        timestamp_ns: header_stamp_ns(&message.header)?,
        data: message.data,
    })
}

struct ParsedPointCloudPayload {
    data: Vec<u8>,
    point_step: u32,
    frame_id: String,
    timestamp_ns: u64,
}

fn parse_pointcloud_payload(payload: &[u8]) -> Option<ParsedPointCloudPayload> {
    let message = decode_generated(
        schema::POINT_CLOUD2_FINGERPRINT,
        payload,
        schema::decode_point_cloud2,
    )?;
    let timestamp_ns = header_stamp_ns(&message.header)?;
    let frame_id = message.header.frame_id.clone();
    let point_step = u32::try_from(message.point_step).ok()?;
    // 防御性 sanity cap:见 POINTCLOUD_MAX_POINT_STEP_BYTES 注释。
    // point_step 来自 untrusted LCM frame,sender 可伪造极端值(0/1/u32::MAX)
    // 让后续 `bytes.len() / point_step` 推算 raw_count 时产生巨大值,触发
    // `Vec::with_capacity(raw_count)` 申请数百 GB alloc(2026-07-08 实机
    // 187GB abort 模式)。
    if !(12..=crate::pointcloud::POINTCLOUD_MAX_POINT_STEP_BYTES).contains(&point_step) {
        warn!(
            "/go2/lidar/points payload point_step={} 超过 sanity cap [12, {}];按异常帧跳过",
            point_step,
            crate::pointcloud::POINTCLOUD_MAX_POINT_STEP_BYTES
        );
        return None;
    }
    // 防御性 sanity cap:payload 总长 4 MB 上限,远超物理合理点云单帧
    // (Go2 单帧 ~30k 点 ≈ 360 KB);超过则 upstream LCM 帧异常,拒绝。
    if message.data.len() > 4 * 1024 * 1024 {
        warn!(
            "/go2/lidar/points payload len={} 超过 4 MiB sanity cap;按异常帧跳过",
            message.data.len()
        );
        return None;
    }
    if message.is_bigendian {
        warn!("/go2/lidar/points big-endian PointCloud2 暂不支持;按异常帧跳过");
        return None;
    }
    let (height, width, row_step) = (
        usize::try_from(message.height).ok()?,
        usize::try_from(message.width).ok()?,
        usize::try_from(message.row_step).ok()?,
    );
    let packed_row = width.checked_mul(point_step as usize)?;
    if height == 0
        || width == 0
        || row_step != packed_row
        || message.data.len() != height.checked_mul(row_step)?
    {
        warn!("/go2/lidar/points PointCloud2 dimensions/data length inconsistent;按异常帧跳过");
        return None;
    }
    let xyz_layout = [("x", 0_i32), ("y", 4_i32), ("z", 8_i32)];
    if xyz_layout.iter().any(|(name, offset)| {
        !message.fields.iter().any(|field| {
            field.name == *name
                && field.offset == *offset
                && field.datatype == 7
                && field.count == 1
        })
    }) {
        warn!("/go2/lidar/points PointCloud2 缺少 packed float32 x/y/z 字段;按异常帧跳过");
        return None;
    }
    Some(ParsedPointCloudPayload {
        data: message.data,
        point_step,
        frame_id,
        timestamp_ns,
    })
}

// ============================================================================
// 启动：构造 LcmTransport (real-lcm) + SharedState + 各订阅 task
// ============================================================================

/// LCM backend 选择。2026-07-08 fake-lcm 物理删除后,只剩 `Real`。
/// `NUWAX_LCM_BACKEND` 仅接受 `real-lcm` / `real` / 空(默认 real);
/// 其它值(fake-lcm / fake / 未知)→ 立即 `process::exit(1)` + `LCM_BACKEND_UNSUPPORTED`。
///
/// Phase 7 readiness 字段仍由 `mapping_adapter::MappingReadiness`
/// + `nav_runtime::NavigationReadinessDetail`
/// + `explore_runtime::ExplorationReadinessDetail` 三个 layer 独立暴露
///   counters + active backend(backend 字段值固定为 "real-lcm")。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LcmBackendKind {
    /// 真实 liblcm backend(2026-07-08 唯一 backend)
    Real,
}

impl LcmBackendKind {
    fn from_env() -> Self {
        // 2026-07-08 fake-lcm 根除(v3 F2 决策):
        // - canonical = `real-lcm`(default),`real` 是迁移 alias
        // - `fake-lcm` / `fake` / 其它非空值 → 立即 exit(1)+ LCM_BACKEND_UNSUPPORTED
        //   严禁静默 fallback(fake backend 已物理删除)
        // - 空值 → 默认 `real-lcm`
        let raw = std::env::var("NUWAX_LCM_BACKEND").unwrap_or_default();
        let normalized = raw.trim().to_ascii_lowercase();
        match normalized.as_str() {
            "" => {
                tracing::info!("NUWAX_LCM_BACKEND 未设置,默认 real-lcm");
                Self::Real
            }
            "real-lcm" => Self::Real,
            "real" => {
                tracing::warn!(
                    "NUWAX_LCM_BACKEND=real 是迁移 alias;已归一化为 real-lcm。后续请直接传 canonical 名。"
                );
                Self::Real
            }
            "fake-lcm" | "fake" => {
                eprintln!(
                    "LCM_BACKEND_UNSUPPORTED: NUWAX_LCM_BACKEND={:?} 已被 2026-07-08 移除。\
                     fake-lcm backend 已物理删除(v3 F2 决策);本 binary 仅支持 real-lcm。退出。",
                    normalized
                );
                std::process::exit(1);
            }
            other => {
                eprintln!(
                    "LCM_BACKEND_UNSUPPORTED: NUWAX_LCM_BACKEND={:?} 已被 2026-07-08 移除。\
                     canonical 取值只允许 real-lcm / real / 空;本 binary 仅支持 real-lcm。退出。",
                    other
                );
                std::process::exit(1);
            }
        }
    }
}

/// 读 `NUWAX_REAL_LCM_REQUIRED`：true/false/1/0/yes/no/on/off (大小写不敏感)。
///
/// 2026-07-08 fake-lcm 物理删除后(v3 F2 决策):
/// - `true` + liblcm 不可用 → 启动期进程直接 exit 1 + `LCM_BACKEND_UNAVAILABLE`
///   (不允许任何 fallback —— fake backend 已物理删除)。
/// - `true` + liblcm OK → 启动 real-lcm backend 正常。
/// - 默认 false；与 `core/settings.py::REAL_LCM_REQUIRED` 语义保持对齐。
pub fn real_lcm_required_from_env() -> bool {
    let raw = std::env::var("NUWAX_REAL_LCM_REQUIRED").unwrap_or_default();
    matches!(
        raw.trim().to_ascii_lowercase().as_str(),
        "1" | "true" | "yes" | "on"
    )
}

/// `LcmTransport::new_real` 的启动结果分类。
///
/// `NUWAX_REAL_LCM_REQUIRED` 只决定创建失败后的退出策略，不能跳过真实 backend
/// 的创建尝试。将这一步保留为纯逻辑，既避免 required=true 被误写成前置退出，
/// 也便于在不终止测试进程的情况下锁定生产启动语义。
#[derive(Debug, PartialEq, Eq)]
enum RealLcmStartupError<E> {
    Required(E),
    Optional(E),
}

fn attempt_real_lcm_startup<T, E>(
    real_lcm_required: bool,
    create: impl FnOnce() -> Result<T, E>,
) -> Result<T, RealLcmStartupError<E>> {
    match create() {
        Ok(transport) => Ok(transport),
        Err(error) if real_lcm_required => Err(RealLcmStartupError::Required(error)),
        Err(error) => Err(RealLcmStartupError::Optional(error)),
    }
}

fn parse_expected_typed_channel(
    channel: &str,
    expected_type: &str,
) -> Result<(LcmTopic, LcmMessageType), String> {
    let (topic, suffix) = crate::nav_runtime::parse_channel_with_suffix(channel)?;
    let suffix = suffix.ok_or_else(|| format!("typed channel {channel} 缺少 #message suffix"))?;
    let actual = LcmMessageType::new(&suffix)
        .map_err(|error| format!("typed channel {channel} message type 无效: {error}"))?;
    let expected = LcmMessageType::new(expected_type)
        .map_err(|error| format!("expected message type {expected_type} 无效: {error}"))?;
    if actual != expected {
        return Err(format!(
            "typed channel {channel} 的 message type={}，期望 {}",
            actual.fqn(),
            expected.fqn()
        ));
    }
    Ok((topic, actual))
}

fn binding_for_key(
    key: ChannelKey,
    expected_type: &str,
) -> Result<(LcmTopic, LcmMessageType), String> {
    let (topic, suffix) = channel_registry::channel_for_with_suffix(key);
    let suffix = suffix.ok_or_else(|| format!("channel_for_with_suffix({key:?}) 缺少 suffix"))?;
    parse_expected_typed_channel(&format!("{}#{suffix}", topic.channel()), expected_type)
}

/// 启动 geometry runtime 异步循环（LCM-first）。
///
/// 2026-07-08 fake-lcm 物理删除后:backend 固定为 `LcmBackendKind::Real`,
/// 启动期调 `LcmTransport::new_real(&lcm_url)`;若 liblcm 不可用 / 启动失败
/// → fail-fast 返回 [`RealLcmBackendUnavailable`] 或 `process::exit(1)`
/// (当 `NUWAX_REAL_LCM_REQUIRED=true` 时)。无 fake fallback。
///
/// # Args
///
/// * `pose_channel` — Phase 1 新增；`geometry_msgs/PoseStamped` typed channel
/// * `odometry_channel` — Phase 1 新增；`nav_msgs/Odometry` typed channel
/// * `cmd_vel_channel` — Phase 1 新增；`geometry_msgs/Twist` typed channel
/// * `nav_cancel_channel` — Phase 1 新增；`nuwax_msgs/CancelNavigation` typed channel
/// * 其余形参来自 Phase 0 默认 channel registry
pub fn spawn_geometry_runtime(
    occupancy_channel: &str,
    tf_channel: &str,
    cloud_map_channel: Option<&str>,
    nav_state_channel: Option<&str>,
    frontiers_channel: Option<&str>,
    pose_channel: Option<&str>,
    odometry_channel: Option<&str>,
    cmd_vel_channel: Option<&str>,
    nav_cancel_channel: Option<&str>,
) -> Result<GeometryRuntimeHandle, RealLcmBackendUnavailable> {
    let (cloud_map_tx, _) = broadcast::channel::<CloudMapNotification>(4);
    let (nav_state_tx, _) = broadcast::channel::<NavState>(8);
    let (frontiers_tx, _) = broadcast::channel::<Vec<FrontierEntry>>(4);
    let (map_revision_tx, _) = broadcast::channel::<MapRevisionEvent>(16);

    let state = Arc::new(RwLock::new(SharedState::default()));
    let nav_runtime = crate::nav_runtime::NavRuntimeState::from_env();

    // 按 NUWAX_LCM_BACKEND 选择 backend。
    // 2026-07-08 fake-lcm 物理删除后,`LcmBackendKind::from_env()` 仅返回 `Real`。
    // Phase 5 (NUWAX_REAL_LCM_REQUIRED=true) 保持:启动失败时进程直接
    // `process::exit(1)`,不允许静默 fallback(fake backend 已物理删除)。
    let real_lcm_required = real_lcm_required_from_env();
    let (transport, backend_name, backend_available): (LcmTransport, &'static str, bool) = {
        let lcm_url = std::env::var("NUWAX_LCM_URL")
            .unwrap_or_else(|_| "udpm://239.255.76.67:7667?ttl=1".to_owned());
        // real-lcm feature gate —— 二选一:
        //   1. binary --features real-lcm-bridge → 真起 liblcm
        //   2. binary 未启用 real-lcm-bridge feature → 保留 Phase 2 行为 (return Err)
        //      2026-07-08 fake-lcm 物理删除后,该分支仅由 CI 误配触发,本意即"无 backend 可用"。
        #[cfg(feature = "real-lcm-bridge")]
        {
            match attempt_real_lcm_startup(real_lcm_required, || LcmTransport::new_real(&lcm_url)) {
                Ok(t) => {
                    info!("LCM backend 选择: real-lcm (provider={})", lcm_url);
                    (t, "real-lcm", true)
                }
                Err(RealLcmStartupError::Required(e)) => {
                    eprintln!(
                        "LCM_BACKEND_UNAVAILABLE: LcmTransport::new_real({}) 失败: {} \
                         —— NUWAX_REAL_LCM_REQUIRED=true，按生产意图立即硬退出 \
                         (exit code = 1)。请确认 liblcm-dev 已安装 \
                         (apt-get install liblcm-dev) 且 provider URL 合法。",
                        lcm_url, e
                    );
                    std::process::exit(1);
                }
                Err(RealLcmStartupError::Optional(e)) => {
                    // 2026-07-08 fake-lcm 物理删除后:无 fake fallback,直接 fail-fast。
                    eprintln!(
                        "LCM_BACKEND_UNAVAILABLE: LcmTransport::new_real({}) 失败: {} \
                         —— 请确认 liblcm-dev 已安装 (apt-get install liblcm-dev) \
                         且 provider URL 合法。",
                        lcm_url, e
                    );
                    return Err(RealLcmBackendUnavailable);
                }
            }
        }
        #[cfg(not(feature = "real-lcm-bridge"))]
        {
            let _ = lcm_url;
            return Err(RealLcmBackendUnavailable);
        }
    };
    info!(
        "nuwax-geometry-runtime 启动：LCM backend = {} (available={})",
        backend_name, backend_available
    );

    // Phase 4 startup health wire-up:在启动日志中暴露 backend 选择,供
    // start 脚本 / smoke test / debug_panel 观测;2026-07-08 fake-lcm
    // 物理删除后,字段值固定为 `real-lcm`。
    info!(
        "LCM backend status (Phase 4 startup wire-up): lcm_backend={} available={}",
        backend_name, backend_available
    );

    let (occ_topic, occ_msg_type) =
        match parse_expected_typed_channel(occupancy_channel, "nav_msgs.OccupancyGrid") {
            Ok(binding) => binding,
            Err(e) => {
                error!("lcm_runtime::spawn: occupancy channel 解析失败: {}", e);
                return Err(RealLcmBackendUnavailable);
            }
        };
    let (tf_topic, tf_msg_type) =
        match parse_expected_typed_channel(tf_channel, "tf2_msgs.TFMessage") {
            Ok(binding) => binding,
            Err(e) => {
                error!("lcm_runtime::spawn: tf channel 解析失败: {}", e);
                return Err(RealLcmBackendUnavailable);
            }
        };
    let cm_binding = match cloud_map_channel {
        Some(c) => match parse_expected_typed_channel(c, "sensor_msgs.PointCloud2") {
            Ok(binding) => Some(binding),
            Err(e) => {
                warn!(
                    "lcm_runtime::spawn: cloud_map channel 解析失败 ({}): 降级无 cloud_map 模式",
                    e
                );
                None
            }
        },
        None => None,
    };
    let nav_binding = match nav_state_channel {
        Some(c) => match parse_expected_typed_channel(c, "nuwax_msgs.NavigationState") {
            Ok(binding) => Some(binding),
            Err(e) => {
                warn!(
                    "lcm_runtime::spawn: nav_state channel 解析失败 ({}): 降级无 nav 订阅",
                    e
                );
                None
            }
        },
        None => None,
    };
    let fr_binding = match frontiers_channel {
        Some(c) => match parse_expected_typed_channel(c, "nuwax_msgs.FrontierList") {
            Ok(binding) => Some(binding),
            Err(e) => {
                warn!(
                    "lcm_runtime::spawn: frontiers channel 解析失败 ({}): 降级无 frontier 订阅",
                    e
                );
                None
            }
        },
        None => None,
    };
    // Phase 1 channel registry 补齐:解析 4 类 typed channel。
    // Phase 4: 改走 `parse_channel_with_suffix`,强制 typed suffix;不再 untyped 兜底。
    let pose_binding = match pose_channel {
        Some(c) => match parse_expected_typed_channel(c, "geometry_msgs.PoseStamped") {
            Ok(binding) => Some(binding),
            Err(e) => {
                warn!(
                    "lcm_runtime::spawn: pose channel 解析失败 ({}): 降级无 /pose_stamped 订阅",
                    e
                );
                None
            }
        },
        None => None,
    };
    let odom_binding = match odometry_channel {
        Some(c) => match parse_expected_typed_channel(c, "nav_msgs.Odometry") {
            Ok(binding) => Some(binding),
            Err(e) => {
                warn!(
                    "lcm_runtime::spawn: odom channel 解析失败 ({}): 降级无 /odom 订阅",
                    e
                );
                None
            }
        },
        None => None,
    };
    let cmd_vel_binding = match cmd_vel_channel {
        Some(c) => match parse_expected_typed_channel(c, "geometry_msgs.Twist") {
            Ok(binding) => Some(binding),
            Err(e) => {
                warn!(
                    "lcm_runtime::spawn: cmd_vel channel 解析失败 ({}): 降级无 /cmd_vel 订阅",
                    e
                );
                None
            }
        },
        None => None,
    };
    let nav_cancel_binding = match nav_cancel_channel {
        Some(c) => match parse_expected_typed_channel(c, "nuwax_msgs.CancelNavigation") {
            Ok(binding) => Some(binding),
            Err(e) => {
                warn!(
                    "lcm_runtime::spawn: nav_cancel channel 解析失败 ({}): 降级无 /nav/cancel 订阅",
                    e
                );
                None
            }
        },
        None => None,
    };

    // Phase 1:打印 channel registry 让运维和 healthcheck 都能看到本进程
    // 注册的全部 typed channel;Phase 2+ 将在此基础上挂接订阅任务。
    info!(
        "LCM channel registry: occupancy={} tf={} cloud_map={} nav_state={} frontiers={} pose={} odom={} cmd_vel={} nav_cancel={}",
        occ_topic.channel(),
        tf_topic.channel(),
        cm_binding.as_ref().map(|(topic, _)| topic.channel()).unwrap_or("<none>"),
        nav_binding.as_ref().map(|(topic, _)| topic.channel()).unwrap_or("<none>"),
        fr_binding.as_ref().map(|(topic, _)| topic.channel()).unwrap_or("<none>"),
        pose_binding.as_ref().map(|(topic, _)| topic.channel()).unwrap_or("<none>"),
        odom_binding.as_ref().map(|(topic, _)| topic.channel()).unwrap_or("<none>"),
        cmd_vel_binding.as_ref().map(|(topic, _)| topic.channel()).unwrap_or("<none>"),
        nav_cancel_binding.as_ref().map(|(topic, _)| topic.channel()).unwrap_or("<none>"),
    );
    info!(
        "LCM msg-type registry: occupancy={} tf={} cloud_map={} nav_state={} frontiers={} pose={} odom={} cmd_vel={} nav_cancel={}",
        occ_msg_type.fqn(), tf_msg_type.fqn(),
        cm_binding.as_ref().map(|(_, kind)| kind.fqn()).unwrap_or("<none>"),
        nav_binding.as_ref().map(|(_, kind)| kind.fqn()).unwrap_or("<none>"),
        fr_binding.as_ref().map(|(_, kind)| kind.fqn()).unwrap_or("<none>"),
        pose_binding.as_ref().map(|(_, kind)| kind.fqn()).unwrap_or("<none>"),
        odom_binding.as_ref().map(|(_, kind)| kind.fqn()).unwrap_or("<none>"),
        cmd_vel_binding.as_ref().map(|(_, kind)| kind.fqn()).unwrap_or("<none>"),
        nav_cancel_binding.as_ref().map(|(_, kind)| kind.fqn()).unwrap_or("<none>"),
    );

    // 累积地图 mmap 初始化（降级允许）
    let cm_shm_path = std::path::PathBuf::from(
        std::env::var("NUWAX_CLOUDMAP_SHM_PATH")
            .unwrap_or_else(|_| "/tmp/nuwax_cloudmap.mmap".into()),
    );
    let cm_writer = match CloudMapShmWriter::new(&cm_shm_path) {
        Ok(w) => Some(w),
        Err(e) => {
            warn!("累积地图 mmap 初始化失败（降级为无 cloud_map 模式）: {}", e);
            None
        }
    };
    // 用 parking_lot::Mutex 避免 std::sync::MutexGuard 不 Send 的问题。
    let cm_writer = Arc::new(parking_lot::Mutex::new(cm_writer));

    // ---- 订阅 /tf：写入 canonical TfTree 历史并合成 base→map 位姿 ----
    let map_frame = std::env::var("NUWAX_MAP_FRAME").unwrap_or_else(|_| "map".into());
    let base_frame = std::env::var("NUWAX_BASE_FRAME").unwrap_or_else(|_| "base_link".into());
    let base_candidates: Vec<String> =
        if base_frame == "base_link" || base_frame == "base_footprint" {
            vec!["base_link".into(), "base_footprint".into()]
        } else {
            vec![
                base_frame.clone(),
                "base_link".into(),
                "base_footprint".into(),
            ]
        };

    let tf_state = state.clone();
    let tf_transport = transport.clone();
    let tf_topic_clone = tf_topic.clone();
    let tf_msg_type_clone = tf_msg_type.clone();
    let map_frame_owned = map_frame.clone();
    let base_candidates_owned = base_candidates.clone();
    tokio::spawn(async move {
        let mut stream = tf_transport
            .subscribe(tf_topic_clone, Some(tf_msg_type_clone))
            .await;
        use futures::StreamExt;
        while let Some(frame_result) = stream.next().await {
            let frame = match frame_result {
                Ok(f) => f,
                Err(e) => {
                    error!("/tf LCM 订阅错误: {}", e);
                    continue;
                }
            };
            let new_entries = match parse_tf_payload(&frame.payload) {
                Some(v) => v,
                None => {
                    warn!("/tf payload 解析失败 ({} 字节)", frame.payload.len());
                    continue;
                }
            };
            let mut s = tf_state.write().await;
            let mut max_stamp_ns: u64 = 0;
            for entry in new_entries {
                let Some(transform) = entry_to_transform(&entry) else {
                    warn!(
                        parent = %entry.parent,
                        child = %entry.child,
                        "忽略 frame/quaternion 非法的 /tf entry"
                    );
                    continue;
                };
                let stamp = entry.stamp_ns;
                s.tf_tree.set(transform);
                if let Some(existing) = s
                    .transforms
                    .iter_mut()
                    .find(|t| t.parent == entry.parent && t.child == entry.child)
                {
                    *existing = entry;
                } else {
                    s.transforms.push(entry);
                }
                if stamp > max_stamp_ns {
                    max_stamp_ns = stamp;
                }
            }
            if max_stamp_ns > TF_ENTRY_TTL_NS {
                let cutoff = max_stamp_ns - TF_ENTRY_TTL_NS;
                s.transforms
                    .retain(|t| t.stamp_ns == 0 || t.stamp_ns >= cutoff);
            }
            let base_refs: Vec<&str> = base_candidates_owned.iter().map(|s| s.as_str()).collect();
            if let Some((x, y, z, qx, qy, qz, qw, _base)) =
                resolve_map_pose_tree(&s.tf_tree, &map_frame_owned, &base_refs)
            {
                s.pose.x = x;
                s.pose.y = y;
                s.pose.z = z;
                s.pose.qx = qx;
                s.pose.qy = qy;
                s.pose.qz = qz;
                s.pose.qw = qw;
                s.pose.frame_id = map_frame_owned.clone();
                if max_stamp_ns > 0 {
                    s.pose.timestamp_ns = max_stamp_ns;
                }
            }
        }
    });

    // ---- 订阅 /odom：若上游 SLAM/odometry 节点直接发 odom，覆盖 pose ----
    if let Some((odom_topic, odom_msg_type)) = odom_binding {
        let odom_state = state.clone();
        let odom_transport = transport.clone();
        tokio::spawn(async move {
            let mut stream = odom_transport
                .subscribe(odom_topic, Some(odom_msg_type))
                .await;
            use futures::StreamExt;
            while let Some(frame_result) = stream.next().await {
                let frame = match frame_result {
                    Ok(f) => f,
                    Err(e) => {
                        error!("/odom LCM 订阅错误: {}", e);
                        continue;
                    }
                };
                if let Some(pose) = parse_odom_payload(&frame.payload) {
                    let mut s = odom_state.write().await;
                    s.pose = pose;
                }
            }
        });
    }

    // ---- 订阅 /map/occupancy ----
    let occ_state = state.clone();
    let occ_transport = transport.clone();
    let occ_topic_clone = occ_topic.clone();
    let occ_msg_type_clone = occ_msg_type.clone();
    let occ_revision_tx = map_revision_tx.clone();
    tokio::spawn(async move {
        let mut stream = occ_transport
            .subscribe(occ_topic_clone, Some(occ_msg_type_clone))
            .await;
        use futures::StreamExt;
        while let Some(frame_result) = stream.next().await {
            let frame = match frame_result {
                Ok(f) => f,
                Err(e) => {
                    error!("/map/occupancy LCM 订阅错误: {}", e);
                    continue;
                }
            };
            if let Some(map) = parse_occupancy_payload(&frame.payload) {
                commit_occupancy_map(&occ_state, map, &occ_revision_tx).await;
            }
        }
    });

    // ---- 订阅 cloud_map（累积点云）----
    if let Some((cm_topic, cm_msg_type)) = cm_binding {
        let cm_state = state.clone();
        let cm_tx = cloud_map_tx.clone();
        let cm_writer = cm_writer.clone();
        let cm_transport = transport.clone();
        let cm_msg_type_clone = cm_msg_type;
        tokio::spawn(async move {
            let mut stream = cm_transport
                .subscribe(cm_topic, Some(cm_msg_type_clone))
                .await;
            use futures::StreamExt;
            while let Some(frame_result) = stream.next().await {
                let frame = match frame_result {
                    Ok(f) => f,
                    Err(e) => {
                        error!("cloud_map LCM 订阅错误: {}", e);
                        continue;
                    }
                };
                if let Some(parsed) = parse_pointcloud_payload(&frame.payload) {
                    // 在锁内完成 mmap 写（不持锁跨 await），拿到结果后再 await
                    let write_result = {
                        let mut guard = cm_writer.lock();
                        if let Some(writer) = guard.as_mut() {
                            writer.write_cloud_map(&parsed.data, parsed.point_step)
                        } else {
                            None
                        }
                    };
                    if let Some((rev, count, vs)) = write_result {
                        {
                            let mut s = cm_state.write().await;
                            s.cloud_map_revision = rev;
                        }
                        let _ = cm_tx.send(CloudMapNotification {
                            revision: rev,
                            point_count: count,
                            voxel_size: vs,
                        });
                    }
                }
            }
        });
    }

    // ---- 订阅 nav_state（navigation state 推送）----
    if let Some((nav_topic, nav_msg_type)) = nav_binding {
        let nav_state = state.clone();
        let nav_tx = nav_state_tx.clone();
        let nav_transport = transport.clone();
        let nav_msg_type_clone = nav_msg_type;
        let nav_runtime_state = nav_runtime.clone();
        tokio::spawn(async move {
            let mut stream = nav_transport
                .subscribe(nav_topic, Some(nav_msg_type_clone))
                .await;
            use futures::StreamExt;
            while let Some(frame_result) = stream.next().await {
                let frame = match frame_result {
                    Ok(f) => f,
                    Err(e) => {
                        error!("nav_state LCM 订阅错误: {}", e);
                        continue;
                    }
                };
                let Some(nav) = crate::typed_codec::decode_nav_state_payload(&frame.payload) else {
                    warn!(
                        "nav_state generated payload 解析失败 ({} 字节)",
                        frame.payload.len()
                    );
                    continue;
                };
                let nav = nav_runtime_state.observe_stack_state(nav).await;
                let mut s = nav_state.write().await;
                s.nav_state = nav.clone();
                let _ = nav_tx.send(nav);
            }
        });
    }

    // ---- 订阅 frontiers ----
    if let Some((fr_topic, fr_msg_type)) = fr_binding {
        let fr_state = state.clone();
        let fr_tx = frontiers_tx.clone();
        let fr_transport = transport.clone();
        let fr_msg_type_clone = fr_msg_type;
        tokio::spawn(async move {
            let mut stream = fr_transport
                .subscribe(fr_topic, Some(fr_msg_type_clone))
                .await;
            use futures::StreamExt;
            while let Some(frame_result) = stream.next().await {
                let frame = match frame_result {
                    Ok(f) => f,
                    Err(e) => {
                        error!("frontiers LCM 订阅错误: {}", e);
                        continue;
                    }
                };
                let Some(entries) = crate::typed_codec::decode_frontiers_payload(&frame.payload)
                else {
                    warn!(
                        "frontiers generated payload 解析失败 ({} 字节)",
                        frame.payload.len()
                    );
                    continue;
                };
                if entries.len() > FRONTIERS_MAX_ENTRIES_PER_FRAME {
                    warn!(
                        "/explore/frontiers payload count={} 超过 sanity cap={};按异常帧跳过",
                        entries.len(),
                        FRONTIERS_MAX_ENTRIES_PER_FRAME
                    );
                    continue;
                }
                let mut s = fr_state.write().await;
                s.frontiers = entries.clone();
                s.frontiers_revision += 1;
                let _ = fr_tx.send(entries);
            }
        });
    }

    // Phase 3: 构造 mapping adapter。backend 不可用时为 None;
    // 配置从 env 探测(NUWAX_MAPPING_*);默认 200×200 @ 0.05m/cell。
    let mapping_adapter = if backend_available {
        let cfg = MappingAdapterConfig::from_env();
        info!(
            "mapping_adapter 启动: resolution={}m auto_bounds_padding=1m occupancy_algo={}",
            cfg.resolution,
            cfg.occupancy_algo.as_dimos_str()
        );
        Some(Arc::new(parking_lot::Mutex::new(MappingAdapter::new(cfg))))
    } else {
        None
    };

    // Phase 3 增强:订阅 /odom 和 /go2/lidar/points 时同步把数据喂给 mapping
    // adapter。`feed_pose` / `feed_pointcloud` 内部持 parking_lot 锁同步,
    // 不会跨 await。
    if let Some(mapping_arc) = mapping_adapter.clone() {
        spawn_mapping_feed_tasks(
            transport.clone(),
            state.clone(),
            mapping_arc,
            map_revision_tx.clone(),
        );
    }

    // Phase 3: 构造 occupancy grid SHM sidecar(默认 /tmp/nuwax_occupancy.mmap)。
    // 与 cloud_map SHM 区分:occupancy 走 grid layout(Phase 3 新增);
    // cloud_map 仍走 lcm_runtime 既有 `cm_writer`(Phase 0 已实装)。
    let occ_shm_path = std::path::PathBuf::from(
        std::env::var("NUWAX_OCCUPANCY_SHM_PATH")
            .unwrap_or_else(|_| "/tmp/nuwax_occupancy.mmap".into()),
    );
    let occupancy_shm = if backend_available {
        match OccupancyGridShmWriter::new(&occ_shm_path) {
            Ok(w) => {
                info!("occupancy grid SHM 已就绪: {:?}", occ_shm_path);
                Some(Arc::new(parking_lot::Mutex::new(w)))
            }
            Err(e) => {
                warn!("occupancy grid SHM 初始化失败 (降级为无 SHM): {}", e);
                None
            }
        }
    } else {
        None
    };
    let occupancy_shm_path = occupancy_shm
        .as_ref()
        .map(|_| occ_shm_path.to_string_lossy().into_owned());

    Ok(GeometryRuntimeHandle {
        state,
        cloud_map_tx,
        nav_state_tx,
        frontiers_tx,
        map_revision_tx,
        transport,
        transport_backend_name: backend_name,
        lcm_backend_available: backend_available,
        // Phase 6 / Rework-9: 默认 `false` —— 这是**生产 spawn 路径**,
        // 该路径必须由 Python `LcmMotionCommandBridge.start()` 成功完成后,
        // 通过 gateway `_start_motion_command_bridge` 链路真实写回 true;
        // 若 Python 端未启动 / start 失败 / providers 不实现 MotionControl,
        // 该 flag 保持 false,`SetNavigationGoal` 在 `compute_readiness` 因
        // `cmd_vel_sink_ready=false` 被拒绝(参见 `REJECT_REASON_CMD_VEL_SINK_DOWN`)。
        //
        // 注意:`build_handle_for_tests`(第 1354 行 test helper profile)
        // 仍保留默认 `true` —— 单测直接调 `set_cmd_vel_sink_ready(false)`
        // 验证 readiness 拒绝分支(参照 `phase6_handle_cmd_vel_sink_ready_setter_propagates_to_readiness`)。
        cmd_vel_sink_ready: Arc::new(std::sync::atomic::AtomicBool::new(false)),
        nav: nav_runtime,
        explore: crate::explore_runtime::ExploreRuntimeHandle::default_with_wavefront(),
        mapping_adapter,
        occupancy_shm,
        occupancy_shm_path,
    })
}

fn ingest_mapping_points(
    adapter: &mut MappingAdapter,
    points: &[nalgebra::Point3<f64>],
    point_step: u32,
    now_ns: u64,
    frame_id: &str,
    source_timestamp_ns: u64,
    state: &Arc<RwLock<SharedState>>,
    map_revision_tx: &broadcast::Sender<MapRevisionEvent>,
) -> Result<bool, nuwax_mapping::SessionError> {
    let odom_xy = (adapter.last_sensor_origin[0], adapter.last_sensor_origin[1]);
    let stats = adapter.feed_pointcloud_session(
        points,
        odom_xy,
        point_step,
        now_ns,
        frame_id,
        source_timestamp_ns,
    )?;
    if stats.accepted == 0 {
        return Ok(false);
    }
    Ok(adapter.build_and_publish(state, Some(map_revision_tx)))
}

async fn commit_occupancy_map(
    state: &Arc<RwLock<SharedState>>,
    mut map: MapState,
    map_revision_tx: &broadcast::Sender<MapRevisionEvent>,
) {
    let event = {
        let mut state = state.write().await;
        map.revision = state.map.revision.saturating_add(1);
        state.map = map;
        MapRevisionEvent::from_map(&state.map)
    };
    let _ = map_revision_tx.send(event);
}

/// Phase 3: 把 LCM `/odom` 与 `/go2/lidar/points` typed channel 喂给 mapping adapter。
///
/// 不替换 `spawn_geometry_runtime` 中已有的 odom/occupancy 订阅路径(那些驱动
/// `SharedState`);mapping feed 是 **附加** 的数据通道,用于累积点云 +
/// 维护 `last_sensor_origin`,然后调用 `build_and_publish` 写回 `SharedState.map`。
///
/// - `/odom` payload 走 `parse_odom_payload` → 提取 x/y/z → `feed_pose`
/// - `/go2/lidar/points` payload 走 `parse_pointcloud_payload` → 转 Point3 →
///   `feed_pointcloud` + `build_and_publish`
///
/// feed 路径持 parking_lot Mutex(同步,不跨 await);build_and_publish
/// 内部走 `SharedState.try_write`(同步)。
fn spawn_mapping_feed_tasks(
    transport: LcmTransport,
    state: Arc<RwLock<SharedState>>,
    mapping_arc: Arc<parking_lot::Mutex<MappingAdapter>>,
    map_revision_tx: broadcast::Sender<MapRevisionEvent>,
) {
    use futures::StreamExt;
    use nalgebra::Point3;

    // ---- /odom → feed_pose ----
    let (odom_topic, odom_msg_type) = match binding_for_key(ChannelKey::Odom, "nav_msgs.Odometry") {
        Ok(binding) => binding,
        Err(e) => {
            error!(
                "spawn_mapping_feed_tasks: odometry typed channel 无效: {}",
                e
            );
            return;
        }
    };
    let transport_odom = transport.clone();
    let mapping_odom = mapping_arc.clone();
    tokio::spawn(async move {
        let mut stream = transport_odom
            .subscribe(odom_topic, Some(odom_msg_type))
            .await;
        while let Some(frame_result) = stream.next().await {
            let frame = match frame_result {
                Ok(f) => f,
                Err(e) => {
                    error!("mapping_feed /odom LCM 订阅错误: {}", e);
                    continue;
                }
            };
            if let Some(pose) = parse_odom_payload(&frame.payload) {
                let mut adapter = mapping_odom.lock();
                adapter.feed_pose(pose.x, pose.y, pose.z);
            }
        }
    });

    // ---- /go2/lidar/points → feed_pointcloud + build_and_publish ----
    let (lidar_topic, lidar_msg_type) =
        match binding_for_key(ChannelKey::PointCloud, "sensor_msgs.PointCloud2") {
            Ok(binding) => binding,
            Err(e) => {
                error!(
                    "spawn_mapping_feed_tasks: pointcloud typed channel 无效: {}",
                    e
                );
                return;
            }
        };
    let transport_lidar = transport.clone();
    let mapping_lidar = mapping_arc.clone();
    let state_lidar = state.clone();
    let map_revision_lidar = map_revision_tx.clone();
    tokio::spawn(async move {
        let mut stream = transport_lidar
            .subscribe(lidar_topic, Some(lidar_msg_type))
            .await;
        while let Some(frame_result) = stream.next().await {
            let frame = match frame_result {
                Ok(f) => f,
                Err(e) => {
                    error!("mapping_feed /go2/lidar/points LCM 订阅错误: {}", e);
                    continue;
                }
            };
            let parsed = match parse_pointcloud_payload(&frame.payload) {
                Some(parsed) => parsed,
                None => continue,
            };
            let ps = parsed.point_step as usize;
            if ps < 12 {
                continue;
            }
            let raw_count = parsed.data.len() / ps;
            // 防御性 sanity cap:见 POINTCLOUD_MAX_RAW_POINTS 注释。
            // raw_count 由 untrusted bytes.len()/point_step 推得,point_step
            // 已在 parse_pointcloud_payload 入口校验过 [12, 256],但 raw_count
            // 仍可能因 bytes.len() 巨大而超 cap(>8M 点);此处直接 saturating
            // 掉,拒绝异常帧。
            if raw_count > crate::pointcloud::POINTCLOUD_MAX_RAW_POINTS {
                warn!(
                    "mapping_feed /go2/lidar/points raw_count={} 超过 sanity cap={};按异常帧跳过",
                    raw_count,
                    crate::pointcloud::POINTCLOUD_MAX_RAW_POINTS
                );
                continue;
            }
            let mut points: Vec<Point3<f64>> = Vec::with_capacity(raw_count);
            for i in 0..raw_count {
                let off = i * ps;
                if off + 12 > parsed.data.len() {
                    break;
                }
                let x = f32::from_le_bytes(parsed.data[off..off + 4].try_into().unwrap_or([0; 4]))
                    as f64;
                let y =
                    f32::from_le_bytes(parsed.data[off + 4..off + 8].try_into().unwrap_or([0; 4]))
                        as f64;
                let z =
                    f32::from_le_bytes(parsed.data[off + 8..off + 12].try_into().unwrap_or([0; 4]))
                        as f64;
                if x.is_nan() || y.is_nan() || z.is_nan() {
                    continue;
                }
                points.push(Point3::new(x, y, z));
            }
            if points.is_empty() {
                continue;
            }
            // MappingSession 是生产建图的唯一点云数据源。只有 session 接受了
            // 新点后才构建 revision，dedup/stale/resource-exhausted 帧不会发布假更新。
            let mut adapter = mapping_lidar.lock();
            let now_ns = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64;
            match ingest_mapping_points(
                &mut adapter,
                &points,
                parsed.point_step,
                now_ns,
                &parsed.frame_id,
                parsed.timestamp_ns,
                &state_lidar,
                &map_revision_lidar,
            ) {
                Ok(_) => {}
                Err(error) => {
                    warn!("mapping_feed: MappingSession 拒绝点云帧: {error}");
                }
            }
        }
    });
}

/// 单测端口隔离:Architect R1 防止重现 07_08 实机 175 GB 崩溃。
///
/// `pid * 10000 + counter` 让 cargo test 的 N 个并行 test 进程分到不同端口,
/// 避免互相 bind 失败 / udpm 重复收包导致 Rust 进程内存爆涨。
///
/// 2026-07-08 fake-lcm 物理删除后,所有 `build_handle_for_tests` 走 real-lcm,
/// 必须用隔离 URL 防 cargo test 撞端口(原 fake backend 无此约束)。
///
/// 不加 `#[cfg(test)]`:函数本身只返回 String(无 test-only 依赖),
/// 公开供 `build_handle_for_tests` 跨 #[cfg(test)] 模块引用,且允许 binary
/// 在测试场景外显式构造测试 URL 用。
pub fn test_lcm_url() -> String {
    use std::sync::atomic::{AtomicU64, Ordering};
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    let pid = std::process::id() as u64;
    let counter = COUNTER.fetch_add(1, Ordering::Relaxed);
    let port = (pid * 10_000 + counter) % 60_000 + 7_667; // [7667, 67666]
    format!("udpm://239.255.76.67:{}?ttl=1", port)
}

/// 单元测试 helper:构造一个最简 `GeometryRuntimeHandle`。
///
/// 不会启动订阅任务,只提供 real-lcm backend(test_lcm_url 隔离端口) +
/// 全新 `SharedState`,供 `nav_runtime` / `mapping_adapter` / 其它模块做集成测试。
/// 2026-07-08 fake-lcm 物理删除后,test helper 改走 `LcmTransport::new_real` —— 单测
/// 真实验证 liblcm 行为,不再有 fake-only 兼容路径。
///
/// 与 `spawn_geometry_runtime` 的关键差异:
/// - 不订阅任何 LCM typed channel
/// - 不写 SHM 文件
/// - 立即返回,适合 `#[tokio::test]` async 测试
pub async fn build_handle_for_tests() -> Result<GeometryRuntimeHandle, RealLcmBackendUnavailable> {
    let (cloud_map_tx, _) = broadcast::channel::<CloudMapNotification>(4);
    let (nav_state_tx, _) = broadcast::channel::<NavState>(8);
    let (frontiers_tx, _) = broadcast::channel::<Vec<FrontierEntry>>(4);
    let (map_revision_tx, _) = broadcast::channel::<MapRevisionEvent>(16);
    let state = Arc::new(RwLock::new(SharedState::default()));
    // 2026-07-08 fake-lcm 物理删除(v3 F2 决策):build_handle_for_tests 改走
    // RealLcmBackend;test_lcm_url() 端口公式防 cargo test 并行端口冲突
    // (Architect R1:直接重现 07_08 实机 175 GB 崩溃)。
    let transport = LcmTransport::new_real(&crate::lcm_runtime::test_lcm_url())
        .expect("test real LCM 启动失败,需先 apt-get install -y liblcm-dev 1.3~1.5");
    // Phase 3: 测试 helper 也构造 mapping_adapter,便于 unit test 走完整链路。
    let mapping_adapter = Some(Arc::new(parking_lot::Mutex::new(MappingAdapter::new(
        MappingAdapterConfig::from_env(),
    ))));
    // Phase 3: 测试 helper 也构造 occupancy_shm(走 /tmp 测试路径)
    // 注意:必须包含 per-call 唯一后缀(monotonic counter),否则并行
    // cargo test 跑多次 build_handle_for_tests 时,多个
    // OccupancyGridShmWriter::new() 会同时 truncate 同一文件 + 同一
    // mmap 段,导致 SIGBUS(实测:28 tests pass 后崩溃)。
    static OCC_COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let occ_test_path = std::path::PathBuf::from(format!(
        "/tmp/nuwax_occupancy_test_{}_{}.mmap",
        std::process::id(),
        OCC_COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed),
    ));
    let occupancy_shm = match OccupancyGridShmWriter::new(&occ_test_path) {
        Ok(w) => Some(Arc::new(parking_lot::Mutex::new(w))),
        Err(_) => None,
    };
    let occupancy_shm_path = occupancy_shm
        .as_ref()
        .map(|_| occ_test_path.to_string_lossy().into_owned());
    Ok(GeometryRuntimeHandle {
        state,
        cloud_map_tx,
        nav_state_tx,
        frontiers_tx,
        map_revision_tx,
        transport,
        transport_backend_name: "real-lcm",
        lcm_backend_available: true,
        // Phase 6 / Rework-9:test helper **保留** 默认 `true`(2026-07-08 fake-lcm
        // 物理删除后,test helper 改走 real backend,默认 true 仍兼容旧测试断言)。
        // 配套测试用 `set_cmd_vel_sink_ready(false)` 验证 readiness 拒绝分支。
        // 生产 spawn 路径(`spawn_geometry_runtime`)默认 `false`。
        cmd_vel_sink_ready: Arc::new(std::sync::atomic::AtomicBool::new(true)),
        nav: crate::nav_runtime::NavRuntimeState::from_env(),
        explore: crate::explore_runtime::ExploreRuntimeHandle::default_with_wavefront(),
        mapping_adapter,
        occupancy_shm,
        occupancy_shm_path,
    })
}

#[cfg(test)]
mod tf_tree_tests {
    use super::*;

    fn entry(stamp_ns: u64, tx: f64) -> TransformEntry {
        TransformEntry {
            parent: "/map".into(),
            child: "/base_link".into(),
            tx,
            ty: 2.0,
            tz: 0.0,
            rx: 0.0,
            ry: 0.0,
            rz: 0.0,
            rw: 1.0,
            stamp_ns,
        }
    }

    #[test]
    fn canonical_tree_retains_history_and_drives_latest_inventory() {
        let state = SharedState::default();
        state
            .tf_tree
            .set(entry_to_transform(&entry(1_000_000_100, 1.0)).unwrap());
        state
            .tf_tree
            .set(entry_to_transform(&entry(2_000_000_200, 3.0)).unwrap());

        assert_eq!(state.tf_tree.history_len("base_link"), 2);
        let historical = state
            .tf_tree
            .lookup_at("base_link", 1.000_000_100, 0.0)
            .unwrap();
        assert_eq!(historical.transform.translation.x, 1.0);
        let latest = state.latest_transform_entries();
        assert_eq!(latest.len(), 1);
        assert_eq!(latest[0].parent, "map");
        assert_eq!(latest[0].child, "base_link");
        assert_eq!(latest[0].tx, 3.0);
        assert_eq!(latest[0].stamp_ns, 2_000_000_200);
    }

    #[test]
    fn canonical_tree_resolves_base_to_map_and_rejects_invalid_quaternion() {
        let state = SharedState::default();
        state
            .tf_tree
            .set(entry_to_transform(&entry(2_000_000_200, 3.0)).unwrap());
        let pose = resolve_map_pose_tree(&state.tf_tree, "/map", &["/base_link"]).unwrap();
        assert_eq!((pose.0, pose.1, pose.2), (3.0, 2.0, 0.0));

        let mut invalid = entry(3_000_000_000, 0.0);
        invalid.rw = 0.0;
        assert!(entry_to_transform(&invalid).is_none());
    }
}

#[cfg(test)]
mod mapping_feed_tests {
    use super::*;
    use crate::mapping_adapter::MappingReadiness;
    use nalgebra::Point3;

    #[test]
    fn production_ingest_uses_mapping_session_and_broadcasts_revision() {
        let config = MappingAdapterConfig {
            resolution: 0.5,
            width_cells: 20,
            height_cells: 20,
            origin_xy: (-5.0, -5.0),
            voxel_size: 0.05,
            occupancy_algo: nuwax_mapping::cost_mapper::CostAlgoSelector::General,
            initial_safe_radius_meters: 0.0,
        };
        let mut adapter = MappingAdapter::new(config);
        adapter.feed_pose(1.0, 2.0, 0.0);
        let mut initial_state = SharedState::default();
        initial_state.map.revision = 9;
        let state = Arc::new(RwLock::new(initial_state));
        let (map_revision_tx, mut map_revision_rx) = broadcast::channel(4);
        let points = vec![Point3::new(1.0, 2.0, 0.0), Point3::new(1.5, 2.0, 0.0)];

        assert!(ingest_mapping_points(
            &mut adapter,
            &points,
            12,
            1_750_000_000_000_000_000,
            "mapping_lidar",
            1_234_567_890_123_456_789,
            &state,
            &map_revision_tx,
        )
        .expect("MappingSession ingest"));

        assert_eq!(adapter.accumulated_count(), points.len());
        assert_eq!(adapter.odom_count(), 1);
        assert_eq!(adapter.pointcloud_count(), 1);
        assert_eq!(adapter.current_revision(), 10);
        #[allow(deprecated)]
        {
            assert!(adapter.accumulator.clone_points().is_empty());
        }
        let event = map_revision_rx.try_recv().expect("map revision event");
        assert_eq!(event.revision, 10);
        assert_eq!(event.size, 20);
        assert_eq!(event.data.len() as u64, event.size);
        assert_eq!(event.checksum, crate::map_checksum::sha256_hex(&event.data));
        let committed = state.try_read().expect("committed mapping state");
        assert_eq!(committed.map.frame_id, "mapping_lidar");
        assert_eq!(committed.map.timestamp_ns, 1_234_567_890_123_456_789);
        assert_eq!(committed.map.revision, event.revision);
        assert_eq!(committed.map.data.len(), event.data.len());
        let readiness = MappingReadiness::compute(Some(&adapter), true);
        assert!(readiness.ready);
    }

    #[tokio::test]
    async fn occupancy_lcm_commit_also_broadcasts_the_committed_revision() {
        let mut initial_state = SharedState::default();
        initial_state.map.revision = 5;
        let state = Arc::new(RwLock::new(initial_state));
        let (map_revision_tx, mut map_revision_rx) = broadcast::channel(4);
        let map = MapState {
            width: 2,
            height: 1,
            resolution: 0.1,
            origin_x: 0.0,
            origin_y: 0.0,
            frame_id: "map".into(),
            revision: 0,
            timestamp_ns: 123_000_000,
            data: vec![0, 100],
        };

        commit_occupancy_map(&state, map, &map_revision_tx).await;

        let event = map_revision_rx
            .recv()
            .await
            .expect("occupancy revision event");
        assert_eq!(event.revision, 6);
        assert_eq!(event.data, vec![0, 100]);
        assert_eq!(event.checksum, crate::map_checksum::sha256_hex(&event.data));
        assert_eq!(state.read().await.map.revision, event.revision);
    }
}

#[cfg(test)]
mod real_lcm_startup_tests {
    use std::cell::Cell;

    use super::{attempt_real_lcm_startup, RealLcmStartupError};

    #[test]
    fn required_backend_attempts_creation_and_accepts_success() {
        let attempted = Cell::new(false);

        let result = attempt_real_lcm_startup(true, || {
            attempted.set(true);
            Ok::<_, &'static str>("real-lcm")
        });

        assert!(attempted.get(), "required=true 也必须先尝试创建 real-lcm");
        assert_eq!(result, Ok("real-lcm"));
    }

    #[test]
    fn required_backend_classifies_actual_creation_failure_for_hard_exit() {
        let result = attempt_real_lcm_startup(true, || Err::<(), _>("liblcm unavailable"));

        assert_eq!(
            result,
            Err(RealLcmStartupError::Required("liblcm unavailable"))
        );
    }

    #[test]
    fn optional_backend_returns_actual_creation_failure() {
        let result = attempt_real_lcm_startup(false, || Err::<(), _>("invalid provider"));

        assert_eq!(
            result,
            Err(RealLcmStartupError::Optional("invalid provider"))
        );
    }
}

#[cfg(test)]
mod payload_sanity_tests {
    //! 防御性 sanity cap 测试(2026-07-08 rework17 修复后)。
    //!
    //! 真实机 175 GB alloc 失败的根因推测:上游 LCM 帧 `count` 字段因 uninit
    //! memory 或协议异常被解析成 `usize::MAX` 级别,触发 `Vec::with_capacity`
    //! 申请上百 GB。本模块给 `parse_tf_payload` / `parse_occupancy_payload` /
    //! frontiers 路径加了 sanity cap,本测试用直接构造异常 payload 的方式
    //! 验证 cap 生效,异常帧返回 None 而不分配。

    use super::{
        parse_occupancy_payload, parse_pointcloud_payload, parse_tf_payload,
        FRONTIERS_MAX_ENTRIES_PER_FRAME, OCCUPANCY_MAX_DIMENSION, TF_MAX_ENTRIES_PER_FRAME,
    };
    use nuwax_codec::lcm::schema;

    /// count 字段填 0xFFFF_FFFF(usize 等价 ~ 18 EB)→ 必须被 sanity cap 拒绝
    #[test]
    fn parse_tf_payload_rejects_huge_count() {
        let payload = i32::MAX.to_be_bytes();
        assert!(
            parse_tf_payload(&payload).is_none(),
            "parse_tf_payload 应该拒绝 count > TF_MAX_ENTRIES_PER_FRAME 的异常帧"
        );
    }

    /// count 字段填 0 → 应正常解析为空 vec
    #[test]
    fn parse_tf_payload_accepts_zero_count() {
        let payload = schema::encode_tf_message_body(&schema::TfMessage::default())
            .expect("encode empty TFMessage");
        let result = parse_tf_payload(&payload).expect("empty TFMessage 应可解析");
        assert!(result.is_empty());
    }

    /// width = u32::MAX → 应被 OCCUPANCY_MAX_DIMENSION 拒绝
    #[test]
    fn parse_occupancy_payload_rejects_huge_dimension() {
        let payload = schema::encode_occupancy_grid_body(&schema::OccupancyGrid {
            info: schema::MapMetaData {
                width: i32::MAX,
                height: i32::MAX,
                ..schema::MapMetaData::default()
            },
            ..schema::OccupancyGrid::default()
        })
        .expect("encode hostile OccupancyGrid");
        assert!(
            parse_occupancy_payload(&payload).is_none(),
            "parse_occupancy_payload 应该拒绝 width/height > OCCUPANCY_MAX_DIMENSION 的异常帧"
        );
    }

    /// 合法 width*height=16 + data_len=16 happy path
    #[test]
    fn parse_occupancy_payload_accepts_well_formed() {
        let payload = schema::encode_occupancy_grid_body(&schema::OccupancyGrid {
            header: schema::Header {
                stamp_sec: 12,
                stamp_nsec: 34,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            info: schema::MapMetaData {
                resolution: 0.05,
                width: 4,
                height: 4,
                ..schema::MapMetaData::default()
            },
            data: vec![0; 16],
        })
        .expect("encode OccupancyGrid");
        let result = parse_occupancy_payload(&payload).expect("happy path 应解析成功");
        assert_eq!(result.width, 4);
        assert_eq!(result.height, 4);
        assert_eq!(result.data.len(), 16);
        assert_eq!(result.frame_id, "map");
        assert_eq!(result.timestamp_ns, 12_000_000_034);
    }

    /// cap 常量本身在物理合理范围内(const-eval 不会触发 lint)
    #[test]
    fn sanity_caps_are_in_physically_reasonable_range() {
        // 这些都是编译期常量,clippy 会把 assert!(const_expr) 警告为可优化;
        // 这里改用 const_assertions 风格:`const _: () = assert!(...);` 是
        // 编译期验证,运行时无开销。
        const _: () = assert!(TF_MAX_ENTRIES_PER_FRAME > 0 && TF_MAX_ENTRIES_PER_FRAME < 1_000_000);
        const _: () = assert!(
            FRONTIERS_MAX_ENTRIES_PER_FRAME > 0 && FRONTIERS_MAX_ENTRIES_PER_FRAME < 1_000_000
        );
        const _: () = assert!(OCCUPANCY_MAX_DIMENSION > 0 && OCCUPANCY_MAX_DIMENSION < 100_000);
        const _: () =
            assert!((OCCUPANCY_MAX_DIMENSION as usize).saturating_pow(2) < 100 * 1024 * 1024);
        const _: () = assert!(
            crate::pointcloud::POINTCLOUD_MAX_RAW_POINTS > 0
                && crate::pointcloud::POINTCLOUD_MAX_RAW_POINTS < 100_000_000
        );
        const _: () = assert!(
            crate::pointcloud::POINTCLOUD_MAX_POINT_STEP_BYTES >= 12
                && crate::pointcloud::POINTCLOUD_MAX_POINT_STEP_BYTES < 4_096
        );
    }

    /// point_step = 1 (untrusted 上游 frame 异常)→ 必须被 sanity cap 拒绝
    /// (point_step < 12 触发)
    #[test]
    fn parse_pointcloud_payload_rejects_tiny_point_step() {
        let payload = pointcloud_body(1, vec![0; 1]);
        assert!(
            parse_pointcloud_payload(&payload).is_none(),
            "parse_pointcloud_payload 应该拒绝 point_step < 12 的异常帧"
        );
    }

    /// point_step = u32::MAX → 必须被 POINTCLOUD_MAX_POINT_STEP_BYTES 拒绝
    #[test]
    fn parse_pointcloud_payload_rejects_huge_point_step() {
        let payload = pointcloud_body(4_096, vec![0; 4_096]);
        assert!(
            parse_pointcloud_payload(&payload).is_none(),
            "parse_pointcloud_payload 应该拒绝 point_step > POINTCLOUD_MAX_POINT_STEP_BYTES 的异常帧"
        );
    }

    /// point_step = 12 + 24 bytes raw → 应正常解析 (happy path)
    #[test]
    fn parse_pointcloud_payload_accepts_well_formed() {
        let payload = pointcloud_body(12, vec![0; 24]);
        let parsed = parse_pointcloud_payload(&payload).expect("happy path 应解析成功");
        assert_eq!(parsed.point_step, 12);
        assert_eq!(parsed.data.len(), 24);
        assert_eq!(parsed.frame_id, "map");
        assert_eq!(parsed.timestamp_ns, 12_345_678_901);
    }

    fn pointcloud_body(point_step: i32, data: Vec<u8>) -> Vec<u8> {
        let width = i32::try_from(data.len() / point_step.max(1) as usize).unwrap();
        schema::encode_point_cloud2_body(&schema::PointCloud2 {
            header: schema::Header {
                stamp_sec: 12,
                stamp_nsec: 345_678_901,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            height: 1,
            width,
            fields: vec![
                schema::PointField {
                    name: "x".into(),
                    offset: 0,
                    datatype: 7,
                    count: 1,
                },
                schema::PointField {
                    name: "y".into(),
                    offset: 4,
                    datatype: 7,
                    count: 1,
                },
                schema::PointField {
                    name: "z".into(),
                    offset: 8,
                    datatype: 7,
                    count: 1,
                },
            ],
            point_step,
            row_step: point_step.saturating_mul(width),
            data,
            is_dense: true,
            ..schema::PointCloud2::default()
        })
        .expect("encode PointCloud2")
    }
}
