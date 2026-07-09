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
//! - `/tf#geometry_msgs.Transform`
//! - `/map/occupancy#nav_msgs.OccupancyGrid`
//! - `/go2/lidar/points#sensor_msgs.PointCloud2`
//! - `/nav/state#nuwax_msgs.NavigationState`
//! - `/explore/frontiers#nuwax_msgs.FrontierList`
//!
//! 这里的 `nav_msgs/Odometry` 等名称是 LCM typed message 命名空间字符串，
//! 不引入 ROS2 package 依赖。

use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::Arc;

use nuwax_protocol::{LcmMessageType, LcmTopic};
use nuwax_transport::LcmTransport;
use thiserror::Error;
use tokio::sync::{broadcast, RwLock};
use tracing::{error, info, warn};

use crate::mapping_adapter::{FeedOnlyAccumulator, MappingAdapter, MappingAdapterConfig};
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
    pub x: f64, pub y: f64, pub z: f64,
    pub qx: f64, pub qy: f64, pub qz: f64, pub qw: f64,
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
    pub tx: f64, pub ty: f64, pub tz: f64,
    pub rx: f64, pub ry: f64, pub rz: f64, pub rw: f64,
    /// 该 TF 边的 stamp（纳秒，LCM 时间域）。0 表示无戳（latched/静态边），永不过期。
    pub stamp_ns: u64,
}

/// 动态 /tf 边存活时长（纳秒）。stamp>0 且超过 TTL 的边在合成位姿前被裁剪，
/// 避免老版本 RTAB-Map/LCM 重启遗留陈旧 map→odom 污染 BFS。stamp==0 不过期。
const TF_ENTRY_TTL_NS: u64 = 5_000_000_000;

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
    pub x: f64, pub y: f64, pub z: f64,
    pub qx: f64, pub qy: f64, pub qz: f64, pub qw: f64,
    pub frame_id: String,
    pub size_cells: u32,
    pub heading: f32,
}

pub struct SharedState {
    pub pose: PoseState,
    pub map: MapState,
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

#[derive(Clone, Debug)]
pub struct CloudMapNotification {
    pub revision: u64,
    pub point_count: u32,
    pub voxel_size: f32,
}

/// 几何 runtime 句柄。`server.rs` 通过此句柄读 SharedState、订阅 cloud_map_tx
/// 以及通过 transport.publish 推送 odom / point cloud(real backend 直透 liblcm)。
#[derive(Clone)]
pub struct GeometryRuntimeHandle {
    pub state: Arc<RwLock<SharedState>>,
    pub cloud_map_tx: broadcast::Sender<CloudMapNotification>,
    pub nav_state_tx: broadcast::Sender<NavState>,
    pub frontiers_tx: broadcast::Sender<Vec<FrontierEntry>>,
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
}

impl GeometryRuntimeHandle {
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

    /// navigation 板块 readiness。Phase 2 baseline:恒 `false`（nav_stack
    /// LCM channel 未接通）。Phase 4 在 nav_runtime.rs 实装后改 true。
    pub fn nav_ready_poll(&self) -> bool {
        false
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
        self.explore.refresh_readiness_cache_with_backend(self.lcm_backend_available);
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
    m[0][0] = 1.0; m[1][1] = 1.0; m[2][2] = 1.0; m[3][3] = 1.0;
    m
}

fn quat_to_rot(x: f64, y: f64, z: f64, w: f64) -> [[f64; 3]; 3] {
    let norm = (x * x + y * y + z * z + w * w).sqrt();
    if norm <= 1e-12 {
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    }
    let (x, y, z, w) = (x / norm, y / norm, z / norm, w / norm);
    [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]
}

fn mat_mul(a: &Mat4, b: &Mat4) -> Mat4 {
    let mut o = [[0.0f64; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            let mut s = 0.0;
            for k in 0..4 { s += a[i][k] * b[k][j]; }
            o[i][j] = s;
        }
    }
    o
}

fn entry_to_matrix(e: &TransformEntry) -> Mat4 {
    let r = quat_to_rot(e.rx, e.ry, e.rz, e.rw);
    let mut m = identity4();
    for i in 0..3 { for j in 0..3 { m[i][j] = r[i][j]; } }
    m[0][3] = e.tx; m[1][3] = e.ty; m[2][3] = e.tz;
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
    if source.is_empty() || target.is_empty() { return None; }
    if source == target { return Some(identity4()); }

    let mut adjacency: HashMap<String, Vec<(String, Mat4)>> = HashMap::new();
    for e in transforms {
        let parent = normalize_frame(&e.parent);
        let child = normalize_frame(&e.child);
        let m = entry_to_matrix(e); // child→parent
        adjacency.entry(child.clone()).or_default().push((parent.clone(), m));
        adjacency.entry(parent).or_default().push((child, rigid_inverse(&m)));
    }

    let mut visited: HashSet<String> = HashSet::new();
    visited.insert(source.clone());
    let mut queue: VecDeque<(String, Mat4)> = VecDeque::new();
    queue.push_back((source, identity4()));
    while let Some((frame, mat)) = queue.pop_front() {
        if frame == target { return Some(mat); }
        if let Some(edges) = adjacency.get(&frame) {
            for (next, next_mat) in edges {
                if visited.contains(next) { continue; }
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
        ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s)
    } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
        let s = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
        (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    } else if m[1][1] > m[2][2] {
        let s = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
        ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    } else {
        let s = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
        ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[0][1] - m[1][0]) / s)
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
            return Some((m[0][3], m[1][3], m[2][3], qx, qy, qz, qw, normalize_frame(base)));
        }
    }
    None
}

// ============================================================================
// LCM payload 解析辅助（minimal 自包含解码）
// ============================================================================
//
// LCM codec 的 typed wire format 当前在 nuwax-codec P0 阶段只定义了 LCM 1.x
// magic/fingerprint 框架，typed message body 仍由各 message crate 提供。
// 这里采用"明确字段布局约定"的策略：runtime 进程内对 typed channel payload
// 按固定字节布局解析（每个解析函数顶部注释说明 layout）。
//
// 该约定为本仓内 typed channel 与 gRPC facade 的固定契约；后续如需替换
// `lcm_msgs` 真实消息，应在 P1 lane 同步更新本文件解析 + nuwax-codec 的
// typed message 引入。

fn parse_odom_payload(payload: &[u8]) -> Option<PoseState> {
    // 56 字节位姿 + 8 字节 stamp + 2 字节 frame_len + frame bytes
    if payload.len() < 66 { return None; }
    let x = f64::from_le_bytes(payload[0..8].try_into().ok()?);
    let y = f64::from_le_bytes(payload[8..16].try_into().ok()?);
    let z = f64::from_le_bytes(payload[16..24].try_into().ok()?);
    let qx = f64::from_le_bytes(payload[24..32].try_into().ok()?);
    let qy = f64::from_le_bytes(payload[32..40].try_into().ok()?);
    let qz = f64::from_le_bytes(payload[40..48].try_into().ok()?);
    let qw = f64::from_le_bytes(payload[48..56].try_into().ok()?);
    let stamp = u64::from_le_bytes(payload[56..64].try_into().ok()?);
    let frame_len = u16::from_le_bytes(payload[64..66].try_into().ok()?) as usize;
    if payload.len() < 66 + frame_len { return None; }
    let frame_id = String::from_utf8_lossy(&payload[66..66+frame_len]).to_string();
    Some(PoseState { x, y, z, qx, qy, qz, qw, frame_id, timestamp_ns: stamp })
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
    // count(u32) + repeat[parent_len(u16)+parent+child_len(u16)+child+7×f64+stamp(u64)]
    if payload.len() < 4 { return None; }
    let count = u32::from_le_bytes(payload[0..4].try_into().ok()?) as usize;
    // 防御性 sanity cap:见 TF_MAX_ENTRIES_PER_FRAME 注释。
    if count > TF_MAX_ENTRIES_PER_FRAME {
        warn!(
            "/tf payload count={} 超过 sanity cap={};按异常帧跳过",
            count, TF_MAX_ENTRIES_PER_FRAME
        );
        return None;
    }
    let mut cursor = 4usize;
    let mut out = Vec::with_capacity(count);
    for _ in 0..count {
        if cursor + 2 > payload.len() { return None; }
        let p_len = u16::from_le_bytes(payload[cursor..cursor+2].try_into().ok()?) as usize;
        cursor += 2;
        if cursor + p_len > payload.len() { return None; }
        let parent = String::from_utf8_lossy(&payload[cursor..cursor+p_len]).to_string();
        cursor += p_len;
        if cursor + 2 > payload.len() { return None; }
        let c_len = u16::from_le_bytes(payload[cursor..cursor+2].try_into().ok()?) as usize;
        cursor += 2;
        if cursor + c_len > payload.len() { return None; }
        let child = String::from_utf8_lossy(&payload[cursor..cursor+c_len]).to_string();
        cursor += c_len;
        if cursor + 56 > payload.len() { return None; }
        let tx = f64::from_le_bytes(payload[cursor..cursor+8].try_into().ok()?);
        let ty = f64::from_le_bytes(payload[cursor+8..cursor+16].try_into().ok()?);
        let tz = f64::from_le_bytes(payload[cursor+16..cursor+24].try_into().ok()?);
        let rx = f64::from_le_bytes(payload[cursor+24..cursor+32].try_into().ok()?);
        let ry = f64::from_le_bytes(payload[cursor+32..cursor+40].try_into().ok()?);
        let rz = f64::from_le_bytes(payload[cursor+40..cursor+48].try_into().ok()?);
        let rw = f64::from_le_bytes(payload[cursor+48..cursor+56].try_into().ok()?);
        cursor += 56;
        if cursor + 8 > payload.len() { return None; }
        let stamp = u64::from_le_bytes(payload[cursor..cursor+8].try_into().ok()?);
        cursor += 8;
        out.push(TransformEntry {
            parent, child,
            tx, ty, tz, rx, ry, rz, rw,
            stamp_ns: stamp,
        });
    }
    Some(out)
}

fn parse_occupancy_payload(payload: &[u8]) -> Option<MapState> {
    // width(u32)+height(u32)+resolution(f32)+origin_x(f64)+origin_y(f64)+frame_len(u16)+frame+stamp(u64)+data_len(u32)+data_bytes(i8)
    if payload.len() < 30 { return None; }
    let width = u32::from_le_bytes(payload[0..4].try_into().ok()?);
    let height = u32::from_le_bytes(payload[4..8].try_into().ok()?);
    // 防御性 sanity cap:见 OCCUPANCY_MAX_DIMENSION 注释。
    if width == 0 || height == 0 || width > OCCUPANCY_MAX_DIMENSION || height > OCCUPANCY_MAX_DIMENSION {
        warn!(
            "/map/occupancy payload width={} height={} 超过 sanity cap={};按异常帧跳过",
            width, height, OCCUPANCY_MAX_DIMENSION
        );
        return None;
    }
    let resolution = f32::from_le_bytes(payload[8..12].try_into().ok()?);
    let origin_x = f64::from_le_bytes(payload[12..20].try_into().ok()?);
    let origin_y = f64::from_le_bytes(payload[20..28].try_into().ok()?);
    let frame_len = u16::from_le_bytes(payload[28..30].try_into().ok()?) as usize;
    if payload.len() < 30 + frame_len + 8 + 4 { return None; }
    let frame_id = String::from_utf8_lossy(&payload[30..30+frame_len]).to_string();
    let mut cur = 30 + frame_len;
    let stamp = u64::from_le_bytes(payload[cur..cur+8].try_into().ok()?);
    cur += 8;
    let data_len = u32::from_le_bytes(payload[cur..cur+4].try_into().ok()?) as usize;
    cur += 4;
    if cur + data_len > payload.len() { return None; }
    // data_len 必须等于 width*height;否则说明上游帧截断/异常,拒绝
    // 防止后续 Vec<i8> 申请非预期容量。
    let expected_len = (width as usize).saturating_mul(height as usize);
    if data_len != expected_len {
        warn!(
            "/map/occupancy payload data_len={} 与 width*height={} 不一致;按异常帧跳过",
            data_len, expected_len
        );
        return None;
    }
    let raw = payload[cur..cur+data_len].to_vec();
    let data: Vec<i8> = raw.into_iter().map(|b| b as i8).collect();
    Some(MapState {
        width, height, resolution, origin_x, origin_y,
        frame_id, revision: 0, timestamp_ns: stamp, data,
    })
}

fn parse_pointcloud_payload(payload: &[u8]) -> Option<(Vec<u8>, u32)> {
    // point_step(u32) + raw PointCloud2 bytes
    if payload.len() < 4 { return None; }
    let point_step = u32::from_le_bytes(payload[0..4].try_into().ok()?);
    // 防御性 sanity cap:见 POINTCLOUD_MAX_POINT_STEP_BYTES 注释。
    // point_step 来自 untrusted LCM frame,sender 可伪造极端值(0/1/u32::MAX)
    // 让后续 `bytes.len() / point_step` 推算 raw_count 时产生巨大值,触发
    // `Vec::with_capacity(raw_count)` 申请数百 GB alloc(2026-07-08 实机
    // 187GB abort 模式)。
    if point_step < 12 || point_step > crate::pointcloud::POINTCLOUD_MAX_POINT_STEP_BYTES {
        warn!(
            "/go2/lidar/points payload point_step={} 超过 sanity cap [12, {}];按异常帧跳过",
            point_step, crate::pointcloud::POINTCLOUD_MAX_POINT_STEP_BYTES
        );
        return None;
    }
    // 防御性 sanity cap:payload 总长 4 MB 上限,远超物理合理点云单帧
    // (Go2 单帧 ~30k 点 ≈ 360 KB);超过则 upstream LCM 帧异常,拒绝。
    if payload.len() > 4 * 1024 * 1024 {
        warn!(
            "/go2/lidar/points payload len={} 超过 4 MiB sanity cap;按异常帧跳过",
            payload.len()
        );
        return None;
    }
    Some((payload[4..].to_vec(), point_step))
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
/// counters + active backend(backend 字段值固定为 "real-lcm")。
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

    let state = Arc::new(RwLock::new(SharedState::default()));

    // 按 NUWAX_LCM_BACKEND 选择 backend。
    // 2026-07-08 fake-lcm 物理删除后,`LcmBackendKind::from_env()` 仅返回 `Real`。
    // Phase 5 (NUWAX_REAL_LCM_REQUIRED=true) 保持:启动失败时进程直接
    // `process::exit(1)`,不允许静默 fallback(fake backend 已物理删除)。
    let real_lcm_required = real_lcm_required_from_env();
    let (transport, backend_name, backend_available): (LcmTransport, &'static str, bool) = {
        let lcm_url = std::env::var("NUWAX_LCM_URL").unwrap_or_else(|_| {
            "udpm://239.255.76.67:7667?ttl=1".to_owned()
        });
        if real_lcm_required {
            eprintln!(
                "LCM_BACKEND_UNAVAILABLE: NUWAX_REAL_LCM_REQUIRED=true 时 real-lcm \
                 backend 启动失败（liblcm 缺失或 provider URL 错），按生产意图 \
                 立即硬退出 (exit code = 1)。"
            );
            std::process::exit(1);
        }
        // real-lcm feature gate —— 二选一:
        //   1. binary --features real-lcm-bridge → 真起 liblcm
        //   2. binary 未启用 real-lcm-bridge feature → 保留 Phase 2 行为 (return Err)
        //      2026-07-08 fake-lcm 物理删除后,该分支仅由 CI 误配触发,本意即"无 backend 可用"。
        #[cfg(feature = "real-lcm-bridge")]
        {
            match LcmTransport::new_real(&lcm_url) {
                Ok(t) => {
                    info!(
                        "LCM backend 选择: real-lcm (provider={})",
                        lcm_url
                    );
                    (t, "real-lcm", true)
                }
                Err(e) => {
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

    let occ_topic = match crate::nav_runtime::parse_channel_with_suffix(occupancy_channel) {
        Ok((t, _)) => t,
        Err(e) => {
            error!("lcm_runtime::spawn: occupancy channel 解析失败: {}", e);
            return Err(RealLcmBackendUnavailable);
        }
    };
    let tf_topic = match crate::nav_runtime::parse_channel_with_suffix(tf_channel) {
        Ok((t, _)) => t,
        Err(e) => {
            error!("lcm_runtime::spawn: tf channel 解析失败: {}", e);
            return Err(RealLcmBackendUnavailable);
        }
    };
    let cm_topic = match cloud_map_channel {
        Some(c) => match crate::nav_runtime::parse_channel_with_suffix(c) {
            Ok((t, _)) => Some(t),
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
    let nav_topic = match nav_state_channel {
        Some(c) => match crate::nav_runtime::parse_channel_with_suffix(c) {
            Ok((t, _)) => Some(t),
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
    let fr_topic = match frontiers_channel {
        Some(c) => match crate::nav_runtime::parse_channel_with_suffix(c) {
            Ok((t, _)) => Some(t),
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
    let pose_topic = match pose_channel {
        Some(c) => match crate::nav_runtime::parse_channel_with_suffix(c) {
            Ok((t, _)) => Some(t),
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
    let odom_topic = match odometry_channel {
        Some(c) => match crate::nav_runtime::parse_channel_with_suffix(c) {
            Ok((t, _)) => Some(t),
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
    let cmd_vel_topic = match cmd_vel_channel {
        Some(c) => match crate::nav_runtime::parse_channel_with_suffix(c) {
            Ok((t, _)) => Some(t),
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
    let nav_cancel_topic = match nav_cancel_channel {
        Some(c) => match crate::nav_runtime::parse_channel_with_suffix(c) {
            Ok((t, _)) => Some(t),
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

    let occ_msg_type = LcmMessageType::new("nav_msgs/OccupancyGrid").expect("typed");
    let tf_msg_type = LcmMessageType::new("geometry_msgs/Transform").expect("typed");
    let cm_msg_type = LcmMessageType::new("sensor_msgs/PointCloud2").expect("typed");
    let nav_msg_type = LcmMessageType::new("nuwax_msgs/NavigationState").expect("typed");
    let fr_msg_type = LcmMessageType::new("nuwax_msgs/FrontierList").expect("typed");
    let pose_msg_type = LcmMessageType::new("geometry_msgs/PoseStamped").expect("typed");
    let odom_msg_type = LcmMessageType::new("nav_msgs/Odometry").expect("typed");
    let cmd_vel_msg_type = LcmMessageType::new("geometry_msgs/Twist").expect("typed");
    let nav_cancel_msg_type = LcmMessageType::new("nuwax_msgs/CancelNavigation").expect("typed");
    // Phase 1:打印 channel registry 让运维和 healthcheck 都能看到本进程
    // 注册的全部 typed channel;Phase 2+ 将在此基础上挂接订阅任务。
    info!(
        "LCM channel registry: occupancy={} tf={} cloud_map={} nav_state={} frontiers={} pose={} odom={} cmd_vel={} nav_cancel={}",
        occ_topic.channel(),
        tf_topic.channel(),
        cm_topic.as_ref().map(LcmTopic::channel).unwrap_or("<none>"),
        nav_topic.as_ref().map(LcmTopic::channel).unwrap_or("<none>"),
        fr_topic.as_ref().map(LcmTopic::channel).unwrap_or("<none>"),
        pose_topic.as_ref().map(LcmTopic::channel).unwrap_or("<none>"),
        odom_topic.as_ref().map(LcmTopic::channel).unwrap_or("<none>"),
        cmd_vel_topic.as_ref().map(LcmTopic::channel).unwrap_or("<none>"),
        nav_cancel_topic.as_ref().map(LcmTopic::channel).unwrap_or("<none>"),
    );
    info!(
        "LCM msg-type registry: occupancy={} tf={} cloud_map={} nav_state={} frontiers={} pose={} odom={} cmd_vel={} nav_cancel={}",
        occ_msg_type.fqn(), tf_msg_type.fqn(), cm_msg_type.fqn(),
        nav_msg_type.fqn(), fr_msg_type.fqn(), pose_msg_type.fqn(),
        odom_msg_type.fqn(), cmd_vel_msg_type.fqn(), nav_cancel_msg_type.fqn(),
    );

    // 累积地图 mmap 初始化（降级允许）
    let cm_shm_path = std::path::PathBuf::from(
        std::env::var("NUWAX_CLOUDMAP_SHM_PATH")
            .unwrap_or_else(|_| "/tmp/nuwax_cloudmap.mmap".into())
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
    eprintln!("[rework18-trace] lcm_runtime: cm_writer ok, before env vars for tf task");

    // ---- 订阅 /tf：动态更新 transforms，按 TTL 裁剪陈旧边，合成 base→map 位姿 ----
    let map_frame = std::env::var("NUWAX_MAP_FRAME").unwrap_or_else(|_| "map".into());
    let base_frame = std::env::var("NUWAX_BASE_FRAME").unwrap_or_else(|_| "base_link".into());
    let base_candidates: Vec<String> = if base_frame == "base_link" || base_frame == "base_footprint" {
        vec!["base_link".into(), "base_footprint".into()]
    } else {
        vec![base_frame.clone(), "base_link".into(), "base_footprint".into()]
    };

    let tf_state = state.clone();
    let tf_transport = transport.clone();
    let tf_topic_clone = tf_topic.clone();
    let tf_msg_type_clone = tf_msg_type.clone();
    let map_frame_owned = map_frame.clone();
    let base_candidates_owned = base_candidates.clone();
    eprintln!("[rework18-trace] lcm_runtime: before tf task spawn");
    tokio::spawn(async move {
        eprintln!("[rework18-trace] tf task: spawned, before subscribe");
        let mut stream = tf_transport.subscribe(tf_topic_clone, Some(tf_msg_type_clone)).await;
        eprintln!("[rework18-trace] tf task: subscribe returned");
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
                let stamp = entry.stamp_ns;
                if let Some(existing) = s.transforms.iter_mut()
                    .find(|t| t.parent == entry.parent && t.child == entry.child) {
                    *existing = entry;
                } else {
                    s.transforms.push(entry);
                }
                if stamp > max_stamp_ns { max_stamp_ns = stamp; }
            }
            if max_stamp_ns > TF_ENTRY_TTL_NS {
                let cutoff = max_stamp_ns - TF_ENTRY_TTL_NS;
                s.transforms.retain(|t| t.stamp_ns == 0 || t.stamp_ns >= cutoff);
            }
            let base_refs: Vec<&str> = base_candidates_owned.iter().map(|s| s.as_str()).collect();
            if let Some((x, y, z, qx, qy, qz, qw, _base)) =
                resolve_map_pose(&s.transforms, &map_frame_owned, &base_refs)
            {
                s.pose.x = x; s.pose.y = y; s.pose.z = z;
                s.pose.qx = qx; s.pose.qy = qy; s.pose.qz = qz; s.pose.qw = qw;
                s.pose.frame_id = map_frame_owned.clone();
                if max_stamp_ns > 0 { s.pose.timestamp_ns = max_stamp_ns; }
            }
        }
    });

    // ---- 订阅 /odom：若上游 SLAM/odometry 节点直接发 odom，覆盖 pose ----
    let odom_state = state.clone();
    let odom_transport = transport.clone();
    // Phase 4: 改走 channel_registry::channel_for(Odom) —— 禁止再硬编码 /odom。
    let odom_topic = channel_registry::channel_for(ChannelKey::Odom);
    let odom_msg_type = LcmMessageType::new("nav_msgs/Odometry").expect("typed");
    tokio::spawn(async move {
        let mut stream = odom_transport.subscribe(odom_topic, Some(odom_msg_type)).await;
        use futures::StreamExt;
        while let Some(frame_result) = stream.next().await {
            let frame = match frame_result {
                Ok(f) => f,
                Err(e) => { error!("/odom LCM 订阅错误: {}", e); continue; }
            };
            if let Some(pose) = parse_odom_payload(&frame.payload) {
                let mut s = odom_state.write().await;
                s.pose = pose;
            }
        }
    });

    // ---- 订阅 /map/occupancy ----
    let occ_state = state.clone();
    let occ_transport = transport.clone();
    let occ_topic_clone = occ_topic.clone();
    let occ_msg_type_clone = occ_msg_type.clone();
    tokio::spawn(async move {
        let mut stream = occ_transport.subscribe(occ_topic_clone, Some(occ_msg_type_clone)).await;
        use futures::StreamExt;
        while let Some(frame_result) = stream.next().await {
            let frame = match frame_result {
                Ok(f) => f,
                Err(e) => { error!("/map/occupancy LCM 订阅错误: {}", e); continue; }
            };
            if let Some(map) = parse_occupancy_payload(&frame.payload) {
                let mut s = occ_state.write().await;
                let next = s.map.revision + 1;
                let mut map = map;
                map.revision = next;
                s.map = map;
            }
        }
    });

    // ---- 订阅 cloud_map（累积点云）----
    if let Some(cm_topic) = cm_topic {
        let cm_state = state.clone();
        let cm_tx = cloud_map_tx.clone();
        let cm_writer = cm_writer.clone();
        let cm_transport = transport.clone();
        let cm_msg_type_clone = cm_msg_type.clone();
        tokio::spawn(async move {
            let mut stream = cm_transport.subscribe(cm_topic, Some(cm_msg_type_clone)).await;
            use futures::StreamExt;
            while let Some(frame_result) = stream.next().await {
                let frame = match frame_result {
                    Ok(f) => f,
                    Err(e) => { error!("cloud_map LCM 订阅错误: {}", e); continue; }
                };
                if let Some((bytes, point_step)) = parse_pointcloud_payload(&frame.payload) {
                    // 在锁内完成 mmap 写（不持锁跨 await），拿到结果后再 await
                    let write_result = {
                        let mut guard = cm_writer.lock();
                        if let Some(writer) = guard.as_mut() {
                            writer.write_cloud_map(&bytes, point_step)
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
                            revision: rev, point_count: count, voxel_size: vs,
                        });
                    }
                }
            }
        });
    }

    // ---- 订阅 nav_state（navigation state 推送）----
    if let Some(nav_topic) = nav_topic {
        let nav_state = state.clone();
        let nav_tx = nav_state_tx.clone();
        let nav_transport = transport.clone();
        let nav_msg_type_clone = nav_msg_type.clone();
        tokio::spawn(async move {
            let mut stream = nav_transport.subscribe(nav_topic, Some(nav_msg_type_clone)).await;
            use futures::StreamExt;
            while let Some(frame_result) = stream.next().await {
                let frame = match frame_result {
                    Ok(f) => f,
                    Err(e) => { error!("nav_state LCM 订阅错误: {}", e); continue; }
                };
                // 简化：复用 odom layout（7×f64 + u64 stamp + u16 frame_len + frame + f32 distance_remaining）
                if frame.payload.len() >= 74 {
                    let status_len = u16::from_le_bytes(
                        frame.payload[64..66].try_into().unwrap_or([0, 0])) as usize;
                    if frame.payload.len() >= 66 + status_len + 4 {
                        let status = String::from_utf8_lossy(
                            &frame.payload[66..66+status_len]).to_string();
                        let cur = 66 + status_len;
                        let goal_id_len = u16::from_le_bytes(
                            frame.payload[cur..cur+2].try_into().unwrap_or([0, 0])) as usize;
                        if frame.payload.len() >= cur + 2 + goal_id_len + 8 + 4 {
                            let goal_id = String::from_utf8_lossy(
                                &frame.payload[cur+2..cur+2+goal_id_len]).to_string();
                            let stamp = u64::from_le_bytes(
                                frame.payload[cur+2+goal_id_len..cur+10+goal_id_len].try_into().unwrap_or([0;8]));
                            let dist = f32::from_le_bytes(
                                frame.payload[cur+10+goal_id_len..cur+14+goal_id_len].try_into().unwrap_or([0;4]));
                            let nav = NavState {
                                status, message: String::new(), goal_id,
                                distance_remaining: dist, timestamp_ns: stamp,
                            };
                            let mut s = nav_state.write().await;
                            s.nav_state = nav.clone();
                            let _ = nav_tx.send(nav);
                        }
                    }
                }
            }
        });
    }

    // ---- 订阅 frontiers ----
    if let Some(fr_topic) = fr_topic {
        let fr_state = state.clone();
        let fr_tx = frontiers_tx.clone();
        let fr_transport = transport.clone();
        let fr_msg_type_clone = fr_msg_type.clone();
        tokio::spawn(async move {
            let mut stream = fr_transport.subscribe(fr_topic, Some(fr_msg_type_clone)).await;
            use futures::StreamExt;
            while let Some(frame_result) = stream.next().await {
                let frame = match frame_result {
                    Ok(f) => f,
                    Err(e) => { error!("frontiers LCM 订阅错误: {}", e); continue; }
                };
                // payload: count(u32) + repeat[x,y,z(f64×3)+qx..qw(f64×4)+frame_len(u16)+frame+size(u32)+heading(f32)]
                if frame.payload.len() < 4 { continue; }
                let count = u32::from_le_bytes(frame.payload[0..4].try_into().unwrap_or([0;4])) as usize;
                // 防御性 sanity cap:见 FRONTIERS_MAX_ENTRIES_PER_FRAME 注释。
                if count > FRONTIERS_MAX_ENTRIES_PER_FRAME {
                    warn!(
                        "/explore/frontiers payload count={} 超过 sanity cap={};按异常帧跳过",
                        count, FRONTIERS_MAX_ENTRIES_PER_FRAME
                    );
                    continue;
                }
                let mut cursor = 4usize;
                let mut entries = Vec::with_capacity(count);
                let mut ok = true;
                for _ in 0..count {
                    if cursor + 56 + 2 > frame.payload.len() { ok = false; break; }
                    let x = f64::from_le_bytes(frame.payload[cursor..cursor+8].try_into().unwrap());
                    let y = f64::from_le_bytes(frame.payload[cursor+8..cursor+16].try_into().unwrap());
                    let z = f64::from_le_bytes(frame.payload[cursor+16..cursor+24].try_into().unwrap());
                    let qx = f64::from_le_bytes(frame.payload[cursor+24..cursor+32].try_into().unwrap());
                    let qy = f64::from_le_bytes(frame.payload[cursor+32..cursor+40].try_into().unwrap());
                    let qz = f64::from_le_bytes(frame.payload[cursor+40..cursor+48].try_into().unwrap());
                    let qw = f64::from_le_bytes(frame.payload[cursor+48..cursor+56].try_into().unwrap());
                    cursor += 56;
                    let f_len = u16::from_le_bytes(frame.payload[cursor..cursor+2].try_into().unwrap()) as usize;
                    cursor += 2;
                    if cursor + f_len + 4 + 4 > frame.payload.len() { ok = false; break; }
                    let frame_id = String::from_utf8_lossy(&frame.payload[cursor..cursor+f_len]).to_string();
                    cursor += f_len;
                    let size_cells = u32::from_le_bytes(frame.payload[cursor..cursor+4].try_into().unwrap());
                    cursor += 4;
                    let heading = f32::from_le_bytes(frame.payload[cursor..cursor+4].try_into().unwrap());
                    cursor += 4;
                    entries.push(FrontierEntry {
                        x, y, z, qx, qy, qz, qw, frame_id,
                        size_cells, heading,
                    });
                }
                if !ok { continue; }
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
            "mapping_adapter 启动: resolution={}m width={} height={} origin=({:?})",
            cfg.resolution, cfg.width_cells, cfg.height_cells, cfg.origin_xy
        );
        Some(Arc::new(parking_lot::Mutex::new(MappingAdapter::new(cfg))))
    } else {
        None
    };

    // Phase 3 增强:订阅 /odom 和 /go2/lidar/points 时同步把数据喂给 mapping
    // adapter。`feed_pose` / `feed_pointcloud` 内部持 parking_lot 锁同步,
    // 不会跨 await。
    if let Some(mapping_arc) = mapping_adapter.clone() {
        spawn_mapping_feed_tasks(transport.clone(), state.clone(), mapping_arc);
    }

    // Phase 3: 构造 occupancy grid SHM sidecar(默认 /tmp/nuwax_occupancy.mmap)。
    // 与 cloud_map SHM 区分:occupancy 走 grid layout(Phase 3 新增);
    // cloud_map 仍走 lcm_runtime 既有 `cm_writer`(Phase 0 已实装)。
    let occ_shm_path = std::path::PathBuf::from(
        std::env::var("NUWAX_OCCUPANCY_SHM_PATH")
            .unwrap_or_else(|_| "/tmp/nuwax_occupancy.mmap".into())
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

    Ok(GeometryRuntimeHandle {
        state,
        cloud_map_tx,
        nav_state_tx,
        frontiers_tx,
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
        nav: crate::nav_runtime::NavRuntimeState::from_env(),
        explore: crate::explore_runtime::ExploreRuntimeHandle::default_with_wavefront(),
        mapping_adapter,
        occupancy_shm,
    })
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
) {
    use futures::StreamExt;
    use nuwax_mapping::PointCloudAccumulator;
    use nalgebra::Point3;

    // ---- /odom → feed_pose ----
    // Phase 4: 改走 channel_registry::channel_for(Odom) —— 禁止硬编码 /odom。
    let odom_topic = channel_registry::channel_for(ChannelKey::Odom);
    let odom_msg_type = match LcmMessageType::new("nav_msgs/Odometry") {
        Ok(m) => m,
        Err(e) => {
            error!(
                "spawn_mapping_feed_tasks: nav_msgs/Odometry LcmMessageType 构造失败: {}",
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
    // Phase 4: 改走 channel_registry::channel_for(PointCloud) —— 禁止硬编码 /go2/lidar/points。
    let lidar_topic = channel_registry::channel_for(ChannelKey::PointCloud);
    let lidar_msg_type = match LcmMessageType::new("sensor_msgs/PointCloud2") {
        Ok(m) => m,
        Err(e) => {
            error!(
                "spawn_mapping_feed_tasks: sensor_msgs/PointCloud2 LcmMessageType 构造失败: {}",
                e
            );
            return;
        }
    };
    let transport_lidar = transport.clone();
    let mapping_lidar = mapping_arc.clone();
    let state_lidar = state.clone();
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
            let (bytes, point_step) = match parse_pointcloud_payload(&frame.payload) {
                Some(v) => v,
                None => continue,
            };
            let ps = point_step as usize;
            if ps < 12 {
                continue;
            }
            let raw_count = bytes.len() / ps;
            // 防御性 sanity cap:见 POINTCLOUD_MAX_RAW_POINTS 注释。
            // raw_count 由 untrusted bytes.len()/point_step 推得,point_step
            // 已在 parse_pointcloud_payload 入口校验过 [12, 256],但 raw_count
            // 仍可能因 bytes.len() 巨大而超 cap(>8M 点);此处直接 saturating
            // 掉,拒绝异常帧。
            if raw_count > crate::pointcloud::POINTCLOUD_MAX_RAW_POINTS {
                warn!(
                    "mapping_feed /go2/lidar/points raw_count={} 超过 sanity cap={};按异常帧跳过",
                    raw_count, crate::pointcloud::POINTCLOUD_MAX_RAW_POINTS
                );
                continue;
            }
            let mut points: Vec<Point3<f64>> = Vec::with_capacity(raw_count);
            for i in 0..raw_count {
                let off = i * ps;
                if off + 12 > bytes.len() {
                    break;
                }
                let x = f32::from_le_bytes(bytes[off..off + 4].try_into().unwrap_or([0; 4])) as f64;
                let y = f32::from_le_bytes(bytes[off + 4..off + 8].try_into().unwrap_or([0; 4])) as f64;
                let z = f32::from_le_bytes(bytes[off + 8..off + 12].try_into().unwrap_or([0; 4])) as f64;
                if x.is_nan() || y.is_nan() || z.is_nan() {
                    continue;
                }
                points.push(Point3::new(x, y, z));
            }
            if points.is_empty() {
                continue;
            }
            // 同步持锁 feed + build_and_publish(后者用 try_write 写 SharedState)
            let mut adapter = mapping_lidar.lock();
            <FeedOnlyAccumulator as PointCloudAccumulator>::feed(
                &mut adapter.accumulator,
                &points,
            );
            adapter.has_data = true;
            adapter.build_and_publish(&state_lidar);
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
    Ok(GeometryRuntimeHandle {
        state,
        cloud_map_tx,
        nav_state_tx,
        frontiers_tx,
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
    })
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
        FRONTIERS_MAX_ENTRIES_PER_FRAME, OCCUPANCY_MAX_DIMENSION,
        TF_MAX_ENTRIES_PER_FRAME,
    };

    /// count 字段填 0xFFFF_FFFF(usize 等价 ~ 18 EB)→ 必须被 sanity cap 拒绝
    #[test]
    fn parse_tf_payload_rejects_huge_count() {
        let mut payload = vec![0xFFu8, 0xFF, 0xFF, 0xFF];
        payload.extend_from_slice(&[0u8; 8]);
        assert!(
            parse_tf_payload(&payload).is_none(),
            "parse_tf_payload 应该拒绝 count > TF_MAX_ENTRIES_PER_FRAME 的异常帧"
        );
    }

    /// count 字段填 0 → 应正常解析为空 vec
    #[test]
    fn parse_tf_payload_accepts_zero_count() {
        let payload = vec![0u8, 0, 0, 0];
        let result = parse_tf_payload(&payload).expect("count=0 应可解析");
        assert!(result.is_empty());
    }

    /// width = u32::MAX → 应被 OCCUPANCY_MAX_DIMENSION 拒绝
    #[test]
    fn parse_occupancy_payload_rejects_huge_dimension() {
        let mut payload = Vec::new();
        payload.extend_from_slice(&u32::MAX.to_le_bytes()); // width
        payload.extend_from_slice(&u32::MAX.to_le_bytes()); // height
        payload.extend_from_slice(&[0u8; 30]); // padding
        assert!(
            parse_occupancy_payload(&payload).is_none(),
            "parse_occupancy_payload 应该拒绝 width/height > OCCUPANCY_MAX_DIMENSION 的异常帧"
        );
    }

    /// 合法 width*height=16 + data_len=16 happy path
    #[test]
    fn parse_occupancy_payload_accepts_well_formed() {
        let mut payload = Vec::new();
        payload.extend_from_slice(&4u32.to_le_bytes());
        payload.extend_from_slice(&4u32.to_le_bytes());
        payload.extend_from_slice(&0.05f32.to_le_bytes());
        payload.extend_from_slice(&0.0f64.to_le_bytes());
        payload.extend_from_slice(&0.0f64.to_le_bytes());
        payload.extend_from_slice(&3u16.to_le_bytes());
        payload.extend_from_slice(b"map");
        payload.extend_from_slice(&0u64.to_le_bytes());
        payload.extend_from_slice(&16u32.to_le_bytes());
        payload.extend_from_slice(&[0u8; 16]);
        let result = parse_occupancy_payload(&payload).expect("happy path 应解析成功");
        assert_eq!(result.width, 4);
        assert_eq!(result.height, 4);
        assert_eq!(result.data.len(), 16);
    }

    /// cap 常量本身在物理合理范围内(const-eval 不会触发 lint)
    #[test]
    fn sanity_caps_are_in_physically_reasonable_range() {
        // 这些都是编译期常量,clippy 会把 assert!(const_expr) 警告为可优化;
        // 这里改用 const_assertions 风格:`const _: () = assert!(...);` 是
        // 编译期验证,运行时无开销。
        const _: () = assert!(TF_MAX_ENTRIES_PER_FRAME > 0 && TF_MAX_ENTRIES_PER_FRAME < 1_000_000);
        const _: () = assert!(FRONTIERS_MAX_ENTRIES_PER_FRAME > 0 && FRONTIERS_MAX_ENTRIES_PER_FRAME < 1_000_000);
        const _: () = assert!(OCCUPANCY_MAX_DIMENSION > 0 && OCCUPANCY_MAX_DIMENSION < 100_000);
        const _: () = assert!((OCCUPANCY_MAX_DIMENSION as usize).saturating_pow(2) < 100 * 1024 * 1024);
        const _: () = assert!(crate::pointcloud::POINTCLOUD_MAX_RAW_POINTS > 0 && crate::pointcloud::POINTCLOUD_MAX_RAW_POINTS < 100_000_000);
        const _: () = assert!(crate::pointcloud::POINTCLOUD_MAX_POINT_STEP_BYTES >= 12 && crate::pointcloud::POINTCLOUD_MAX_POINT_STEP_BYTES < 4_096);
    }

    /// point_step = 1 (untrusted 上游 frame 异常)→ 必须被 sanity cap 拒绝
    /// (point_step < 12 触发)
    #[test]
    fn parse_pointcloud_payload_rejects_tiny_point_step() {
        let mut payload = Vec::new();
        payload.extend_from_slice(&1u32.to_le_bytes()); // point_step = 1 < 12
        payload.extend_from_slice(&[0u8; 16]);
        assert!(
            parse_pointcloud_payload(&payload).is_none(),
            "parse_pointcloud_payload 应该拒绝 point_step < 12 的异常帧"
        );
    }

    /// point_step = u32::MAX → 必须被 POINTCLOUD_MAX_POINT_STEP_BYTES 拒绝
    #[test]
    fn parse_pointcloud_payload_rejects_huge_point_step() {
        let mut payload = Vec::new();
        payload.extend_from_slice(&u32::MAX.to_le_bytes()); // point_step = u32::MAX > 256
        payload.extend_from_slice(&[0u8; 16]);
        assert!(
            parse_pointcloud_payload(&payload).is_none(),
            "parse_pointcloud_payload 应该拒绝 point_step > POINTCLOUD_MAX_POINT_STEP_BYTES 的异常帧"
        );
    }

    /// point_step = 12 + 24 bytes raw → 应正常解析 (happy path)
    #[test]
    fn parse_pointcloud_payload_accepts_well_formed() {
        let mut payload = Vec::new();
        payload.extend_from_slice(&12u32.to_le_bytes());
        payload.extend_from_slice(&[0u8; 24]);
        let (bytes, ps) = parse_pointcloud_payload(&payload).expect("happy path 应解析成功");
        assert_eq!(ps, 12);
        assert_eq!(bytes.len(), 24);
    }
}
