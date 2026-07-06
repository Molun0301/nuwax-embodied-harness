//! LCM-first geometry runtime state loop。
//!
//! 该模块替换原 `ros2_bridge.rs`：不再订阅 ROS2 topic，而是通过
//! `nuwax_transport::LcmTransport` 订阅 LCM typed channel，将 typed frame
//! 累积到 `SharedState`，供 gRPC facade（`server.rs`）查询/订阅。
//!
//! ## backend 策略
//! - 默认走 `LcmTransport::new_fake()`（CI/开发机不依赖 liblcm）。
//! - 可通过 `NUWAX_LCM_BACKEND` 环境变量显式选择：
//!   - `fake`(默认)：内存 fake backend,不依赖 liblcm
//!   - `real`：real-lcm backend；**当前上游 `nuwax-transport/real-lcm` feature 仍为空桩,
//!     尚未实装 Rust LCM binding + MurmurHash64A fingerprint 切换**,
//!     详见 `docs/开发文档/建图导航探索重构/2026-07-01_彻底去ROS2化-执行方案/06_LCM接入与wire-format验证.md`。
//!     本期 `real` 选择会被强制 fallback 到 fake 并打印 warn。
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

// ============================================================================
// 状态定义（与原 ros2_bridge.rs 同结构，便于 server.rs 不感知 backend 切换）
// ============================================================================

/// Phase 1 fail-fast 错误：上游 dimos `nuwax-lcm-sys`/`nuwax-lcm` 尚不存在
///（`nuwax-transport/real-lcm` feature 仍为空桩），所以 `real-lcm` backend
/// 启动期直接拒绝；不允许 fallback fake-lcm。错误码 `LCM_BACKEND_UNAVAILABLE`
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
/// 以及通过 transport.publish 推送 odom / point cloud（fake backend 直接
/// 写入内存 topic;real backend 在上游 real-lcm feature 实装后透传）。
#[derive(Clone)]
pub struct GeometryRuntimeHandle {
    pub state: Arc<RwLock<SharedState>>,
    pub cloud_map_tx: broadcast::Sender<CloudMapNotification>,
    pub nav_state_tx: broadcast::Sender<NavState>,
    pub frontiers_tx: broadcast::Sender<Vec<FrontierEntry>>,
    pub transport: LcmTransport,
    /// canonical LCM backend 名（HealthCheck / 日志 / diagnostics 用）：
    /// - `fake-lcm` — 内存 fake（CI 默认）
    /// - `real-lcm-unavailable` — 用户请求 real-lcm 但上游 blocker 命中
    pub transport_backend_name: &'static str,
    /// backend 是否真正可用。当 `false` 时 server.rs 的 HealthCheck 必须
    /// 返回 `healthy=false` + `LCM_BACKEND_UNAVAILABLE` message；
    /// 该 flag 当前唯一触发源是 `LcmBackendKind::Real`（fail-fast 设计）。
    pub lcm_backend_available: bool,
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
    /// `Arc<parking_lot::Mutex<MappingAdapter>>`:
    /// - `Arc` 允许多订阅任务共享
    /// - `parking_lot::Mutex` 走同步路径,避免 async 持锁跨 await
    pub mapping_adapter: Option<Arc<parking_lot::Mutex<MappingAdapter>>>,
    /// Phase 3: occupancy grid SHM sidecar 写入器。
    /// - `None` —— backend 不可用 / LCM fail-fast 路径不构造
    /// - `Some` —— `get_occupancy_grid_shm` 走 SHM 直读
    pub occupancy_shm: Option<Arc<parking_lot::Mutex<OccupancyGridShmWriter>>>,
}

impl GeometryRuntimeHandle {
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

    /// exploration 板块 readiness。Phase 2 baseline:恒 `false`（controller
    /// 未实装）。Phase 5 在 explore_runtime.rs 实装后,只要 handle 存在
    /// (`Spawn` 时已经构造默认 wavefront source)即视为 ready;
    /// 详细 active/inactive 由 `ExploreRuntimeHandle::is_active` 暴露。
    pub fn explore_ready_poll(&self) -> bool {
        // 当前实现:handle 在 spawn 时必已注入,仅在 spawn 失败时为 None
        // (该路径上 LCM backend 阻断,会先报 LCM_BACKEND_UNAVAILABLE)。
        true
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
    for i in 0..3 { for j in 0..3 { o[i][j] = m[j][i]; } }
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

fn resolve_map_pose(
    transforms: &[TransformEntry],
    map_frame: &str,
    base_frames: &[&str],
) -> Option<(f64, f64, f64, f64, f64, f64, f64, String)> {
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

fn parse_tf_payload(payload: &[u8]) -> Option<Vec<TransformEntry>> {
    // count(u32) + repeat[parent_len(u16)+parent+child_len(u16)+child+7×f64+stamp(u64)]
    if payload.len() < 4 { return None; }
    let count = u32::from_le_bytes(payload[0..4].try_into().ok()?) as usize;
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
    Some((payload[4..].to_vec(), point_step))
}

// ============================================================================
// 启动：构造 LcmTransport (fake) + SharedState + 各订阅 task
// ============================================================================

/// LCM backend 选择。基于 `NUWAX_LCM_BACKEND` 环境变量解析：
/// - `fake`(默认)：内存 fake backend,不依赖 liblcm
/// - `real`：real-lcm backend；**当前上游 nuwax-transport/real-lcm feature 仍为空桩**,
///   启动期强制 fallback 到 fake 并打印 warn,避免误传 `real` 时偷偷走空 feature。
/// 详见模块顶部 doc-comment 与
/// `docs/开发文档/建图导航探索重构/2026-07-01_彻底去ROS2化-执行方案/06_LCM接入与wire-format验证.md`。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LcmBackendKind {
    /// 内存 fake backend（CI/开发机默认）
    Fake,
    /// 真实 liblcm backend；当前上游未实装,启动期 fallback 到 Fake
    Real,
}

impl LcmBackendKind {
    fn from_env() -> Self {
        // Phase 0 收敛：canonical = `real-lcm` / `fake-lcm`；`real` / `fake` 为
        // 迁移 alias，归一化到 canonical 并打 warn。其他值（空 / 未知）默认
        // 走 Fake backend。
        let raw = std::env::var("NUWAX_LCM_BACKEND").unwrap_or_default();
        let normalized = raw.trim().to_ascii_lowercase();
        let canonical = match normalized.as_str() {
            "real-lcm" | "real" => {
                if normalized == "real" {
                    tracing::warn!(
                        "NUWAX_LCM_BACKEND=real 是迁移 alias；已归一化为 real-lcm。后续请直接传 canonical 名。"
                    );
                }
                "real-lcm"
            }
            "fake-lcm" | "fake" | "" => {
                if normalized == "fake" {
                    tracing::warn!(
                        "NUWAX_LCM_BACKEND=fake 是迁移 alias；已归一化为 fake-lcm。后续请直接传 canonical 名。"
                    );
                }
                "fake-lcm"
            }
            other => {
                tracing::warn!(
                    "NUWAX_LCM_BACKEND={:?} 无法识别；归一化为 fake-lcm。canonical 取值只允许 real-lcm / fake-lcm。",
                    other
                );
                "fake-lcm"
            }
        };
        match canonical {
            "real-lcm" => Self::Real,
            // 默认/未知/显式 fake 都走 Fake backend
            _ => Self::Fake,
        }
    }
}

/// 读 `NUWAX_REAL_LCM_REQUIRED`：true/false/1/0/yes/no/on/off (大小写不敏感)。
///
/// 当 `NUWAX_REAL_LCM_REQUIRED=true` + 选择 real-lcm backend 但 real-lcm 上游 blocker
/// 命中（feature 仍空桩 + fingerprint 未切 MurmurHash64A）时，启动期进程直接
/// exit 1，**不**在 binary 端静默 fallback 到 fake-lcm；运行期故障仍只走
/// `HealthCheck healthy=false` 路径，不重新触发 exit。
///
/// 默认 false；与 `core/settings.py::REAL_LCM_REQUIRED` 语义保持对齐（Python 端
/// 也是 fail-fast 设计，但 Python 不执进程退出）。
pub fn real_lcm_required_from_env() -> bool {
    let raw = std::env::var("NUWAX_REAL_LCM_REQUIRED").unwrap_or_default();
    matches!(
        raw.trim().to_ascii_lowercase().as_str(),
        "1" | "true" | "yes" | "on"
    )
}

/// 启动 geometry runtime 异步循环（LCM-first）。
///
/// 当前默认走 fake backend；`NUWAX_LCM_BACKEND=real-lcm` 启动期直接
/// fail-fast 返回 [`RealLcmBackendUnavailable`]（不在 fake 路径静默 fallback）：
/// 上游 dimos `nuwax-transport/real-lcm` feature 仍为空桩 + fingerprint 仍
/// 是 FNV-1a（非 lcm-gen MurmurHash64A），属于 dependency mismatch blocker。
/// 详见 feedback_phase1.md §2。
///
/// P1 实装 Rust LCM binding + MurmurHash64A fingerprint 切换后,
/// 在 `LcmBackendKind::Real` 分支补 `LcmTransport::new_real(url)` 调用即可,
/// 无需改订阅语义。
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
    // Phase 1 fail-fast:real-lcm 请求启动期直接拒绝；不允许 fallback fake-lcm。
    // Phase 5 加强（codex 反馈 P2）：当 `NUWAX_REAL_LCM_REQUIRED=true` 命中时，
    // 进程直接 `process::exit(1)`（不再只 `return Err(...)` 让 main 软退出），
    // 防止上游 systemd / start_gateway 把 binary restart-loop 仍以"启动期失败"
    // 状态回写 ready_file，从而绕开生产意图。运行期故障仍仅走 HealthCheck
    // healthy=false 路径，**不**重复触发 exit。
    let real_lcm_required = real_lcm_required_from_env();
    let (transport, backend_name, backend_available): (LcmTransport, &'static str, bool) =
        match LcmBackendKind::from_env() {
            LcmBackendKind::Fake => {
                info!("LCM backend 选择: fake（默认；不依赖 liblcm）");
                (LcmTransport::new_fake(), "fake-lcm", true)
            }
            LcmBackendKind::Real => {
                // dependency mismatch blocker:
                //   - dimos workspace 无 `nuwax-lcm-sys` / `nuwax-lcm` crate
                //   - `nuwax-transport/real-lcm` feature 仍为 `[]` 空桩
                //   - fingerprint 当前 = FNV-1a 64-bit,非 lcm-gen MurmurHash64A
                // 处置:启动期直接返回 RealLcmBackendUnavailable,main.rs 立即退出。
                // **绝不**在 Real 失败路径调用 `LcmTransport::new_fake()`。
                //
                // Phase 5 (codex-P2) 进一步要求：
                // 当 `NUWAX_REAL_LCM_REQUIRED=true` 时，进程**硬退出**（exit code = 1），
                // 错误信息仍保留 stderr `LCM_BACKEND_UNAVAILABLE` 关键字
                // 供 start 脚本与 smoke test grep 识别。
                if real_lcm_required {
                    eprintln!(
                        "LCM_BACKEND_UNAVAILABLE: NUWAX_REAL_LCM_REQUIRED=true 时 real-lcm \
                         backend 上游 dimos nuwax-transport/real-lcm feature 仍为空桩，\
                         按生产意图立即硬退出 (exit code = 1)。"
                    );
                    std::process::exit(1);
                }
                return Err(RealLcmBackendUnavailable);
            }
        };
    info!(
        "nuwax-geometry-runtime 启动：LCM backend = {} (available={})",
        backend_name, backend_available
    );

    let occ_topic = LcmTopic::new(occupancy_channel)
        .unwrap_or_else(|_| LcmTopic::new("/map/occupancy").expect("default topic"));
    let tf_topic = LcmTopic::new(tf_channel)
        .unwrap_or_else(|_| LcmTopic::new("/tf").expect("default topic"));
    let cm_topic = cloud_map_channel.map(LcmTopic::new).transpose().ok().flatten();
    let nav_topic = nav_state_channel.map(LcmTopic::new).transpose().ok().flatten();
    let fr_topic = frontiers_channel.map(LcmTopic::new).transpose().ok().flatten();
    // Phase 1 channel registry 补齐:解析 4 类 typed channel;若 LcmTopic 校验失败
    // (例如 typed suffix 缺失)回退到 default string,然后降级为 None。
    let pose_topic = pose_channel
        .map(|c| LcmTopic::new(c).unwrap_or_else(|_| LcmTopic::new("/pose_stamped").expect("default")))
        .or_else(|| LcmTopic::new("/pose_stamped").ok());
    let odom_topic = odometry_channel
        .map(|c| LcmTopic::new(c).unwrap_or_else(|_| LcmTopic::new("/odom").expect("default")))
        .or_else(|| LcmTopic::new("/odom").ok());
    let cmd_vel_topic = cmd_vel_channel
        .map(|c| LcmTopic::new(c).unwrap_or_else(|_| LcmTopic::new("/cmd_vel").expect("default")))
        .or_else(|| LcmTopic::new("/cmd_vel").ok());
    let nav_cancel_topic = nav_cancel_channel
        .map(|c| LcmTopic::new(c).unwrap_or_else(|_| LcmTopic::new("/nav/cancel").expect("default")))
        .or_else(|| LcmTopic::new("/nav/cancel").ok());

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
    tokio::spawn(async move {
        let mut stream = tf_transport.subscribe(tf_topic_clone, Some(tf_msg_type_clone)).await;
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
    let odom_topic = LcmTopic::new("/odom").expect("default");
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
    let odom_topic = match LcmTopic::new("/odom") {
        Ok(t) => t,
        Err(e) => {
            error!("spawn_mapping_feed_tasks: /odom LcmTopic 构造失败: {}", e);
            return;
        }
    };
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
    let lidar_topic = match LcmTopic::new("/go2/lidar/points") {
        Ok(t) => t,
        Err(e) => {
            error!(
                "spawn_mapping_feed_tasks: /go2/lidar/points LcmTopic 构造失败: {}",
                e
            );
            return;
        }
    };
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

/// 单元测试 helper:构造一个最简 `GeometryRuntimeHandle`。
///
/// 不会启动订阅任务,只提供 fake backend + 全新 `SharedState`,供
/// `nav_runtime` / `mapping_adapter` / 其它模块做集成测试。
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
    let transport = LcmTransport::new_fake();
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
        transport_backend_name: "fake-lcm",
        lcm_backend_available: true,
        nav: crate::nav_runtime::NavRuntimeState::from_env(),
        explore: crate::explore_runtime::ExploreRuntimeHandle::default_with_wavefront(),
        mapping_adapter,
        occupancy_shm,
    })
}