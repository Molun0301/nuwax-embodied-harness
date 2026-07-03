//! LCM-first geometry runtime state loop。
//!
//! 该模块替换原 `ros2_bridge.rs`：不再订阅 ROS2 topic，而是通过
//! `nuwax_transport::LcmTransport` 订阅 LCM typed channel，将 typed frame
//! 累积到 `SharedState`，供 gRPC facade（`server.rs`）查询/订阅。
//!
//! ## backend 策略
//! - P0 默认走 `LcmTransport::new_fake()`（CI/开发机不依赖 liblcm）。
//! - P1（`real-lcm` feature）替换为 `RealLcmBackend`，由 `nuwax-workspace`
//!   `real-lcm` 实装完成后切换；不允许回退 ROS2。
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
use tokio::sync::{broadcast, RwLock};
use tracing::{error, info, warn};

use crate::pointcloud::CloudMapShmWriter;

// ============================================================================
// 状态定义（与原 ros2_bridge.rs 同结构，便于 server.rs 不感知 backend 切换）
// ============================================================================

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

/// 几何 runtime 句柄。`server.rs` 通过此句柄读 SharedState、订阅 cloud_map_tx。
#[derive(Clone)]
pub struct GeometryRuntimeHandle {
    pub state: Arc<RwLock<SharedState>>,
    pub cloud_map_tx: broadcast::Sender<CloudMapNotification>,
    pub nav_state_tx: broadcast::Sender<NavState>,
    pub frontiers_tx: broadcast::Sender<Vec<FrontierEntry>>,
    pub transport_backend_name: &'static str,
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

/// 启动 geometry runtime 异步循环（LCM-first）。当前使用 fake backend；
/// P1 实装 `real-lcm` feature 后，构造 `LcmTransport::new(real_lcm_backend)`
/// 即可，无需改订阅语义。
pub fn spawn_geometry_runtime(
    occupancy_channel: &str,
    tf_channel: &str,
    cloud_map_channel: Option<&str>,
    nav_state_channel: Option<&str>,
    frontiers_channel: Option<&str>,
) -> GeometryRuntimeHandle {
    let (cloud_map_tx, _) = broadcast::channel::<CloudMapNotification>(4);
    let (nav_state_tx, _) = broadcast::channel::<NavState>(8);
    let (frontiers_tx, _) = broadcast::channel::<Vec<FrontierEntry>>(4);

    let state = Arc::new(RwLock::new(SharedState::default()));

    // P0 默认 fake backend；real-lcm feature 由 P1 切换
    let transport = LcmTransport::new_fake();
    let backend_name: &'static str = "fake-lcm"; // 与 FakeLcmBackend::name() 对齐
    info!("nuwax-geometry-runtime 启动：LCM backend = {}", backend_name);

    let occ_topic = LcmTopic::new(occupancy_channel)
        .unwrap_or_else(|_| LcmTopic::new("/map/occupancy").expect("default topic"));
    let tf_topic = LcmTopic::new(tf_channel)
        .unwrap_or_else(|_| LcmTopic::new("/tf").expect("default topic"));
    let cm_topic = cloud_map_channel.map(LcmTopic::new).transpose().ok().flatten();
    let nav_topic = nav_state_channel.map(LcmTopic::new).transpose().ok().flatten();
    let fr_topic = frontiers_channel.map(LcmTopic::new).transpose().ok().flatten();

    let occ_msg_type = LcmMessageType::new("nav_msgs/OccupancyGrid").expect("typed");
    let tf_msg_type = LcmMessageType::new("geometry_msgs/Transform").expect("typed");
    let cm_msg_type = LcmMessageType::new("sensor_msgs/PointCloud2").expect("typed");
    let nav_msg_type = LcmMessageType::new("nuwax_msgs/NavigationState").expect("typed");
    let fr_msg_type = LcmMessageType::new("nuwax_msgs/FrontierList").expect("typed");

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

    GeometryRuntimeHandle {
        state,
        cloud_map_tx,
        nav_state_tx,
        frontiers_tx,
        transport_backend_name: backend_name,
    }
}