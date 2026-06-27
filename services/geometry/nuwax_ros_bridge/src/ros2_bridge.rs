use r2r::QosProfile;
use std::sync::Arc;
use std::rc::Rc;
use std::cell::RefCell;
use std::collections::{HashMap, HashSet, VecDeque};
use tokio::sync::{broadcast, RwLock};
use tokio_stream::StreamExt;
use tracing::{info, error};

use crate::pointcloud::CloudMapShmWriter;

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
    /// 该 TF 边的 header.stamp（纳秒，ROS 时间域）。0 表示无戳（latched/静态边），永不过期。
    pub stamp_ns: u64,
}

// 动态 /tf 边的存活时长（纳秒）。超过此 TTL 且带有效戳的边在合成位姿前被裁剪，
// 避免 RTAB-Map 重启/地图版本切换后陈旧 map→odom 边污染 BFS。stamp_ns==0 的边不过期。
const TF_ENTRY_TTL_NS: u64 = 5_000_000_000; // 5s

#[derive(Clone, Default)]
pub struct NavState {
    pub status: String,
    pub message: String,
    pub goal_id: String,
    pub distance_remaining: f32,
}

pub struct SharedState {
    pub pose: PoseState,
    pub map: MapState,
    pub transforms: Vec<TransformEntry>,
    pub nav_state: NavState,
    pub nav2_action_server_ready: bool,
    pub cloud_map_revision: u64,
}

impl Default for SharedState {
    fn default() -> Self {
        Self {
            pose: PoseState::default(),
            map: MapState::default(),
            transforms: Vec::new(),
            nav_state: NavState { status: "IDLE".into(), message: "空闲".into(), ..Default::default() },
            nav2_action_server_ready: false,
            cloud_map_revision: 0,
        }
    }
}


#[derive(Clone, Debug)]
pub struct CloudMapNotification {
    pub revision: u64,
    pub point_count: u32,
    pub voxel_size: f32,
}

#[derive(Clone)]
pub struct Ros2BridgeHandle {
    pub state: Arc<RwLock<SharedState>>,
    pub cloud_map_tx: broadcast::Sender<CloudMapNotification>,
}

pub fn spawn_ros2_bridge(
    map_topic: &str,
    tf_topic: &str,
    cloud_map_topic: Option<&str>,
) -> Result<Ros2BridgeHandle, Box<dyn std::error::Error>> {
    let (cloud_map_tx, _) = broadcast::channel::<CloudMapNotification>(4);
    let cloud_map_tx_clone = cloud_map_tx.clone();
    let state = Arc::new(RwLock::new(SharedState::default()));
    let state_clone = state.clone();

    let map_topic = map_topic.to_string();
    let tf_topic = tf_topic.to_string();
    let cm_topic = cloud_map_topic.map(|s| s.to_string());

    std::thread::spawn(move || {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("创建 tokio runtime 失败");
        let local = tokio::task::LocalSet::new();
        rt.block_on(local.run_until(async move {
            if let Err(e) = ros2_async_loop(
                &map_topic, &tf_topic,
                state_clone,
                cm_topic.as_deref(), cloud_map_tx_clone,
            ).await {
                error!("ROS2 桥接线程异常退出: {}", e);
            }
        }));
    });

    Ok(Ros2BridgeHandle { state, cloud_map_tx })
}

// ===== tf2 坐标链合成（map→odom→base_link）=====
// 把存储的 TransformEntry 列表当作图，BFS 解析 source→target 的刚体变换。
// 算法移植自 services/localization/frame_tree_ops.py，保持与 Python 侧一致。

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

// 刚体变换的逆：[R|t]^-1 = [R^T | -R^T·t]
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

// BFS 解析 source→target 坐标变换，返回把 source 坐标映射到 target 坐标的 4x4 矩阵。
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
        ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s)
    }
}

// 从 transforms 解析机器人在 map 坐标系下的位姿（base_link 原点在 map 中的位置）。
// 返回 (x, y, z, qx, qy, qz, qw, resolved_base_frame)；链路不完整时返回 None。
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
// __HELPERS_END__

async fn ros2_async_loop(
    map_topic: &str,
    tf_topic: &str,
    state: Arc<RwLock<SharedState>>,
    cloud_map_topic: Option<&str>,
    cloud_map_tx: broadcast::Sender<CloudMapNotification>,
) -> Result<(), Box<dyn std::error::Error>> {
    let cm_shm_path = std::path::PathBuf::from(
        std::env::var("NUWAX_CLOUDMAP_SHM_PATH")
            .unwrap_or_else(|_| "/tmp/nuwax_cloudmap.mmap".into())
    );
    let cm_shm_writer: Rc<RefCell<Option<CloudMapShmWriter>>> = Rc::new(RefCell::new(
        match CloudMapShmWriter::new(&cm_shm_path) {
            Ok(w) => Some(w),
            Err(e) => {
                error!("累积地图 mmap 初始化失败（降级为无 cloud_map 模式）: {}", e);
                None
            }
        }
    ));

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "nuwax_ros_bridge", "")?;

    let map_sub = node.subscribe::<r2r::nav_msgs::msg::OccupancyGrid>(
        map_topic, QosProfile::default()
    )?;
    let tf_sub = node.subscribe::<r2r::tf2_msgs::msg::TFMessage>(
        tf_topic, QosProfile::default()
    )?;

    info!("ROS2 桥接启动: map={}, tf={}", map_topic, tf_topic);

    // 订阅占据栅格（默认 /map_obstacles）→ 更新 SharedState.map
    let map_state = state.clone();
    tokio::task::spawn_local(async move {
        let mut stream = map_sub;
        while let Some(msg) = stream.next().await {
            let mut s = map_state.write().await;
            s.map.width = msg.info.width;
            s.map.height = msg.info.height;
            s.map.resolution = msg.info.resolution;
            s.map.origin_x = msg.info.origin.position.x;
            s.map.origin_y = msg.info.origin.position.y;
            s.map.frame_id = msg.header.frame_id.clone();
            s.map.revision += 1;
            s.map.data = msg.data;
        }
    });


    // 订阅累积地图话题
    if let Some(cm_topic) = cloud_map_topic {
        let cm_sub = node.subscribe::<r2r::sensor_msgs::msg::PointCloud2>(
            cm_topic, QosProfile::default()
        )?;
        let cm_writer = cm_shm_writer.clone();
        let cm_state = state.clone();
        let cm_tx = cloud_map_tx.clone();
        info!("订阅累积地图: {}", cm_topic);
        tokio::task::spawn_local(async move {
            let mut stream = cm_sub;
            while let Some(msg) = stream.next().await {
                if let Some(ref mut writer) = *cm_writer.borrow_mut() {
                    if let Some((rev, count, vs)) = writer.write_cloud_map(&msg.data, msg.point_step) {
                        let mut s = cm_state.write().await;
                        s.cloud_map_revision = rev;
                        drop(s);
                        let _ = cm_tx.send(CloudMapNotification {
                            revision: rev, point_count: count, voxel_size: vs,
                        });
                    }
                }
            }
        });
    }

    // 订阅 /tf → 更新 SharedState.transforms，并通过 tf2 链合成 map 系下的机器人位姿。
    let map_frame = std::env::var("NUWAX_MAP_FRAME").unwrap_or_else(|_| "map".into());
    let base_frame = std::env::var("NUWAX_BASE_FRAME").unwrap_or_else(|_| "base_link".into());
    let tf_state = state.clone();
    tokio::task::spawn_local(async move {
        let base_candidates: Vec<&str> = if base_frame == "base_link" || base_frame == "base_footprint" {
            vec!["base_link", "base_footprint"]
        } else {
            vec![base_frame.as_str(), "base_link", "base_footprint"]
        };
        let mut stream = tf_sub;
        while let Some(msg) = stream.next().await {
            let mut s = tf_state.write().await;
            let mut max_stamp_ns: u64 = 0;
            for tf in &msg.transforms {
                let stamp_ns = (tf.header.stamp.sec as u64) * 1_000_000_000 + (tf.header.stamp.nanosec as u64);
                let entry = TransformEntry {
                    parent: tf.header.frame_id.clone(),
                    child: tf.child_frame_id.clone(),
                    tx: tf.transform.translation.x,
                    ty: tf.transform.translation.y,
                    tz: tf.transform.translation.z,
                    rx: tf.transform.rotation.x,
                    ry: tf.transform.rotation.y,
                    rz: tf.transform.rotation.z,
                    rw: tf.transform.rotation.w,
                    stamp_ns,
                };
                if let Some(existing) = s.transforms.iter_mut().find(|t| t.parent == entry.parent && t.child == entry.child) {
                    *existing = entry;
                } else {
                    s.transforms.push(entry);
                }
                if stamp_ns > max_stamp_ns { max_stamp_ns = stamp_ns; }
            }
            // TTL 裁剪：以"已见最新戳"为参考时间（ROS 时间域），丢弃带戳且超过 TTL 的陈旧动态边；
            // stamp_ns==0（无戳/latched）的边视为静态、永不过期。这同时给 transforms 长度上界，
            // 并阻止 RTAB-Map 重启遗留的旧 map→odom 边参与下面的 BFS 合成。
            if max_stamp_ns > TF_ENTRY_TTL_NS {
                let cutoff = max_stamp_ns - TF_ENTRY_TTL_NS;
                s.transforms.retain(|t| t.stamp_ns == 0 || t.stamp_ns >= cutoff);
            }
            // 合成 base→map 链，得到机器人在 map 系下的位姿。链路不完整时保留上次有效值。
            if let Some((x, y, z, qx, qy, qz, qw, _base)) =
                resolve_map_pose(&s.transforms, &map_frame, &base_candidates)
            {
                s.pose.x = x; s.pose.y = y; s.pose.z = z;
                s.pose.qx = qx; s.pose.qy = qy; s.pose.qz = qz; s.pose.qw = qw;
                s.pose.frame_id = map_frame.clone();
                if max_stamp_ns > 0 { s.pose.timestamp_ns = max_stamp_ns; }
            }
        }
    });

    // 主循环：spin node
    loop {
        node.spin_once(std::time::Duration::from_millis(5));
        tokio::task::yield_now().await;
    }
}
