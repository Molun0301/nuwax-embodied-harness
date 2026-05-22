use r2r::QosProfile;
use std::sync::Arc;
use std::rc::Rc;
use std::cell::RefCell;
use tokio::sync::{mpsc, oneshot, RwLock};
use tokio_stream::StreamExt;
use tracing::{info, error};

use crate::pointcloud::{PointCloudProcessor, PointCloudShmWriter};

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
}

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
}

impl Default for SharedState {
    fn default() -> Self {
        Self {
            pose: PoseState::default(),
            map: MapState::default(),
            transforms: Vec::new(),
            nav_state: NavState { status: "IDLE".into(), message: "空闲".into(), ..Default::default() },
            nav2_action_server_ready: false,
        }
    }
}

pub enum BridgeCommand {
    PublishOdom {
        frame_id: String,
        child_frame_id: String,
        x: f64, y: f64, z: f64,
        qx: f64, qy: f64, qz: f64, qw: f64,
        lvx: f64, lvy: f64, lvz: f64,
        avx: f64, avy: f64, avz: f64,
        reply: oneshot::Sender<Result<(), String>>,
    },
    PublishPointCloud {
        frame_id: String,
        data: Vec<u8>,
        point_count: u32,
        point_step: u32,
        reply: oneshot::Sender<Result<(), String>>,
    },
    SendNavGoal {
        x: f64, y: f64, z: f64,
        qx: f64, qy: f64, qz: f64, qw: f64,
        reply: oneshot::Sender<Result<String, String>>,
    },
    CancelNav {
        reply: oneshot::Sender<Result<(), String>>,
    },
}

#[derive(Clone)]
pub struct Ros2BridgeHandle {
    tx: mpsc::Sender<BridgeCommand>,
    pub state: Arc<RwLock<SharedState>>,
}

impl Ros2BridgeHandle {
    pub async fn publish_odom(
        &self, frame_id: &str, child_frame_id: &str,
        x: f64, y: f64, z: f64,
        qx: f64, qy: f64, qz: f64, qw: f64,
        lvx: f64, lvy: f64, lvz: f64,
        avx: f64, avy: f64, avz: f64,
    ) -> Result<(), String> {
        let (reply_tx, reply_rx) = oneshot::channel();
        self.tx.send(BridgeCommand::PublishOdom {
            frame_id: frame_id.into(), child_frame_id: child_frame_id.into(),
            x, y, z, qx, qy, qz, qw, lvx, lvy, lvz, avx, avy, avz,
            reply: reply_tx,
        }).await.map_err(|_| "桥接线程已退出".to_string())?;
        reply_rx.await.map_err(|_| "桥接线程未响应".to_string())?
    }

    pub async fn publish_pointcloud(
        &self, frame_id: &str, data: Vec<u8>, point_count: u32, point_step: u32,
    ) -> Result<(), String> {
        let (reply_tx, reply_rx) = oneshot::channel();
        self.tx.send(BridgeCommand::PublishPointCloud {
            frame_id: frame_id.into(), data, point_count, point_step,
            reply: reply_tx,
        }).await.map_err(|_| "桥接线程已退出".to_string())?;
        reply_rx.await.map_err(|_| "桥接线程未响应".to_string())?
    }

    pub async fn send_nav_goal(
        &self, x: f64, y: f64, z: f64, qx: f64, qy: f64, qz: f64, qw: f64,
    ) -> Result<String, String> {
        let (reply_tx, reply_rx) = oneshot::channel();
        self.tx.send(BridgeCommand::SendNavGoal {
            x, y, z, qx, qy, qz, qw, reply: reply_tx,
        }).await.map_err(|_| "桥接线程已退出".to_string())?;
        reply_rx.await.map_err(|_| "桥接线程未响应".to_string())?
    }

    pub async fn cancel_nav(&self) -> Result<(), String> {
        let (reply_tx, reply_rx) = oneshot::channel();
        self.tx.send(BridgeCommand::CancelNav { reply: reply_tx })
            .await.map_err(|_| "桥接线程已退出".to_string())?;
        reply_rx.await.map_err(|_| "桥接线程未响应".to_string())?
    }
}

pub fn spawn_ros2_bridge(
    odom_topic: &str,
    pointcloud_topic: &str,
    map_topic: &str,
    tf_topic: &str,
    nav_action: &str,
    odom_source_topic: Option<&str>,
    pointcloud_source_topic: Option<&str>,
) -> Result<Ros2BridgeHandle, Box<dyn std::error::Error>> {
    let (tx, rx) = mpsc::channel::<BridgeCommand>(256);
    let state = Arc::new(RwLock::new(SharedState::default()));
    let state_clone = state.clone();

    let odom_topic = odom_topic.to_string();
    let pc_topic = pointcloud_topic.to_string();
    let map_topic = map_topic.to_string();
    let tf_topic = tf_topic.to_string();
    let nav_action = nav_action.to_string();
    let odom_src = odom_source_topic.map(|s| s.to_string());
    let pc_src = pointcloud_source_topic.map(|s| s.to_string());

    let pc_shm_path = std::path::PathBuf::from(
        std::env::var("NUWAX_POINTCLOUD_SHM_PATH")
            .unwrap_or_else(|_| "/tmp/nuwax_pointcloud.shm".into())
    );

    std::thread::spawn(move || {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("创建 tokio runtime 失败");
        let local = tokio::task::LocalSet::new();
        rt.block_on(local.run_until(async move {
            if let Err(e) = ros2_async_loop(
                &odom_topic, &pc_topic, &map_topic, &tf_topic, &nav_action,
                rx, state_clone, &pc_shm_path,
                odom_src.as_deref(), pc_src.as_deref(),
            ).await {
                error!("ROS2 桥接线程异常退出: {}", e);
            }
        }));
    });

    Ok(Ros2BridgeHandle { tx, state })
}

async fn ros2_async_loop(
    odom_topic: &str,
    pc_topic: &str,
    map_topic: &str,
    tf_topic: &str,
    nav_action: &str,
    mut rx: mpsc::Receiver<BridgeCommand>,
    state: Arc<RwLock<SharedState>>,
    pc_shm_path: &std::path::Path,
    odom_source_topic: Option<&str>,
    pointcloud_source_topic: Option<&str>,
) -> Result<(), Box<dyn std::error::Error>> {
    // 初始化点云共享内存写入器
    let pc_shm_writer: Rc<RefCell<Option<PointCloudShmWriter>>> = Rc::new(RefCell::new(
        match PointCloudShmWriter::new(pc_shm_path) {
            Ok(w) => Some(w),
            Err(e) => {
                error!("点云共享内存初始化失败（降级为无 SHM 模式）: {}", e);
                None
            }
        }
    ));

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "nuwax_ros_bridge", "")?;

    let odom_pub = node.create_publisher::<r2r::nav_msgs::msg::Odometry>(
        odom_topic, QosProfile::default()
    )?;
    let pc_pub = node.create_publisher::<r2r::sensor_msgs::msg::PointCloud2>(
        pc_topic, QosProfile::default()
    )?;

    let map_sub = node.subscribe::<r2r::nav_msgs::msg::OccupancyGrid>(
        map_topic, QosProfile::default()
    )?;
    let tf_sub = node.subscribe::<r2r::tf2_msgs::msg::TFMessage>(
        tf_topic, QosProfile::default()
    )?;

    let nav_client = node.create_action_client::<r2r::nav2_msgs::action::NavigateToPose::Action>(
        nav_action
    )?;

    info!("ROS2 桥接启动: odom={}, map={}, tf={}, nav={}", odom_topic, map_topic, tf_topic, nav_action);

    // 直接订阅 Go2 原生话题（通过 CycloneDDS RMW 可见）
    if let Some(src) = odom_source_topic {
        let odom_src_sub = node.subscribe::<r2r::nav_msgs::msg::Odometry>(
            src, QosProfile::default()
        )?;
        let odom_pub_clone = odom_pub.clone();
        let odom_state = state.clone();
        info!("直接订阅 Go2 里程计: {}", src);
        tokio::task::spawn_local(async move {
            let mut stream = odom_src_sub;
            while let Some(msg) = stream.next().await {
                let mut s = odom_state.write().await;
                s.pose.x = msg.pose.pose.position.x;
                s.pose.y = msg.pose.pose.position.y;
                s.pose.z = msg.pose.pose.position.z;
                s.pose.qx = msg.pose.pose.orientation.x;
                s.pose.qy = msg.pose.pose.orientation.y;
                s.pose.qz = msg.pose.pose.orientation.z;
                s.pose.qw = msg.pose.pose.orientation.w;
                s.pose.frame_id = msg.header.frame_id.clone();
                drop(s);
                let _ = odom_pub_clone.publish(&msg);
            }
        });
    }

    if let Some(src) = pointcloud_source_topic {
        let pc_src_sub = node.subscribe::<r2r::sensor_msgs::msg::PointCloud2>(
            src, QosProfile::default()
        )?;
        let pc_pub_clone = pc_pub.clone();
        let pc_processor = PointCloudProcessor::default_go2();
        let pc_shm_for_sub = pc_shm_writer.clone();
        info!("直接订阅 Go2 点云: {}", src);
        tokio::task::spawn_local(async move {
            let mut stream = pc_src_sub;
            while let Some(msg) = stream.next().await {
                let (processed_data, processed_count) = pc_processor.process(
                    &msg.data, msg.point_step
                );
                if processed_count == 0 { continue; }
                if let Some(ref mut writer) = *pc_shm_for_sub.borrow_mut() {
                    writer.write_frame(&processed_data, processed_count);
                }
                let out_step = 12u32;
                let mut out_msg = r2r::sensor_msgs::msg::PointCloud2::default();
                out_msg.header = msg.header.clone();
                out_msg.height = 1;
                out_msg.width = processed_count;
                out_msg.point_step = out_step;
                out_msg.row_step = out_step * processed_count;
                out_msg.is_dense = true;
                out_msg.data = processed_data;
                out_msg.fields = vec![
                    r2r::sensor_msgs::msg::PointField { name: "x".into(), offset: 0, datatype: 7, count: 1 },
                    r2r::sensor_msgs::msg::PointField { name: "y".into(), offset: 4, datatype: 7, count: 1 },
                    r2r::sensor_msgs::msg::PointField { name: "z".into(), offset: 8, datatype: 7, count: 1 },
                ];
                let _ = pc_pub_clone.publish(&out_msg);
            }
        });
    }

    // 订阅 /map → 更新 SharedState.map
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

    // 订阅 /tf → 更新 SharedState.transforms + pose
    let tf_state = state.clone();
    tokio::task::spawn_local(async move {
        let mut stream = tf_sub;
        while let Some(msg) = stream.next().await {
            let mut s = tf_state.write().await;
            for tf in &msg.transforms {
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
                };
                if let Some(existing) = s.transforms.iter_mut().find(|t| t.parent == entry.parent && t.child == entry.child) {
                    *existing = entry.clone();
                } else {
                    s.transforms.push(entry.clone());
                }
                if tf.child_frame_id == "base_link" || tf.child_frame_id == "base_footprint" {
                    s.pose.x = tf.transform.translation.x;
                    s.pose.y = tf.transform.translation.y;
                    s.pose.z = tf.transform.translation.z;
                    s.pose.qx = tf.transform.rotation.x;
                    s.pose.qy = tf.transform.rotation.y;
                    s.pose.qz = tf.transform.rotation.z;
                    s.pose.qw = tf.transform.rotation.w;
                    s.pose.frame_id = tf.header.frame_id.clone();
                }
            }
        }
    });

    // 主循环：spin node + 处理命令
    let cmd_state = state.clone();
    loop {
        node.spin_once(std::time::Duration::from_millis(5));

        match rx.try_recv() {
            Ok(cmd) => handle_command(cmd, &odom_pub, &pc_pub, &nav_client, &cmd_state, &pc_shm_writer).await,
            Err(mpsc::error::TryRecvError::Empty) => {}
            Err(mpsc::error::TryRecvError::Disconnected) => {
                info!("命令通道关闭，桥接线程退出");
                break;
            }
        }

        tokio::task::yield_now().await;
    }

    Ok(())
}

async fn handle_command(
    cmd: BridgeCommand,
    odom_pub: &r2r::Publisher<r2r::nav_msgs::msg::Odometry>,
    pc_pub: &r2r::Publisher<r2r::sensor_msgs::msg::PointCloud2>,
    nav_client: &r2r::ActionClient<r2r::nav2_msgs::action::NavigateToPose::Action>,
    state: &Arc<RwLock<SharedState>>,
    pc_shm_writer: &Rc<RefCell<Option<PointCloudShmWriter>>>,
) {
    match cmd {
        BridgeCommand::PublishOdom {
            frame_id, child_frame_id,
            x, y, z, qx, qy, qz, qw,
            lvx, lvy, lvz, avx, avy, avz, reply
        } => {
            let mut msg = r2r::nav_msgs::msg::Odometry::default();
            msg.header.frame_id = frame_id;
            msg.child_frame_id = child_frame_id;
            msg.pose.pose.position.x = x;
            msg.pose.pose.position.y = y;
            msg.pose.pose.position.z = z;
            msg.pose.pose.orientation.x = qx;
            msg.pose.pose.orientation.y = qy;
            msg.pose.pose.orientation.z = qz;
            msg.pose.pose.orientation.w = qw;
            msg.twist.twist.linear.x = lvx;
            msg.twist.twist.linear.y = lvy;
            msg.twist.twist.linear.z = lvz;
            msg.twist.twist.angular.x = avx;
            msg.twist.twist.angular.y = avy;
            msg.twist.twist.angular.z = avz;
            let r = odom_pub.publish(&msg).map_err(|e| format!("{}", e));
            let _ = reply.send(r);
        }
        BridgeCommand::PublishPointCloud {
            frame_id, data, point_count, point_step, reply
        } => {
            let processor = PointCloudProcessor::default_go2();
            let (processed_data, processed_count) = processor.process(&data, point_step);
            let out_step = 12u32; // XYZ only after processing

            // 写入共享内存供 Python 侧零拷贝读取
            if let Some(ref mut writer) = *pc_shm_writer.borrow_mut() {
                writer.write_frame(&processed_data, processed_count);
            }

            let mut msg = r2r::sensor_msgs::msg::PointCloud2::default();
            msg.header.frame_id = frame_id;
            msg.height = 1;
            msg.width = processed_count;
            msg.point_step = out_step;
            msg.row_step = out_step * processed_count;
            msg.is_dense = true;
            msg.data = processed_data;
            msg.fields = vec![
                r2r::sensor_msgs::msg::PointField { name: "x".into(), offset: 0, datatype: 7, count: 1 },
                r2r::sensor_msgs::msg::PointField { name: "y".into(), offset: 4, datatype: 7, count: 1 },
                r2r::sensor_msgs::msg::PointField { name: "z".into(), offset: 8, datatype: 7, count: 1 },
            ];
            let r = pc_pub.publish(&msg).map_err(|e| format!("{}", e));
            let _ = reply.send(r);
        }
        BridgeCommand::SendNavGoal {
            x, y, z, qx, qy, qz, qw, reply
        } => {
            let goal_id = uuid::Uuid::new_v4().to_string();
            let mut goal = r2r::nav2_msgs::action::NavigateToPose::Goal::default();
            goal.pose.header.frame_id = "map".to_string();
            goal.pose.pose.position.x = x;
            goal.pose.pose.position.y = y;
            goal.pose.pose.position.z = z;
            goal.pose.pose.orientation.x = qx;
            goal.pose.pose.orientation.y = qy;
            goal.pose.pose.orientation.z = qz;
            goal.pose.pose.orientation.w = qw;

            let state_for_nav = state.clone();
            let gid = goal_id.clone();
            match nav_client.send_goal_request(goal) {
                Ok(fut) => {
                    {
                        let mut s = state_for_nav.write().await;
                        s.nav_state.status = "NAVIGATING".into();
                        s.nav_state.message = format!("导航到 ({:.3}, {:.3})", x, y);
                        s.nav_state.goal_id = gid.clone();
                    }
                    info!("发送导航目标: ({:.3}, {:.3}, {:.3}) goal_id={}", x, y, z, gid);
                    let _ = reply.send(Ok(gid.clone()));
                    tokio::task::spawn_local(async move {
                        match fut.await {
                            Ok((_goal_handle, result_fut, _feedback_stream)) => {
                                match result_fut.await {
                                    Ok((_status, _result)) => {
                                        let mut s = state_for_nav.write().await;
                                        s.nav_state.status = "SUCCEEDED".into();
                                        s.nav_state.message = "导航完成".into();
                                    }
                                    Err(e) => {
                                        let mut s = state_for_nav.write().await;
                                        s.nav_state.status = "FAILED".into();
                                        s.nav_state.message = format!("导航失败: {}", e);
                                    }
                                }
                            }
                            Err(e) => {
                                let mut s = state_for_nav.write().await;
                                s.nav_state.status = "REJECTED".into();
                                s.nav_state.message = format!("目标被拒绝: {}", e);
                            }
                        }
                    });
                }
                Err(e) => {
                    let _ = reply.send(Err(format!("发送目标失败: {}", e)));
                }
            }
        }
        BridgeCommand::CancelNav { reply } => {
            {
                let mut s = state.write().await;
                s.nav_state.status = "IDLE".into();
                s.nav_state.message = "导航已取消".into();
            }
            info!("导航已取消");
            let _ = reply.send(Ok(()));
        }
    }
}
