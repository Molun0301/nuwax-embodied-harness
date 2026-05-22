use tokio::sync::RwLock;
use std::sync::Arc;
use tonic::{Request, Response, Status};
use tracing::info;

use crate::proto::*;
use crate::proto::geometry_runtime_server::GeometryRuntime;
use crate::ros2_bridge::Ros2BridgeHandle;

pub struct GeometryRuntimeService {
    state: Arc<RwLock<ServiceState>>,
    bridge: Ros2BridgeHandle,
}

struct ServiceState {
    phase: String,
    message: String,
}

impl GeometryRuntimeService {
    pub fn new(bridge: Ros2BridgeHandle) -> Self {
        Self {
            state: Arc::new(RwLock::new(ServiceState {
                phase: "running".into(),
                message: "ROS2 桥接已启动".into(),
            })),
            bridge,
        }
    }
}

#[tonic::async_trait]
impl GeometryRuntime for GeometryRuntimeService {
    async fn start(&self, _req: Request<StartRequest>) -> Result<Response<StartResult>, Status> {
        Ok(Response::new(StartResult { success: true, message: "已在运行".into() }))
    }

    async fn stop(&self, _req: Request<StopRequest>) -> Result<Response<StopResult>, Status> {
        info!("收到 Stop 指令");
        Ok(Response::new(StopResult { success: true }))
    }

    async fn get_status(&self, _req: Request<Empty>) -> Result<Response<RuntimeStatus>, Status> {
        let s = self.state.read().await;
        Ok(Response::new(RuntimeStatus {
            phase: s.phase.clone(),
            message: s.message.clone(),
            uptime_ms: 0,
            rss_kb: get_rss_kb(),
        }))
    }

    async fn health_check(&self, _req: Request<HealthCheckRequest>) -> Result<Response<HealthCheckResponse>, Status> {
        Ok(Response::new(HealthCheckResponse {
            healthy: true,
            phase: "running".into(),
            uptime_ms: 0,
            rss_kb: get_rss_kb(),
            video_frame_sequence: 0,
            video_fps_estimate: 0,
        }))
    }

    async fn get_current_pose(&self, _req: Request<PoseRequest>) -> Result<Response<Pose>, Status> {
        let s = self.bridge.state.read().await;
        Ok(Response::new(Pose {
            position: Some(Vec3 { x: s.pose.x, y: s.pose.y, z: s.pose.z }),
            orientation: Some(Quat { x: s.pose.qx, y: s.pose.qy, z: s.pose.qz, w: s.pose.qw }),
            frame_id: s.pose.frame_id.clone(),
            timestamp_ns: s.pose.timestamp_ns,
        }))
    }

    type SubscribePoseStream = tokio_stream::wrappers::ReceiverStream<Result<PoseUpdate, Status>>;
    async fn subscribe_pose(&self, _req: Request<PoseRequest>) -> Result<Response<Self::SubscribePoseStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(32);
        let state = self.bridge.state.clone();
        tokio::spawn(async move {
            let mut seq = 0u64;
            loop {
                tokio::time::sleep(std::time::Duration::from_millis(100)).await;
                let s = state.read().await;
                seq += 1;
                let update = PoseUpdate {
                    pose: Some(Pose {
                        position: Some(Vec3 { x: s.pose.x, y: s.pose.y, z: s.pose.z }),
                        orientation: Some(Quat { x: s.pose.qx, y: s.pose.qy, z: s.pose.qz, w: s.pose.qw }),
                        frame_id: s.pose.frame_id.clone(),
                        timestamp_ns: s.pose.timestamp_ns,
                    }),
                    sequence: seq,
                };
                if tx.send(Ok(update)).await.is_err() { break; }
            }
        });
        Ok(Response::new(tokio_stream::wrappers::ReceiverStream::new(rx)))
    }

    async fn get_frame_tree(&self, _req: Request<Empty>) -> Result<Response<FrameTree>, Status> {
        let s = self.bridge.state.read().await;
        let transforms = s.transforms.iter().map(|t| Transform {
            parent_frame: t.parent.clone(),
            child_frame: t.child.clone(),
            translation: Some(Vec3 { x: t.tx, y: t.ty, z: t.tz }),
            rotation: Some(Quat { x: t.rx, y: t.ry, z: t.rz, w: t.rw }),
            timestamp_ns: 0,
        }).collect();
        Ok(Response::new(FrameTree { transforms }))
    }

    async fn get_occupancy_grid(&self, _req: Request<Empty>) -> Result<Response<OccupancyGridMeta>, Status> {
        let s = self.bridge.state.read().await;
        Ok(Response::new(OccupancyGridMeta {
            width: s.map.width,
            height: s.map.height,
            resolution: s.map.resolution,
            origin_x: s.map.origin_x,
            origin_y: s.map.origin_y,
            frame_id: s.map.frame_id.clone(),
            revision: s.map.revision,
            timestamp_ns: s.map.timestamp_ns,
        }))
    }

    async fn get_occupancy_grid_data(&self, _req: Request<Empty>) -> Result<Response<OccupancyGridData>, Status> {
        let s = self.bridge.state.read().await;
        Ok(Response::new(OccupancyGridData {
            meta: Some(OccupancyGridMeta {
                width: s.map.width,
                height: s.map.height,
                resolution: s.map.resolution,
                origin_x: s.map.origin_x,
                origin_y: s.map.origin_y,
                frame_id: s.map.frame_id.clone(),
                revision: s.map.revision,
                timestamp_ns: s.map.timestamp_ns,
            }),
            data: s.map.data.iter().map(|&v| v as u8).collect(),
        }))
    }

    type SubscribeMapUpdatesStream = tokio_stream::wrappers::ReceiverStream<Result<MapUpdateNotification, Status>>;
    async fn subscribe_map_updates(&self, _req: Request<Empty>) -> Result<Response<Self::SubscribeMapUpdatesStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(8);
        let state = self.bridge.state.clone();
        tokio::spawn(async move {
            let mut last_rev = 0u64;
            loop {
                tokio::time::sleep(std::time::Duration::from_millis(500)).await;
                let s = state.read().await;
                if s.map.revision > last_rev {
                    last_rev = s.map.revision;
                    let notif = MapUpdateNotification {
                        meta: Some(OccupancyGridMeta {
                            width: s.map.width, height: s.map.height,
                            resolution: s.map.resolution,
                            origin_x: s.map.origin_x, origin_y: s.map.origin_y,
                            frame_id: s.map.frame_id.clone(),
                            revision: s.map.revision, timestamp_ns: s.map.timestamp_ns,
                        }),
                        shm_ref: None,
                    };
                    if tx.send(Ok(notif)).await.is_err() { break; }
                }
            }
        });
        Ok(Response::new(tokio_stream::wrappers::ReceiverStream::new(rx)))
    }

    async fn get_occupancy_grid_shm(&self, _req: Request<Empty>) -> Result<Response<ShmRef>, Status> {
        Err(Status::unimplemented("Phase 2d shm 实现"))
    }

    async fn set_navigation_goal(&self, req: Request<NavigationGoal>) -> Result<Response<NavigationAck>, Status> {
        let goal = req.into_inner();
        let pose = goal.target_pose.unwrap_or_default();
        let pos = pose.position.unwrap_or_default();
        let ori = pose.orientation.unwrap_or_default();
        match self.bridge.send_nav_goal(pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w).await {
            Ok(goal_id) => Ok(Response::new(NavigationAck {
                accepted: true, goal_id, message: "目标已接受".into(),
            })),
            Err(e) => Ok(Response::new(NavigationAck {
                accepted: false, goal_id: String::new(), message: e,
            })),
        }
    }

    async fn cancel_navigation(&self, _req: Request<Empty>) -> Result<Response<CancelResult>, Status> {
        match self.bridge.cancel_nav().await {
            Ok(()) => Ok(Response::new(CancelResult { success: true, message: "已取消".into() })),
            Err(e) => Ok(Response::new(CancelResult { success: false, message: e })),
        }
    }

    type SubscribeNavigationStateStream = tokio_stream::wrappers::ReceiverStream<Result<NavigationStateUpdate, Status>>;
    async fn subscribe_navigation_state(&self, _req: Request<Empty>) -> Result<Response<Self::SubscribeNavigationStateStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(16);
        let state = self.bridge.state.clone();
        tokio::spawn(async move {
            loop {
                tokio::time::sleep(std::time::Duration::from_millis(200)).await;
                let s = state.read().await;
                let update = NavigationStateUpdate {
                    status: s.nav_state.status.clone(),
                    message: s.nav_state.message.clone(),
                    current_pose: None,
                    distance_remaining: s.nav_state.distance_remaining,
                    goal_id: s.nav_state.goal_id.clone(),
                    timestamp_ns: 0,
                };
                if tx.send(Ok(update)).await.is_err() { break; }
            }
        });
        Ok(Response::new(tokio_stream::wrappers::ReceiverStream::new(rx)))
    }

    async fn get_navigation_readiness(&self, _req: Request<Empty>) -> Result<Response<Nav2Readiness>, Status> {
        let s = self.bridge.state.read().await;
        Ok(Response::new(Nav2Readiness {
            ready: s.nav2_action_server_ready,
            waiting_for: if s.nav2_action_server_ready { String::new() } else { "action_server".into() },
            message: if s.nav2_action_server_ready { "Nav2 就绪".into() } else { "等待 Nav2 action server".into() },
            action_server_ready: s.nav2_action_server_ready,
            tf_ready: !s.transforms.is_empty(),
            lifecycle_states: std::collections::HashMap::new(),
            errors: std::collections::HashMap::new(),
        }))
    }

    async fn get_frontiers(&self, _req: Request<Empty>) -> Result<Response<FrontierList>, Status> {
        Ok(Response::new(FrontierList { frontiers: vec![], timestamp_ns: 0 }))
    }

    type SubscribeFrontiersStream = tokio_stream::wrappers::ReceiverStream<Result<FrontierUpdate, Status>>;
    async fn subscribe_frontiers(&self, _req: Request<Empty>) -> Result<Response<Self::SubscribeFrontiersStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(8);
        tokio::spawn(async move { let _ = tx; });
        Ok(Response::new(tokio_stream::wrappers::ReceiverStream::new(rx)))
    }

    async fn publish_odom(&self, req: Request<OdomInput>) -> Result<Response<Empty>, Status> {
        let input = req.into_inner();
        let pose = input.pose.unwrap_or_default();
        let pos = pose.position.unwrap_or_default();
        let ori = pose.orientation.unwrap_or_default();
        let lin = input.linear_velocity.unwrap_or_default();
        let ang = input.angular_velocity.unwrap_or_default();
        self.bridge.publish_odom(
            &pose.frame_id, "base_link",
            pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w,
            lin.x, lin.y, lin.z, ang.x, ang.y, ang.z,
        ).await.map_err(|e| Status::internal(e))?;
        Ok(Response::new(Empty {}))
    }

    async fn publish_point_cloud(&self, req: Request<PointCloudShmRef>) -> Result<Response<Empty>, Status> {
        let input = req.into_inner();
        let shm = input.shm_ref.unwrap_or_default();
        self.bridge.publish_pointcloud(
            &input.frame_id, shm.key.into_bytes(), input.point_count, input.point_step,
        ).await.map_err(|e| Status::internal(e))?;
        Ok(Response::new(Empty {}))
    }
}

fn get_rss_kb() -> u32 {
    std::fs::read_to_string("/proc/self/status")
        .ok()
        .and_then(|s| {
            s.lines()
                .find(|l| l.starts_with("VmRSS"))
                .and_then(|l| l.split_whitespace().nth(1))
                .and_then(|v| v.parse::<u32>().ok())
        })
        .unwrap_or(0)
}
