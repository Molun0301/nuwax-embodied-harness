//! gRPC facade — `GeometryRuntime` service.
//!
//! 所有状态来自 `GeometryRuntimeHandle`（持有 LCM runtime SharedState）；
//! 与 backend（fake / real-lcm）解耦。本文件不引用任何 ROS2 / r2r / DDS 依赖。

use std::sync::Arc;
use tokio::sync::RwLock;
use tonic::{Request, Response, Status};
use tracing::info;

use crate::lcm_runtime::GeometryRuntimeHandle;
use crate::proto::*;
use crate::proto::geometry_runtime_server::GeometryRuntime;

pub struct GeometryRuntimeService {
    state: Arc<RwLock<ServiceState>>,
    bridge: GeometryRuntimeHandle,
}

struct ServiceState {
    phase: String,
    message: String,
    started_at_ns: u64,
}

impl GeometryRuntimeService {
    pub fn new(bridge: GeometryRuntimeHandle) -> Self {
        Self {
            state: Arc::new(RwLock::new(ServiceState {
                phase: "running".into(),
                message: "Geometry LCM runtime 已启动".into(),
                started_at_ns: now_ns(),
            })),
            bridge,
        }
    }
}

fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

fn uptime_ms(started_at_ns: u64) -> u64 {
    now_ns().saturating_sub(started_at_ns) / 1_000_000
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

#[tonic::async_trait]
impl GeometryRuntime for GeometryRuntimeService {
    // ========================================================================
    // 控制面
    // ========================================================================

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
            uptime_ms: uptime_ms(s.started_at_ns),
            rss_kb: get_rss_kb(),
            lcm_backend: self.bridge.transport_backend_name.to_string(),
        }))
    }

    async fn health_check(&self, _req: Request<HealthCheckRequest>) -> Result<Response<HealthCheckResponse>, Status> {
        let s = self.state.read().await;
        Ok(Response::new(HealthCheckResponse {
            healthy: true,
            phase: s.phase.clone(),
            uptime_ms: uptime_ms(s.started_at_ns),
            rss_kb: get_rss_kb(),
            video_frame_sequence: 0,
            video_fps_estimate: 0,
            lcm_backend: self.bridge.transport_backend_name.to_string(),
        }))
    }

    // ========================================================================
    // 定位
    // ========================================================================

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
            // 变化驱动：仅当位姿实际更新（timestamp_ns 变化）时才推送帧；
            // 避免静止时以 100ms 周期推送大量重复帧。last_stamp 初值 u64::MAX
            // 确保首个有效帧（含 stamp==0 的早期态）至少推送一次后再去重。
            let mut last_stamp = u64::MAX;
            loop {
                tokio::time::sleep(std::time::Duration::from_millis(100)).await;
                let update = {
                    let s = state.read().await;
                    if s.pose.timestamp_ns == last_stamp {
                        continue;
                    }
                    last_stamp = s.pose.timestamp_ns;
                    seq += 1;
                    PoseUpdate {
                        pose: Some(Pose {
                            position: Some(Vec3 { x: s.pose.x, y: s.pose.y, z: s.pose.z }),
                            orientation: Some(Quat { x: s.pose.qx, y: s.pose.qy, z: s.pose.qz, w: s.pose.qw }),
                            frame_id: s.pose.frame_id.clone(),
                            timestamp_ns: s.pose.timestamp_ns,
                        }),
                        sequence: seq,
                    }
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
            timestamp_ns: t.stamp_ns,
        }).collect();
        Ok(Response::new(FrameTree { transforms }))
    }

    // ========================================================================
    // 地图
    // ========================================================================

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
        let (meta, raw_data) = {
            let s = self.bridge.state.read().await;
            (
                OccupancyGridMeta {
                    width: s.map.width,
                    height: s.map.height,
                    resolution: s.map.resolution,
                    origin_x: s.map.origin_x,
                    origin_y: s.map.origin_y,
                    frame_id: s.map.frame_id.clone(),
                    revision: s.map.revision,
                    timestamp_ns: s.map.timestamp_ns,
                },
                s.map.data.clone(),
            )
        };
        Ok(Response::new(OccupancyGridData {
            meta: Some(meta),
            data: raw_data.iter().map(|&v| v as u8).collect(),
        }))
    }

    async fn get_occupancy_grid_shm(&self, _req: Request<Empty>) -> Result<Response<ShmRef>, Status> {
        Err(Status::unimplemented("OccupancyGrid SHM sidecar 待 P1 接入"))
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

    type SubscribeCloudMapUpdatesStream = tokio_stream::wrappers::ReceiverStream<Result<CloudMapShmRef, Status>>;
    async fn subscribe_cloud_map_updates(&self, _req: Request<Empty>) -> Result<Response<Self::SubscribeCloudMapUpdatesStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(4);
        let mut broadcast_rx = self.bridge.cloud_map_tx.subscribe();
        tokio::spawn(async move {
            while let Ok(notif) = broadcast_rx.recv().await {
                let msg = CloudMapShmRef {
                    shm_ref: Some(ShmRef {
                        key: std::env::var("NUWAX_CLOUDMAP_SHM_PATH")
                            .unwrap_or_else(|_| "/tmp/nuwax_cloudmap.mmap".into()),
                        size: 0,
                        revision: notif.revision,
                    }),
                    point_count: notif.point_count,
                    voxel_size: notif.voxel_size,
                };
                if tx.send(Ok(msg)).await.is_err() { break; }
            }
        });
        Ok(Response::new(tokio_stream::wrappers::ReceiverStream::new(rx)))
    }

    // ========================================================================
    // 导航兼容
    // ========================================================================

    async fn set_navigation_goal(&self, req: Request<NavigationGoal>) -> Result<Response<NavigationState>, Status> {
        // nuwax-nav-stack 接 LCM /nav/goal channel 后的实装放到 Phase 3 之后的 lane；
        // 当前返回 unconfigured。
        Ok(Response::new(NavigationState {
            status: "UNCONFIGURED".into(),
            message: "SetNavigationGoal 等待 nuwax-nav-stack LCM 接线完成".into(),
            goal_id: req.get_ref().goal_id.clone(),
            distance_remaining: 0.0,
            timestamp_ns: now_ns(),
        }))
    }

    async fn cancel_navigation(&self, req: Request<CancelNavigationRequest>) -> Result<Response<NavigationState>, Status> {
        Ok(Response::new(NavigationState {
            status: "CANCELLED".into(),
            message: "Cancel 已记录;真正下发走 nuwax-nav-stack LCM".into(),
            goal_id: req.get_ref().goal_id.clone(),
            distance_remaining: 0.0,
            timestamp_ns: now_ns(),
        }))
    }

    type SubscribeNavigationStateStream = tokio_stream::wrappers::ReceiverStream<Result<NavigationState, Status>>;
    async fn subscribe_navigation_state(&self, _req: Request<Empty>) -> Result<Response<Self::SubscribeNavigationStateStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(8);
        let mut broadcast_rx = self.bridge.nav_state_tx.subscribe();
        tokio::spawn(async move {
            while let Ok(nav) = broadcast_rx.recv().await {
                let state = NavigationState {
                    status: nav.status.clone(),
                    message: nav.message.clone(),
                    goal_id: nav.goal_id.clone(),
                    distance_remaining: nav.distance_remaining,
                    timestamp_ns: nav.timestamp_ns,
                };
                if tx.send(Ok(state)).await.is_err() { break; }
            }
        });
        Ok(Response::new(tokio_stream::wrappers::ReceiverStream::new(rx)))
    }

    async fn get_navigation_readiness(&self, _req: Request<Empty>) -> Result<Response<NavigationReadiness>, Status> {
        let s = self.bridge.state.read().await;
        let ready = !s.nav_state.status.is_empty() && s.nav_state.status != "UNCONFIGURED";
        Ok(Response::new(NavigationReadiness {
            ready,
            reason: if ready { "nuwax-nav-stack 已就绪".into() } else { "nuwax-nav-stack 尚未接线".into() },
        }))
    }

    // ========================================================================
    // 探索
    // ========================================================================

    async fn get_frontiers(&self, _req: Request<Empty>) -> Result<Response<FrontierList>, Status> {
        let s = self.bridge.state.read().await;
        let frontiers = s.frontiers.iter().map(|f| Frontier {
            pose: Some(Pose {
                position: Some(Vec3 { x: f.x, y: f.y, z: f.z }),
                orientation: Some(Quat { x: f.qx, y: f.qy, z: f.qz, w: f.qw }),
                frame_id: f.frame_id.clone(),
                timestamp_ns: 0,
            }),
            size_cells: f.size_cells,
            heading: f.heading,
        }).collect();
        Ok(Response::new(FrontierList {
            frontiers,
            revision: s.frontiers_revision,
        }))
    }

    type SubscribeFrontiersStream = tokio_stream::wrappers::ReceiverStream<Result<FrontierList, Status>>;
    async fn subscribe_frontiers(&self, _req: Request<Empty>) -> Result<Response<Self::SubscribeFrontiersStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(4);
        let mut broadcast_rx = self.bridge.frontiers_tx.subscribe();
        let state_handle = self.bridge.state.clone();
        tokio::spawn(async move {
            while let Ok(entries) = broadcast_rx.recv().await {
                let revision = {
                    let s = state_handle.read().await;
                    s.frontiers_revision
                };
                let frontiers = entries.iter().map(|f| Frontier {
                    pose: Some(Pose {
                        position: Some(Vec3 { x: f.x, y: f.y, z: f.z }),
                        orientation: Some(Quat { x: f.qx, y: f.qy, z: f.qz, w: f.qw }),
                        frame_id: f.frame_id.clone(),
                        timestamp_ns: 0,
                    }),
                    size_cells: f.size_cells,
                    heading: f.heading,
                }).collect();
                if tx.send(Ok(FrontierList { frontiers, revision })).await.is_err() { break; }
            }
        });
        Ok(Response::new(tokio_stream::wrappers::ReceiverStream::new(rx)))
    }

    // ========================================================================
    // 输入发布兼容（gRPC 端用于调试；走 LCM 后端发布）
    // ========================================================================

    async fn publish_odom(&self, _req: Request<PublishOdomRequest>) -> Result<Response<PublishResult>, Status> {
        Ok(Response::new(PublishResult {
            success: true,
            message: "odom LCM publish 已接受;P1 走 real backend".into(),
        }))
    }

    async fn publish_point_cloud(&self, _req: Request<PublishPointCloudRequest>) -> Result<Response<PublishResult>, Status> {
        Ok(Response::new(PublishResult {
            success: true,
            message: "PointCloud2 LCM publish 已接受;P1 走 real backend".into(),
        }))
    }
}