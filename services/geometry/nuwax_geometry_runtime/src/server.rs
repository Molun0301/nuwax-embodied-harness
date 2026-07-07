// Phase 6 (codex-round3 R3-4)：server.rs 多处 format! 调用被 clippy 1.88
// `uninlined_format_args` 报告(e.g. line 239/590/605/624/639 的 `format!("...{}", e)`),
// 风格性 lint。统一 file-level 标注不强行收敛——debug log 与 Status::internal
// 字符串拼接的可读性高于命名参数形式。
#![allow(clippy::uninlined_format_args)]

//! gRPC facade — `GeometryRuntime` service.
//!
//! 所有状态来自 `GeometryRuntimeHandle`（持有 LCM runtime SharedState）；
//! 与 backend（fake / real-lcm）解耦。本文件不引用任何 ROS2 / r2r / DDS 依赖。
//!
//! Phase 2 重构:
//! - 状态聚合统一走 `runtime_diagnostics`（health / readiness / blockers）
//! - typed payload encode 统一走 `typed_codec`（禁止再写 ad-hoc bytes）
//! - env 解析入口统一走 `channel_registry`

use std::sync::Arc;
use tokio::sync::RwLock;
use tonic::{Request, Response, Status};
use tracing::info;

use crate::lcm_runtime::{GeometryRuntimeHandle, PoseState};
use crate::nav_runtime::{
    cancel as nav_cancel, compute_readiness, ensure_goal_id, readiness_to_reason, set_goal,
    CancelOutcome, SetGoalInput, SetGoalOutcome,
};
use crate::proto::*;
use crate::proto::geometry_runtime_server::GeometryRuntime;
use crate::runtime_diagnostics::{self, RuntimeDiagnostics, ServiceState, SharedServiceState};
use crate::typed_codec;

pub struct GeometryRuntimeService {
    state: SharedServiceState,
    bridge: GeometryRuntimeHandle,
}

impl GeometryRuntimeService {
    pub fn new(bridge: GeometryRuntimeHandle) -> Self {
        Self {
            state: Arc::new(RwLock::new(ServiceState::new_running(
                "Geometry LCM runtime 已启动",
            ))),
            bridge,
        }
    }

    /// 构造本进程的 RuntimeDiagnostics(读 ServiceState 算 uptime)
    async fn diagnostics(&self) -> RuntimeDiagnostics {
        let s = self.state.read().await;
        RuntimeDiagnostics::from_handle(&self.bridge, &s.phase, s.started_at_ns)
    }
}

fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

/// 把内部 `NavState`(crate::lcm_runtime::NavState)转成 gRPC
/// `proto::NavigationState`,避免 server.rs 直接依赖 internal type。
fn nav_state_to_proto(nav: crate::lcm_runtime::NavState) -> NavigationState {
    NavigationState {
        status: nav.status,
        message: nav.message,
        goal_id: nav.goal_id,
        distance_remaining: nav.distance_remaining,
        timestamp_ns: nav.timestamp_ns,
    }
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
        let diag = RuntimeDiagnostics::from_handle(&self.bridge, &s.phase, s.started_at_ns);
        Ok(Response::new(RuntimeStatus {
            phase: diag.phase,
            message: s.message.clone(),
            uptime_ms: diag.uptime_ms,
            rss_kb: diag.rss_kb,
            lcm_backend: diag.lcm_backend,
        }))
    }

    async fn health_check(&self, _req: Request<HealthCheckRequest>) -> Result<Response<HealthCheckResponse>, Status> {
        let diag = self.diagnostics().await;
        // Phase 1 fail-fast + Phase 2 diagnostics 收敛:health_check 统一
        // 从 RuntimeDiagnostics 读 healthy + lcm_backend。
        // HealthCheck 调用方可从 `lcm_backend` 字段 grep
        // "LCM_BACKEND_UNAVAILABLE" 关键字做 CI gate。
        Ok(Response::new(HealthCheckResponse {
            healthy: diag.healthy,
            phase: diag.phase,
            uptime_ms: diag.uptime_ms,
            rss_kb: diag.rss_kb,
            video_frame_sequence: 0,
            video_fps_estimate: 0,
            lcm_backend: diag.lcm_backend,
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
        // Phase 3: 真实 SHM sidecar 写入(occupancy_shm::OccupancyGridShmWriter);
        // 成功返回 ShmRef { key=shm_path, size=map.data.len(), revision=map.revision }
        // 失败 / 无数据 / 写盘错误 → 返回 error message,不假装成功。
        if self.bridge.occupancy_shm.is_none() {
            return Err(Status::unavailable(
                "occupancy_shm 未初始化 (LCM backend 不可用,导致 SHM 写入被禁用)",
            ));
        }
        let shm_path = std::env::var("NUWAX_OCCUPANCY_SHM_PATH")
            .unwrap_or_else(|_| "/tmp/nuwax_occupancy.mmap".into());
        // 锁顺序:先 copy 关键字段(revision/size)再 drop read lock,最后持 writer lock;
        // 避免 reader 与 writer 同时持锁导致自死锁。
        let map_snapshot = {
            let s = self.bridge.state.read().await;
            if s.map.width == 0 || s.map.height == 0 {
                return Err(Status::failed_precondition(
                    "occupancy grid 尚未生成 (mapping adapter 无点云数据)",
                ));
            }
            s.map.clone()
        };
        let size = map_snapshot.data.len();
        let revision = map_snapshot.revision;
        let mut writer = self.bridge.occupancy_shm.as_ref().expect("checked above").lock();
        if let Err(e) = writer.write_occupancy(&map_snapshot) {
            return Err(Status::internal(format!("write_occupancy 失败: {}", e)));
        }
        Ok(Response::new(ShmRef {
            key: shm_path,
            size: size as u64,
            revision,
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
    //
    // Phase 4:`/nav/goal` 与 `/nav/cancel` publisher 已由 nav_runtime.rs 接管
    // (`publish_goal_frame` / `publish_cancel_frame`),通过 LcmTransport.publish
    // 推到 LCM typed channel。本节保留 setter RPC 入口供 Python/前端调用。
    // 删除 Phase 3 的 "UNIMPLEMENTED" placeholder 注释 —— readiness 与状态机
    // 都在 nav_runtime.rs 内真实执行(`set_goal` / `cancel`)。
    //
    // Phase 7:`SetNavigationGoal` 在 real production profile 且 cmd_vel_sink=false
    // 时被 nav_runtime::set_goal 拒绝;readiness 全量字段由
    // `mapping_adapter::MappingReadiness` / `nav_runtime::NavigationReadinessDetail`
    // / `explore_runtime::ExplorationReadinessDetail` 暴露,见
    // `docs/.../11_readiness_收敛.md`。

    async fn set_navigation_goal(&self, req: Request<NavigationGoal>) -> Result<Response<NavigationState>, Status> {
        // Phase 4:接入 nav_runtime。流程:
        // 1) 归一化 NavigationGoal -> SetGoalInput(自动生成 UUID 当 goal_id 空)
        // 2) 调 set_goal():readiness 校验 -> publish /nav/goal typed frame
        //    -> 状态机推进 -> publish /cmd_vel 起步速度
        // 3) 把 NavState 同步到 SharedState.nav_state + broadcast 通道
        let ng = req.get_ref();
        let pose = ng.pose.as_ref();
        let pos = pose.and_then(|p| p.position.as_ref());
        let orient = pose.and_then(|p| p.orientation.as_ref());
        let target = PoseState {
            x: pos.map(|v| v.x).unwrap_or(0.0),
            y: pos.map(|v| v.y).unwrap_or(0.0),
            z: pos.map(|v| v.z).unwrap_or(0.0),
            qx: orient.map(|v| v.x).unwrap_or(0.0),
            qy: orient.map(|v| v.y).unwrap_or(0.0),
            qz: orient.map(|v| v.z).unwrap_or(0.0),
            qw: orient.map(|v| v.w).unwrap_or(1.0),
            frame_id: pose.map(|p| p.frame_id.clone()).unwrap_or_default(),
            timestamp_ns: pose.map(|p| p.timestamp_ns).unwrap_or(now_ns()),
        };
        let goal_id = ensure_goal_id(&ng.goal_id);
        let frame_id = if ng.frame_id.is_empty() { "map".to_string() } else { ng.frame_id.clone() };
        let input = SetGoalInput { goal_id: goal_id.clone(), frame_id, target };
        let SetGoalOutcome { state: nav } = set_goal(&self.bridge, &self.bridge.nav, input).await;
        // 同步到 SharedState.nav_state,供 SubscribeNavigationState 推送。
        {
            let mut s = self.bridge.state.write().await;
            s.nav_state = nav.clone();
        }
        let _ = self.bridge.nav_state_tx.send(nav.clone());
        Ok(Response::new(nav_state_to_proto(nav)))
    }

    async fn cancel_navigation(&self, req: Request<CancelNavigationRequest>) -> Result<Response<NavigationState>, Status> {
        // Phase 4:接入 nav_runtime.cancel。流程:
        // 1) publish /nav/cancel typed frame
        // 2) publish zero cmd_vel
        // 3) 状态机切到 Cancelled
        let goal_id = ensure_goal_id(&req.get_ref().goal_id);
        let CancelOutcome { state: nav } =
            nav_cancel(&self.bridge, &self.bridge.nav, goal_id.clone()).await;
        {
            let mut s = self.bridge.state.write().await;
            s.nav_state = nav.clone();
        }
        let _ = self.bridge.nav_state_tx.send(nav.clone());
        Ok(Response::new(nav_state_to_proto(nav)))
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
        // Phase 4:细分 readiness 子项(LCM backend + tf + costmap +
        // planner profile + cmd_vel sink)由 nav_runtime::compute_readiness
        // 计算,JSON 序列化到 `reason` 字段。Python 侧
        // `get_navigation_readiness` 解析 JSON 返回 dict(满足"costmap、
        // tf、planner、cmd_vel sink、LCM backend 子项"要求)。
        // 顶层 ready/reason 仍走 runtime_diagnostics 的粗略判定,保证
        // CI/CLI 现有 grep "nav ready" 的守门不会失效。
        let r = runtime_diagnostics::navigation_readiness(&self.bridge);
        let detail = compute_readiness(&self.bridge, &self.bridge.nav).await;
        let json = readiness_to_reason(&detail);
        Ok(Response::new(NavigationReadiness {
            ready: r.ready && detail.ready,
            reason: format!("{}; details={}", r.reason, json),
        }))
    }

    /// Phase 7:mapping 板块 readiness 综合快照。
    async fn get_mapping_readiness(&self, _req: Request<Empty>) -> Result<Response<MappingReadiness>, Status> {
        use crate::mapping_adapter::MappingReadiness as MappingReadinessDetail;
        // parking_lot Mutex guard 不能直接跨 await 持有,这里把锁解开再算。
        let snapshot = {
            let lcm_backend = self.bridge.lcm_backend_available;
            match &self.bridge.mapping_adapter {
                Some(arc) => {
                    let g = arc.lock();
                    MappingReadinessDetail::compute(Some(&*g), lcm_backend)
                }
                None => MappingReadinessDetail::compute(None, lcm_backend),
            }
        };
        let message_json = serde_json::to_string(&serde_json::json!({
            "lcm_backend": snapshot.lcm_backend,
            "adapter_present": snapshot.adapter_present,
            "odom_count": snapshot.odom_count,
            "pointcloud_count": snapshot.pointcloud_count,
            "last_occupancy_revision": snapshot.last_occupancy_revision,
            "accumulated_point_count": snapshot.accumulated_point_count,
            "last_error": snapshot.last_error,
        }))
        .unwrap_or_else(|e| format!("{{\"serialization_error\":\"{}\"}}", e));
        Ok(Response::new(MappingReadiness {
            ready: snapshot.ready,
            message: message_json,
        }))
    }

    /// Phase 7:exploration 板块 readiness 综合快照。
    async fn get_exploration_readiness(&self, _req: Request<Empty>) -> Result<Response<ExplorationReadiness>, Status> {
        let backend = self.bridge.lcm_backend_available;
        let snapshot = self.bridge.explore.readiness(backend).await;
        let message_json = serde_json::to_string(&serde_json::json!({
            "backend_available": snapshot.backend_available,
            "controller_active": snapshot.controller_active,
            "source_count": snapshot.source_count,
            "frontier_source_ready": snapshot.frontier_source_ready,
            "costmap_input_ready": snapshot.costmap_input_ready,
            "costmap_input_revision": snapshot.costmap_input_revision,
            "active_source": snapshot.active_source,
            "arbitration": snapshot.arbitration,
        }))
        .unwrap_or_else(|e| format!("{{\"serialization_error\":\"{}\"}}", e));
        Ok(Response::new(ExplorationReadiness {
            ready: snapshot.ready,
            message: message_json,
        }))
    }

    // ========================================================================
    // cmd_vel sink readiness 写回（rework17）
    // ========================================================================
    //
    // Python `LcmMotionCommandBridge` 在 start/stop 时通过本 RPC 把 sink ready
    // 状态推到 Rust handle。`compute_readiness` 的 cmd_vel_sink 子项读这个 flag;
    // 之前 spawn 默认 false + 跨进程无通道 → readiness 永远 false → 真实
    // production 路径走不通。
    async fn set_cmd_vel_sink_ready(
        &self,
        request: Request<CmdVelSinkReadyRequest>,
    ) -> Result<Response<CmdVelSinkReadyResponse>, Status> {
        let ready = request.into_inner().ready;
        self.bridge.set_cmd_vel_sink_ready(ready);
        Ok(Response::new(CmdVelSinkReadyResponse { success: true }))
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
    // 探索 start / stop / state / diagnostics（Phase 5 additive）
    // ========================================================================
    //
    // 这些 RPC 把 `nuwax-exploration::ExplorationController` 暴露给 Python
    // 平台层；上游 `nuwax-exploration` crate 的 source adapter（wavefront）
    // 在 `explore_runtime.rs` 内部组装；本 facade 只做字段转换 + 锁定 dimos
    // 行为,不再自造隐式 fallback。
    //
    // 状态契约（与 proto 注释一致）：
    // - `ACCEPTED` / `REJECTED` / `PENDING` / `STOPPED` / `ACTIVE` / `UNIMPLEMENTED`
    // - `last_stop_reason` 字符串: `no_information_gain` / `no_reachable_free_space` /
    //   `consecutive_failures_exceeded` / `user_cancelled` / 空串

    async fn start_exploration(&self, req: Request<ExploreRequest>) -> Result<Response<ExplorationState>, Status> {
        let r = req.get_ref();
        let snapshot = self
            .bridge
            .explore
            .start(&r.request_id, &r.arbitration)
            .await;
        Ok(Response::new(ExplorationState {
            status: snapshot.status.to_string(),
            message: snapshot.message,
            request_id: snapshot.request_id,
            selected_source: snapshot.selected_source,
            last_stop_reason: snapshot.last_stop_reason,
            frontier_count: snapshot.frontier_count,
            revision: snapshot.revision,
            timestamp_ns: now_ns(),
        }))
    }

    async fn stop_exploration(&self, req: Request<StopExplorationRequest>) -> Result<Response<ExplorationState>, Status> {
        let r = req.get_ref();
        let snapshot = self
            .bridge
            .explore
            .stop(&r.request_id)
            .await;
        Ok(Response::new(ExplorationState {
            status: snapshot.status.to_string(),
            message: snapshot.message,
            request_id: snapshot.request_id,
            selected_source: snapshot.selected_source,
            last_stop_reason: snapshot.last_stop_reason,
            frontier_count: snapshot.frontier_count,
            revision: snapshot.revision,
            timestamp_ns: now_ns(),
        }))
    }

    async fn get_exploration_state(&self, _req: Request<Empty>) -> Result<Response<ExplorationState>, Status> {
        // 单次 tick 触发 controller poll;即便上游没有 costmap 输入,
        // 也保证状态机能跑通而不假装成功。
        let pose_xy = {
            let s = self.bridge.state.read().await;
            (s.pose.x, s.pose.y)
        };
        let snapshot = self.bridge.explore.tick(pose_xy).await;
        Ok(Response::new(ExplorationState {
            status: snapshot.status.to_string(),
            message: snapshot.message,
            request_id: snapshot.request_id,
            selected_source: snapshot.selected_source,
            last_stop_reason: snapshot.last_stop_reason,
            frontier_count: snapshot.frontier_count,
            revision: snapshot.revision,
            timestamp_ns: now_ns(),
        }))
    }

    async fn get_exploration_diagnostics(&self, _req: Request<Empty>) -> Result<Response<ExplorationDiagnostics>, Status> {
        let diag = self.bridge.explore.diagnostics().await;
        let sources = diag
            .sources
            .into_iter()
            .map(|s| ExplorationSourceDiag {
                source_name: s.source_name,
                is_active: s.is_active,
                last_goal_id: s.last_goal_id,
                last_error: s.last_error,
            })
            .collect();
        Ok(Response::new(ExplorationDiagnostics {
            active: diag.active,
            arbitration: diag.arbitration,
            selected_source: diag.selected_source,
            last_stop_reason: diag.last_stop_reason,
            sources,
            frontier_count: diag.frontier_count,
            revision: diag.revision,
            timestamp_ns: now_ns(),
        }))
    }

    // ========================================================================
    // 输入发布兼容（gRPC 端用于调试；走 LCM 后端发布）
    // ========================================================================
    //
    // Phase 4 重写：`publish_odom` / `publish_point_cloud` 不再硬编码 channel；
    // 改走 `channel_registry::channel_for(Odom / PointCloud)`，由 env 或
    // default 拼 typed suffix。删除 stale "B6 blocker" / "Phase 5 才完成"
    // 注释 —— typed payload 编码（typed_codec）与 LcmTransport.publish 调用
    // 已是当前正确路径，real-lcm backend 待 Phase 3 完成后再启用。

    async fn publish_odom(&self, req: Request<PublishOdomRequest>) -> Result<Response<PublishResult>, Status> {
        // Phase 4:typed payload 编码走 typed_codec;channel 走 channel_registry;
        // 禁止本文件再写硬编码 `/odom` 或 ad-hoc bytes。
        use nuwax_protocol::LcmMessageType;
        let pose = req.get_ref().pose.as_ref();
        let (frame_id, ts_ns) = match pose {
            Some(p) => (p.frame_id.clone(), p.timestamp_ns),
            None => (String::new(), now_ns()),
        };
        let (x, y, z, qx, qy, qz, qw) = match pose {
            Some(p) => {
                let pos = p.position.as_ref();
                let q = p.orientation.as_ref();
                (
                    pos.map(|v| v.x).unwrap_or(0.0),
                    pos.map(|v| v.y).unwrap_or(0.0),
                    pos.map(|v| v.z).unwrap_or(0.0),
                    q.map(|v| v.x).unwrap_or(0.0),
                    q.map(|v| v.y).unwrap_or(0.0),
                    q.map(|v| v.z).unwrap_or(0.0),
                    q.map(|v| v.w).unwrap_or(0.0),
                )
            }
            None => (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        };
        let pose_state = crate::lcm_runtime::PoseState {
            x, y, z, qx, qy, qz, qw,
            frame_id,
            timestamp_ns: ts_ns,
        };
        let payload = typed_codec::encode_odometry_payload(&pose_state);

        // Phase 4:走 channel_registry::channel_for(Odom) 公共 helper。
        let topic = crate::channel_registry::channel_for(
            crate::channel_registry::ChannelKey::Odom,
        );
        let msg_type = match LcmMessageType::new("nav_msgs/Odometry") {
            Ok(m) => m,
            Err(e) => {
                tracing::error!("publish_odom: LcmMessageType 构造失败: {}", e);
                return Ok(Response::new(PublishResult {
                    success: false,
                    message: format!("LcmMessageType::new failed: {}", e),
                }));
            }
        };
        let frame = typed_codec::build_typed_frame(topic, msg_type, payload, 0);
        let lcm_transport = self.bridge.transport.clone();
        match lcm_transport.publish(frame).await {
            Ok(()) => {
                // Phase 7:发布成功后在 mapping_adapter 上累加 odom_count,
                // readiness 暴露用。不动 feed_* 函数签名。
                if let Some(adapter) = self.bridge.mapping_adapter.as_ref() {
                    adapter.lock().increment_odom_count();
                }
                Ok(Response::new(PublishResult {
                    success: true,
                    message: format!("odom LCM publish 已接受（backend={}）", self.bridge.transport_backend_name),
                }))
            }
            Err(e) => {
                tracing::warn!("publish_odom: LCM publish 失败: {}", e);
                Ok(Response::new(PublishResult {
                    success: false,
                    message: format!("LCM publish failed: {}", e),
                }))
            }
        }
    }

    async fn publish_point_cloud(&self, req: Request<PublishPointCloudRequest>) -> Result<Response<PublishResult>, Status> {
        // Phase 4:channel 走 channel_registry::channel_for(PointCloud)，
        // 不再硬编码 `/go2/lidar/points`。
        use nuwax_protocol::LcmMessageType;
        let pcr = req.get_ref();
        let payload = typed_codec::encode_pointcloud_payload(&pcr.points, pcr.point_step);

        let topic = crate::channel_registry::channel_for(
            crate::channel_registry::ChannelKey::PointCloud,
        );
        let msg_type = match LcmMessageType::new("sensor_msgs/PointCloud2") {
            Ok(m) => m,
            Err(e) => {
                tracing::error!("publish_point_cloud: LcmMessageType 构造失败: {}", e);
                return Ok(Response::new(PublishResult {
                    success: false,
                    message: format!("LcmMessageType::new failed: {}", e),
                }));
            }
        };
        let frame = typed_codec::build_typed_frame(topic, msg_type, payload, 0);
        let lcm_transport = self.bridge.transport.clone();
        match lcm_transport.publish(frame).await {
            Ok(()) => {
                // Phase 7:同 publish_odom,发布成功后累加 pointcloud_count + occupancy revision。
                if let Some(adapter) = self.bridge.mapping_adapter.as_ref() {
                    adapter.lock().increment_pointcloud_count();
                    adapter.lock().increment_occupancy_revision();
                }
                Ok(Response::new(PublishResult {
                    success: true,
                    message: format!("PointCloud2 LCM publish 已接受（backend={}）", self.bridge.transport_backend_name),
                }))
            }
            Err(e) => {
                tracing::warn!("publish_point_cloud: LCM publish 失败: {}", e);
                Ok(Response::new(PublishResult {
                    success: false,
                    message: format!("LCM publish failed: {}", e),
                }))
            }
        }
    }
}