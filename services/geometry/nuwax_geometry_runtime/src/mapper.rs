//! 当前会话的地图拥有方；实时帧直达本对象，控制操作用自身队列形成检查点。
use crate::{
    corrector::{CorrectionInput, Corrector, PgoUpdate},
    mapping_backend::{
        self, MappingBackendComputeConfig, MappingBackendInput, MappingBackendKind,
        MappingBackendSpec, MappingKernel,
    },
    persistent_occupancy::PersistentOccupancyGrid,
    pointcloud::CloudMap,
    proto,
};
use nalgebra::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion};
use std::{
    path::PathBuf,
    sync::{mpsc, Arc},
    time::{Duration, Instant, SystemTime},
};
use tokio::sync::{oneshot, watch};
use tonic::Status;

#[path = "correction_worker.rs"]
mod correction_worker;
use correction_worker::{CorrectionWorker, Rebuilt};

#[derive(Clone, Debug)]
pub struct MappingFrame {
    pub session_id: u64,
    pub odom_pose: proto::Pose,
    pub points_body: Vec<[f64; 3]>,
    pub ray_origin_body: [f64; 3],
    pub mapping_only: bool,
    pub sensor_id: String,
    pub registered_cloud: Option<nuwax_codec::lcm::schema::PointCloud2>,
}
#[derive(Clone, Debug)]
pub struct MapperConfig {
    pub map_name: String,
    pub map_frame_id: String,
    pub odom_frame_id: String,
    pub base_frame_id: String,
    pub backend: MappingBackendSpec,
    pub compute: MappingBackendComputeConfig,
    pub correction_backend: String,
    pub cloudmap_shm_path: Option<PathBuf>,
}
#[cfg(test)]
impl Default for MapperConfig {
    fn default() -> Self {
        Self {
            map_name: String::new(),
            map_frame_id: "map".into(),
            odom_frame_id: "odom".into(),
            base_frame_id: "base_link".into(),
            backend: MappingBackendSpec::Voxel,
            compute: MappingBackendComputeConfig {
                resolution: 0.05,
                initial_safe_radius_meters: 0.0,
                window_capacity: 8_000_000,
            },
            correction_backend: "none".into(),
            cloudmap_shm_path: None,
        }
    }
}
impl MapperConfig {
    pub fn from_runtime() -> Result<Self, String> {
        Self::from_config(nuwax_config::geometry()?, nuwax_config::robot()?)
    }
    pub fn from_config(
        g: &nuwax_config::sections::Geometry,
        r: &nuwax_config::sections::Robot,
    ) -> Result<Self, String> {
        let kind = match g.mapping_backend.as_str() {
            "voxel" => MappingBackendKind::Voxel,
            "raytracing" => MappingBackendKind::Raytracing,
            "slam_toolbox" => MappingBackendKind::SlamToolbox,
            v => return Err(format!("未知建图后端：{v}")),
        };
        let planar = g.pipeline == "slam_2d";
        Ok(Self {
            map_name: String::new(),
            map_frame_id: r.map_frame.clone(),
            odom_frame_id: r.odom_frame.clone(),
            base_frame_id: r.base_frame.clone(),
            backend: if planar {
                MappingBackendSpec::SlamToolbox
            } else {
                MappingBackendSpec::from_config(kind, g)?
            },
            compute: MappingBackendComputeConfig {
                resolution: g.mapping_resolution,
                initial_safe_radius_meters: g.mapping_initial_safe_radius_meters as f32,
                window_capacity: 8_000_000,
            },
            correction_backend: if planar {
                "none".into()
            } else {
                g.correction_backend.clone()
            },
            cloudmap_shm_path: Some(crate::pointcloud::cloud_map_shm_path()),
        })
    }
}
#[derive(Clone)]
pub struct MapSnapshot {
    pub occupancy: Option<proto::OccupancyGrid>,
    pub backend_checkpoint: Vec<u8>,
    pub planar_checkpoint: Option<Vec<u8>>,
    pub planar_resume_unavailable_reason: Option<String>,
    pub map_from_odom: [f64; 16],
    pub last_pose: Option<proto::Pose>,
    pub(crate) cloud: crate::pointcloud::CloudSnapshot,
    pub(crate) scans: Vec<(PathBuf, u64)>,
    pub(crate) final_poses: Vec<(u64, [f64; 16])>,
}
#[derive(Clone)]
pub struct MapPublication {
    pub map: Option<proto::MapUpdate>,
    pub fine: Option<Arc<nuwax_map_chunks::SealedMap>>,
    pub forbidden: Option<proto::OccupancyGrid>,
}
#[derive(Clone, Debug)]
pub(crate) struct CorrectedKeyframe {
    pub stamp: u64,
    pub local_pose: Isometry3<f64>,
    pub old: Isometry3<f64>,
    pub new: Isometry3<f64>,
}
#[derive(Clone, Debug)]
pub(crate) struct CorrectionEvent { pub keyframes: Vec<CorrectedKeyframe> }
#[derive(Clone, Default)]
pub(crate) struct RecentPublication {
    pub clear: Option<(u64, Vec<u8>)>,
    pub frame: Option<Vec<u8>>,
}
impl std::ops::Deref for RecentPublication {
    type Target = [u8];
    fn deref(&self) -> &[u8] {
        self.frame.as_deref().unwrap_or_else(|| &self.clear.as_ref().unwrap().1)
    }
}
#[derive(Clone)]
pub struct Mapper {
    session_id: u64,
    planar_view_only: bool,
    sender: mpsc::Sender<Command>,
    latest_input: Arc<parking_lot::Mutex<Option<Input>>>,
    poses: watch::Receiver<Option<proto::Pose>>,
    maps: watch::Receiver<Option<proto::MapUpdate>>,
    fine_root: watch::Receiver<Option<Arc<nuwax_map_chunks::SealedMap>>>,
    errors: watch::Receiver<Option<String>>,
    correction_stopped: watch::Receiver<Option<String>>,
    correction_events: Arc<tokio::sync::Mutex<tokio::sync::mpsc::UnboundedReceiver<CorrectionEvent>>>,
    transform: watch::Receiver<Isometry3<f64>>,
    config: MapperConfig,
    forbidden: watch::Receiver<Option<proto::OccupancyGrid>>,
    forbidden_status: watch::Receiver<Result<(), String>>,
    recent_cloud: watch::Receiver<Option<RecentPublication>>,
    worker: Arc<std::sync::Mutex<Option<std::thread::JoinHandle<()>>>>,
}
enum Input {
    Frame(MappingFrame),
    Grid(proto::OccupancyGrid),
}
enum Command {
    InputReady,
    Snapshot(oneshot::Sender<Result<MapSnapshot, String>>),
    Publication(oneshot::Sender<MapPublication>),
    Display(oneshot::Sender<crate::pointcloud::CloudSnapshot>),
    Surface(oneshot::Sender<Result<Option<Arc<nuwax_map_chunks::SealedMap>>, String>>),
    Dense(
        proto::DenseCloudChunkRequest,
        oneshot::Sender<Result<proto::DenseCloudChunk, String>>,
    ),
    Transform(Isometry3<f64>, oneshot::Sender<Result<(), String>>),
    Name(String),
    Shutdown,
    ActivateShared(oneshot::Sender<Result<(), String>>),
    Forbidden(PathBuf, oneshot::Sender<Result<(), String>>),
}
impl Mapper {
    pub fn start(session_id: u64, config: MapperConfig) -> Result<Arc<Self>, String> {
        Self::spawn(session_id, config, None, false, true).map_err(|error| error.message().into())
    }
    pub fn load(
        session_id: u64,
        config: MapperConfig,
        snapshot: MapSnapshot,
        read_only: bool,
    ) -> Result<Arc<Self>, String> {
        Self::spawn(session_id, config, Some(snapshot), read_only, true)
            .map_err(|error| error.message().into())
    }
    pub fn prepare_load(
        session_id: u64,
        config: MapperConfig,
        snapshot: MapSnapshot,
        read_only: bool,
    ) -> Result<Arc<Self>, Status> {
        Self::spawn(session_id, config, Some(snapshot), read_only, false)
    }
    pub async fn activate_shared_cloud(&self) -> Result<(), String> {
        let (tx, rx) = oneshot::channel();
        self.sender
            .send(Command::ActivateShared(tx))
            .map_err(|_| "建图任务已结束")?;
        rx.await.map_err(|_| "建图任务已结束")?
    }
    fn spawn(
        session_id: u64,
        config: MapperConfig,
        snapshot: Option<MapSnapshot>,
        read_only: bool,
        attach_shared: bool,
    ) -> Result<Arc<Self>, Status> {
        let planar_view_only = snapshot
            .as_ref()
            .is_some_and(|saved| saved.planar_resume_unavailable_reason.is_some());
        let (sender, receiver) = mpsc::channel();
        let latest_input = Arc::new(parking_lot::Mutex::new(None::<Input>));
        let pending_input = latest_input.clone();
        let (forbidden_status_tx, forbidden_status) = watch::channel(Ok(()));
        let (pose_tx, poses) = watch::channel(None);
        let (map_tx, maps) = watch::channel(None);
        let (fine_root_tx, fine_root) = watch::channel(None);
        let (error_tx, errors) = watch::channel(None);
        let (transform_tx, transform) = watch::channel(Isometry3::identity());
        let (forbidden_tx, forbidden) = watch::channel(None);
        let (recent_cloud_tx, recent_cloud) = watch::channel(None);
        let mut worker = Worker::new(
            session_id,
            config.clone(),
            snapshot,
            read_only,
            pose_tx,
            map_tx,
            fine_root_tx,
            transform_tx,
            forbidden_tx,
            recent_cloud_tx,
            forbidden_status_tx,
            attach_shared,
        )?;
        let correction_stopped = worker.correction_stopped.subscribe();
        let correction_events = Arc::new(tokio::sync::Mutex::new(worker.correction_receiver.take().unwrap()));
        let worker_handle = std::thread::Builder::new()
            .name(format!("geometry-map-{session_id}"))
            .spawn(move || loop {
                worker.poll_correction();
                worker.poll_forbidden();
                let command = match receiver.recv_timeout(Duration::from_millis(50)) {
                    Ok(command) => command,
                    Err(mpsc::RecvTimeoutError::Timeout) => continue,
                    Err(mpsc::RecvTimeoutError::Disconnected) => break,
                };
                if matches!(command, Command::Shutdown) {
                    worker.corrector.take();
                    break;
                }
                let result = if matches!(command, Command::InputReady) {
                    let input = pending_input.lock().take();
                    if worker.failure.is_some() {
                        continue;
                    }
                    match input {
                        Some(Input::Frame(frame)) => worker.frame(frame),
                        Some(Input::Grid(grid)) => worker.grid(grid),
                        None => Ok(()),
                    }
                } else {
                    worker.command(command)
                };
                if let Err(error) = result {
                    worker.failure = Some(error.clone());
                    error_tx.send_replace(Some(error));
                }
            })
            .map_err(|e| Status::unavailable(e.to_string()))?;
        Ok(Arc::new(Self {
            session_id,
            planar_view_only,
            sender,
            latest_input,
            poses,
            maps,
            fine_root,
            errors,
            correction_stopped,
            correction_events,
            transform,
            config,
            forbidden,
            forbidden_status,
            recent_cloud,
            worker: Arc::new(std::sync::Mutex::new(Some(worker_handle))),
        }))
    }
    pub fn session_id(&self) -> u64 {
        self.session_id
    }
    pub fn feed(&self, frame: MappingFrame) {
        if frame.session_id == self.session_id {
            self.feed_input(Input::Frame(frame));
        }
    }
    pub fn feed_grid(&self, session_id: u64, grid: proto::OccupancyGrid) {
        if session_id == self.session_id {
            self.feed_input(Input::Grid(grid));
        }
    }
    fn feed_input(&self, input: Input) {
        let mut pending = self.latest_input.lock();
        let notify = pending.is_none();
        *pending = Some(input);
        if notify {
            let _ = self.sender.send(Command::InputReady);
        }
    }
    pub fn forbidden_status(&self) -> Result<(), String> {
        self.forbidden_status.borrow().clone()
    }
    pub fn pose_rx(&self) -> watch::Receiver<Option<proto::Pose>> {
        self.poses.clone()
    }
    pub fn map_rx(&self) -> watch::Receiver<Option<proto::MapUpdate>> {
        self.maps.clone()
    }
    pub fn error_rx(&self) -> watch::Receiver<Option<String>> {
        self.errors.clone()
    }
    pub(crate) async fn next_correction_event(&self) -> Option<CorrectionEvent> {
        self.correction_events.lock().await.recv().await
    }
    pub(crate) fn correction_stopped_rx(&self) -> watch::Receiver<Option<String>> {
        self.correction_stopped.clone()
    }
    pub fn planar_view_only(&self) -> bool {
        self.planar_view_only
    }
    pub fn has_occupancy(&self) -> bool {
        self.maps.borrow().is_some()
    }
    /// 物理栅格与有效栅格都取自已发布的同一份地图更新。
    pub fn occupancy(&self, include_forbidden: bool) -> Option<proto::OccupancyGrid> {
        self.maps.borrow().as_ref().map(|map| proto::OccupancyGrid {
            map_name: map.map_name.clone(),
            meta: map.meta.clone(),
            data: if include_forbidden {
                map.data.clone()
            } else {
                map.raw_data.clone()
            },
        })
    }
    pub fn transform_pose(&self, pose: &proto::Pose) -> proto::Pose {
        pose_from_isometry(
            *self.transform.borrow() * pose_isometry(pose),
            self.maps
                .borrow()
                .as_ref()
                .and_then(|m| m.meta.as_ref())
                .map(|m| m.frame_id.as_str())
                .unwrap_or(&self.config.map_frame_id),
            pose.timestamp_ns,
        )
    }
    pub fn map_from_odom(&self) -> Isometry3<f64> {
        *self.transform.borrow()
    }
    pub fn map_frame_id(&self) -> &str {
        &self.config.map_frame_id
    }
    pub fn correction_backend(&self) -> &str {
        &self.config.correction_backend
    }
    pub fn recent_cloud_rx(&self) -> watch::Receiver<Option<RecentPublication>> {
        self.recent_cloud.clone()
    }
    pub fn forbidden_mask(&self) -> Option<proto::OccupancyGrid> {
        self.forbidden.borrow().clone()
    }
    pub async fn set_map_from_odom(&self, transform: Isometry3<f64>) -> Result<(), String> {
        let (tx, rx) = oneshot::channel();
        self.sender
            .send(Command::Transform(transform, tx))
            .map_err(|_| "建图任务已结束")?;
        rx.await.map_err(|_| "建图任务已结束")?
    }
    pub async fn shutdown(&self) -> Result<(), String> {
        let _ = self.sender.send(Command::Shutdown);
        let worker = self.worker.lock().map_err(|_| "建图任务回收锁损坏")?.take();
        if let Some(worker) = worker {
            tokio::task::spawn_blocking(move || worker.join())
                .await
                .map_err(|e| e.to_string())?
                .map_err(|_| "建图任务崩溃")?;
        }
        Ok(())
    }
    pub fn set_map_name(&self, name: String) {
        let _ = self.sender.send(Command::Name(name));
    }
    pub(crate) async fn display_cloud(&self) -> Result<crate::pointcloud::CloudSnapshot, String> {
        let (tx, rx) = oneshot::channel();
        self.sender
            .send(Command::Display(tx))
            .map_err(|_| "建图任务已结束")?;
        rx.await.map_err(|_| "建图任务已结束".into())
    }
    pub async fn publication(&self) -> Result<MapPublication, String> {
        let (tx, rx) = oneshot::channel();
        self.sender
            .send(Command::Publication(tx))
            .map_err(|_| "建图任务已结束")?;
        rx.await.map_err(|_| "建图任务已结束".into())
    }
    pub async fn snapshot(&self) -> Result<MapSnapshot, String> {
        let (tx, rx) = oneshot::channel();
        self.sender
            .send(Command::Snapshot(tx))
            .map_err(|_| "建图任务已结束")?;
        rx.await.map_err(|_| "建图任务已结束")?
    }
    pub async fn surface_map(&self) -> Result<Option<Arc<nuwax_map_chunks::SealedMap>>, String> {
        let (tx, rx) = oneshot::channel();
        self.sender
            .send(Command::Surface(tx))
            .map_err(|_| "建图任务已结束")?;
        rx.await.map_err(|_| "建图任务已结束")?
    }
    pub async fn dense_chunk(
        &self,
        request: proto::DenseCloudChunkRequest,
    ) -> Result<proto::DenseCloudChunk, String> {
        let (tx, rx) = oneshot::channel();
        self.sender
            .send(Command::Dense(request, tx))
            .map_err(|_| "建图任务已结束")?;
        rx.await.map_err(|_| "建图任务已结束")?
    }
    /// 读取当前精细根，不排入建图命令队列。
    pub fn fine_root(&self) -> Option<Arc<nuwax_map_chunks::SealedMap>> {
        self.fine_root.borrow().clone()
    }
    pub async fn set_forbidden_zones(&self, path: PathBuf) -> Result<(), String> {
        let (tx, rx) = oneshot::channel();
        self.sender
            .send(Command::Forbidden(path, tx))
            .map_err(|_| "建图任务已结束")?;
        rx.await.map_err(|_| "建图任务已结束")?
    }
}
struct Worker {
    session_id: u64,
    config: MapperConfig,
    read_only: bool,
    kernel: Option<MappingKernel>,
    cloud: CloudMap,
    grid: Option<PersistentOccupancyGrid>,
    map_from_odom: Isometry3<f64>,
    /// 在线去动态证据坐标 ← 里程计：会话开始为单位变换，每次采用重放地图后取这次校正的最新值。
    evidence_from_odom: Isometry3<f64>,
    poses: watch::Sender<Option<proto::Pose>>,
    maps: watch::Sender<Option<proto::MapUpdate>>,
    fine_root: watch::Sender<Option<Arc<nuwax_map_chunks::SealedMap>>>,
    transform: watch::Sender<Isometry3<f64>>,
    corrector: Option<CorrectionWorker>,
    correction_stopped: watch::Sender<Option<String>>,
    correction_events: tokio::sync::mpsc::UnboundedSender<CorrectionEvent>,
    correction_receiver: Option<tokio::sync::mpsc::UnboundedReceiver<CorrectionEvent>>,
    sequence: u64,
    forbidden_path: Option<PathBuf>,
    forbidden: watch::Sender<Option<proto::OccupancyGrid>>,
    failure: Option<String>,
    anchored: bool,
    base_anchor: Isometry3<f64>,
    built_timeline: Option<PgoUpdate>,
    built_until: u64,
    built_corrections: Vec<(u64, Isometry3<f64>)>,
    forbidden_zones: Vec<crate::forbidden_zones::ForbiddenZone>,
    forbidden_metadata: Option<(SystemTime, u64)>,
    forbidden_checked_at: Instant,
    forbidden_status: watch::Sender<Result<(), String>>,
    recent_cloud: watch::Sender<Option<RecentPublication>>,
    last_input: Option<MappingBackendInput>,
    forbidden_cells: Option<ForbiddenCells>,
    grid_stamp: u64,
    latest_odometry: Option<proto::Pose>,
    planar_resume_unavailable_reason: Option<String>,
}
impl Worker {
    fn new(
        session_id: u64,
        config: MapperConfig,
        snapshot: Option<MapSnapshot>,
        read_only: bool,
        poses: watch::Sender<Option<proto::Pose>>,
        maps: watch::Sender<Option<proto::MapUpdate>>,
        fine_root: watch::Sender<Option<Arc<nuwax_map_chunks::SealedMap>>>,
        transform: watch::Sender<Isometry3<f64>>,
        forbidden: watch::Sender<Option<proto::OccupancyGrid>>,
        recent_cloud: watch::Sender<Option<RecentPublication>>,
        forbidden_status: watch::Sender<Result<(), String>>,
        attach_shared: bool,
    ) -> Result<Self, Status> {
        let kernel = if config.backend == MappingBackendSpec::SlamToolbox {
            None
        } else {
            Some(
                if let Some(snapshot) = snapshot
                    .as_ref()
                    .filter(|s| !s.backend_checkpoint.is_empty())
                {
                    mapping_backend::restore_mapping_kernel_checkpoint(
                        config.backend,
                        config.compute,
                        &snapshot.backend_checkpoint,
                    )
                    .map_err(Status::failed_precondition)?
                } else {
                    config.backend.into_kernel(config.compute.window_capacity)
                },
            )
        };
        let mut cloud = CloudMap::new(
            session_id,
            config
                .cloudmap_shm_path
                .as_deref()
                .filter(|_| attach_shared),
            snapshot.as_ref().map(|s| s.cloud.clone()),
        )
        .map_err(Status::unavailable)?;
        if let Some(snapshot) = snapshot.as_ref() {
            cloud.load_history(snapshot.scans.clone(), snapshot.final_poses.clone());
        }
        let map_from_odom = snapshot
            .as_ref()
            .map(|s| matrix_isometry(&s.map_from_odom))
            .unwrap_or_else(Isometry3::identity);
        let grid = snapshot
            .as_ref()
            .and_then(|s| s.occupancy.as_ref())
            .map(proto_grid_to_kernel)
            .transpose()
            .map_err(Status::unavailable)?
            .map(PersistentOccupancyGrid::from_grid)
            .transpose()
            .map_err(Status::unavailable)?;
        let corrector = if matches!(config.correction_backend.as_str(), "none" | "relocalization") {
            None
        } else {
            Some(CorrectionWorker::start(session_id, config.clone(),
                Corrector::new(&config.correction_backend).map_err(Status::unavailable)?)
                .map_err(Status::unavailable)?)
        };
        let (correction_events, correction_receiver) = tokio::sync::mpsc::unbounded_channel();
        let mut value = Self {
            session_id,
            config,
            read_only,
            kernel,
            cloud,
            grid,
            map_from_odom,
            evidence_from_odom: Isometry3::identity(),
            poses,
            maps,
            fine_root,
            transform,
            corrector,
            correction_stopped: watch::channel(None).0,
            correction_events,
            correction_receiver: Some(correction_receiver),
            sequence: 0,
            forbidden_path: None,
            forbidden,
            failure: None,
            anchored: snapshot.is_some(),
            base_anchor: map_from_odom,
            built_timeline: None,
            built_until: 0,
            built_corrections: vec![(0, map_from_odom)],
            forbidden_zones: Vec::new(),
            forbidden_metadata: None,
            forbidden_checked_at: Instant::now(),
            forbidden_status,
            recent_cloud,
            last_input: None,
            forbidden_cells: None,
            latest_odometry: snapshot
                .as_ref()
                .and_then(|s| s.last_pose.as_ref())
                .map(|pose| {
                    pose_from_isometry(
                        map_from_odom.inverse() * pose_isometry(pose),
                        "odom",
                        pose.timestamp_ns,
                    )
                }),
            planar_resume_unavailable_reason: snapshot
                .as_ref()
                .and_then(|saved| saved.planar_resume_unavailable_reason.clone()),
            grid_stamp: snapshot
                .as_ref()
                .and_then(|s| s.occupancy.as_ref())
                .and_then(|g| g.meta.as_ref())
                .map_or(0, |m| m.timestamp_ns),
        };
        value.fine_root.send_replace(value.cloud.surface());
        value.transform.send_replace(map_from_odom);
        value.publish_grid().map_err(Status::unavailable)?;
        Ok(value)
    }
    fn command(&mut self, command: Command) -> Result<(), String> {
        match command {
            Command::InputReady => Ok(()),
            Command::Display(reply) => {
                let _ = reply.send(self.cloud.snapshot());
                Ok(())
            }
            Command::Publication(reply) => {
                let map = self.maps.borrow().clone().filter(|map| {
                    map.meta
                        .as_ref()
                        .is_some_and(|meta| meta.timestamp_ns == self.grid_stamp)
                });
                let fine = if map.is_some() {
                    self.cloud.surface()
                } else {
                    None
                };
                let forbidden = if map.is_some() {
                    self.forbidden.borrow().clone()
                } else {
                    None
                };
                let _ = reply.send(MapPublication {
                    map,
                    fine,
                    forbidden,
                });
                Ok(())
            }
            Command::Snapshot(reply) => {
                let result = self.snapshot();
                let _ = reply.send(result);
                Ok(())
            }
            Command::Surface(reply) => {
                let _ = reply.send(Ok(self.cloud.surface()));
                Ok(())
            }
            Command::Dense(request, reply) => {
                let _ = reply.send(self.cloud.dense_chunk(request, &self.config.map_frame_id));
                Ok(())
            }
            Command::Transform(transform, reply) => {
                self.map_from_odom = transform;
                self.transform.send_replace(transform);
                self.publish_pose();
                let _ = reply.send(Ok(()));
                Ok(())
            }
            Command::Shutdown => Ok(()),
            Command::ActivateShared(reply) => {
                let result = match self.config.cloudmap_shm_path.as_deref() {
                    Some(path) => self.cloud.attach_shared(path),
                    None => Ok(()),
                };
                let _ = reply.send(result);
                Ok(())
            }
            Command::Name(name) => {
                self.config.map_name = name;
                if let Err(error) = self.publish_grid() {
                    tracing::warn!(%error,"禁行区尚未刷新，等待输入");
                }
                Ok(())
            }
            Command::Forbidden(path, reply) => {
                let result = if self.forbidden_path.as_ref() != Some(&path) {
                    self.forbidden_path = Some(path);
                    self.forbidden_checked_at = Instant::now();
                    self.refresh_forbidden(true)
                } else {
                    self.forbidden_status.borrow().clone()
                };
                let _ = self.forbidden_status.send_replace(result.clone());
                if result.is_err() {
                    self.maps.send_replace(None);
                }
                let _ = reply.send(result);
                Ok(())
            }
        }
    }
    fn grid(&mut self, grid: proto::OccupancyGrid) -> Result<(), String> {
        if !self.read_only {
            let prepared = PersistentOccupancyGrid::from_grid(proto_grid_to_kernel(&grid)?)?;
            self.grid_stamp = grid.meta.as_ref().map_or(0, |m| m.timestamp_ns);
            self.grid = Some(prepared);
            if let Err(error) = self.publish_grid() {
                tracing::warn!(%error, "禁行区尚未刷新，等待输入");
            }
        }
        Ok(())
    }
    fn refresh_forbidden(&mut self, first_read: bool) -> Result<(), String> {
        let path = self.forbidden_path.as_ref().unwrap();
        let metadata = match std::fs::metadata(path) {
            Ok(metadata) => Some((
                metadata
                    .modified()
                    .map_err(|e| format!("禁行区读取失败：{e}"))?,
                metadata.len(),
            )),
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => None,
            Err(error) => return Err(format!("禁行区读取失败：{error}")),
        };
        if !first_read
            && metadata == self.forbidden_metadata
            && self.forbidden_status.borrow().is_ok()
        {
            return Ok(());
        }
        let zones = crate::forbidden_zones::read(path)?;
        if zones != self.forbidden_zones {
            self.forbidden_cells = None;
        }
        self.forbidden_zones = zones;
        self.forbidden_metadata = metadata;
        let _ = self.forbidden_status.send_replace(Ok(()));
        self.publish_grid()
    }
    fn poll_forbidden(&mut self) {
        if self.forbidden_path.is_none()
            || self.forbidden_checked_at.elapsed() < Duration::from_secs(1)
        {
            return;
        }
        self.forbidden_checked_at = Instant::now();
        let result = self.refresh_forbidden(false);
        let _ = self.forbidden_status.send_replace(result.clone());
        if result.is_err() {
            self.maps.send_replace(None);
        }
    }
    fn frame(&mut self, frame: MappingFrame) -> Result<(), String> {
        if frame.session_id != self.session_id {
            return Ok(());
        }
        self.poll_correction();
        if !self.anchored && !self.read_only {
            let odom = pose_isometry(&frame.odom_pose);
            let (_, _, yaw) = odom.rotation.euler_angles();
            let rotation = UnitQuaternion::from_euler_angles(0., 0., -yaw);
            self.map_from_odom = Isometry3::from_parts(
                Translation3::from(-(rotation * odom.translation.vector)),
                rotation,
            );
            self.base_anchor = self.map_from_odom;
            self.built_corrections.push((frame.odom_pose.timestamp_ns, self.map_from_odom));
            self.anchored = true;
            self.transform.send_replace(self.map_from_odom);
        }
        let odom = pose_isometry(&frame.odom_pose);
        if !frame.mapping_only
            && self
                .latest_odometry
                .as_ref()
                .is_none_or(|pose| pose.timestamp_ns <= frame.odom_pose.timestamp_ns)
        {
            self.latest_odometry = Some(frame.odom_pose.clone());
        }
        self.publish_pose();
        if self.kernel.is_none() {
            return Ok(());
        }
        self.sequence += 1;
        let stamp = frame.odom_pose.timestamp_ns;
        let map_from_body = self.map_from_odom * odom;
        let points: Vec<_> = frame
            .points_body
            .iter()
            .map(|p| map_from_body * Point3::from(*p))
            .collect();
        let origin = map_from_body * Point3::from(frame.ray_origin_body);
        if self.read_only {
            let ground = self.cloud.classify_readonly(
                self.sequence,
                &frame,
                &points,
                [origin.x, origin.y, origin.z],
                self.map_from_odom,
            )?;
            if !frame.mapping_only {
                self.publish_recent(encode_recent_cloud(
                    self.sequence,
                    &frame,
                    &points,
                    &ground,
                    map_from_body,
                    [origin.x, origin.y, origin.z],
                    &self.config,
                )?);
            }
            return Ok(());
        }
        let (dynamic, pgo, ground) = self.cloud.accumulate(
            self.sequence,
            &frame,
            &points,
            [origin.x, origin.y, origin.z],
            self.map_from_odom,
            self.evidence_from_odom,
        )?;
        if self.cloud.take_recording_limit() {
            self.stop_correction("扫描记录已达上限，回环校正已停止".into());
        }
        if !frame.mapping_only {
            self.publish_recent(encode_recent_cloud(
                self.sequence,
                &frame,
                &points,
                &ground,
                map_from_body,
                [origin.x, origin.y, origin.z],
                &self.config,
            )?);
        }
        let input = MappingBackendInput {
            source_timestamp_ns: stamp,
            received_at_ns: stamp,
            registered_frame_id: self.config.map_frame_id.clone(),
            sensor_origin_map: [origin.x, origin.y, origin.z],
            dynamic_frame: Some(dynamic),
            registered_points_map: points,
        };
        let candidate = mapping_backend::compute_candidate_in_place(
            self.kernel.as_mut().unwrap(),
            &input,
            self.config.compute,
        )?;
        let committed_this_frame = candidate.is_some();
        if let Some(candidate) = candidate {
            PersistentOccupancyGrid::merge_or_create(&mut self.grid, &candidate.latest_frame_occupancy)?;
            self.grid_stamp = stamp;
            self.cloud.commit(stamp)?;
            self.fine_root.send_replace(self.cloud.surface());
            if let Err(error) = self.publish_grid() {
                tracing::warn!(%error,"禁行区尚未刷新，等待输入");
            }
        }
        self.last_input = Some(input);
        if committed_this_frame && !frame.mapping_only {
            if let Some(corrector) = &self.corrector {
                corrector.submit(CorrectionInput::new(&frame, &pgo,
                    self.grid.as_ref().map(|g| (&g.grid, self.grid_stamp)), self.base_anchor));
            }
        }
        Ok(())
    }
    fn publish_recent(&self, bytes: Vec<u8>) {
        let clear = self.recent_cloud.borrow().as_ref().and_then(|value| value.clear.clone());
        self.recent_cloud.send_replace(Some(RecentPublication { clear, frame: Some(bytes) }));
    }
    fn stop_correction(&mut self, reason: String) {
        tracing::error!(%reason, "回环校正已停止");
        if let Some(corrector) = &self.corrector { corrector.request_stop(); }
        self.correction_stopped.send_replace(Some(reason));
    }
    fn poll_correction(&mut self) {
        while let Some(update) = self.corrector.as_ref().and_then(|worker| worker.next()) {
            let result = match update {
                correction_worker::Update::Correction(update, reply) => {
                    if self.is_small(&update) {
                        self.apply_small(update);
                        let _ = reply.send(correction_worker::Reply::Adopted);
                        Ok(())
                    } else {
                        self.cloud.replay_prefix().map(|prefix| {
                            let _ = reply.send(correction_worker::Reply::Replay(prefix));
                        })
                    }
                }
                correction_worker::Update::Prefix(reply) => self.cloud.replay_prefix().map(|prefix| { let _ = reply.send(prefix); }),
                correction_worker::Update::Rebuilt(mut rebuilt, reply) => {
                    self.cloud.replay_prefix().and_then(|prefix| {
                        if rebuilt.position.remaining(&prefix) > 5 {
                            let _ = reply.send(correction_worker::Reply::CatchUp(rebuilt, prefix));
                            return Ok(());
                        }
                        rebuilt.replay(&prefix, &std::sync::atomic::AtomicBool::new(false))?;
                        self.adopt_rebuilt(*rebuilt)?;
                        let _ = reply.send(correction_worker::Reply::Adopted);
                        Ok(())
                    })
                }
                correction_worker::Update::Stopped(reason) => Err(reason),
            };
            if let Err(reason) = result {
                self.stop_correction(format!("回环校正已停止：{reason}"));
            }
        }
    }
    fn building_correction(&self, stamp: u64) -> Isometry3<f64> {
        if let Some(timeline) = &self.built_timeline {
            if stamp <= self.built_until {
                return timeline.correction_at(stamp);
            }
        }
        self.built_corrections.iter().rev()
            .find(|(start, _)| *start <= stamp).unwrap().1
    }
    fn is_small(&self, update: &PgoUpdate) -> bool {
        update.local_poses().all(|(stamp, local_pose)| {
            let optimized = update.correction_at(stamp) * local_pose;
            let built = self.building_correction(stamp) * local_pose;
            let delta = built.inverse() * optimized;
            delta.translation.vector.norm() + 10.0 * delta.rotation.angle() < 0.05
        })
    }
    fn apply_small(&mut self, update: PgoUpdate) {
        self.map_from_odom = update.latest;
        self.built_corrections.push((self.last_input.as_ref().unwrap().source_timestamp_ns, update.latest));
        self.transform.send_replace(update.latest);
        self.publish_pose();
    }
    fn adopt_rebuilt(&mut self, mut rebuilt: Rebuilt) -> Result<(), String> {
        let event = CorrectionEvent { keyframes: rebuilt.update.local_poses().map(|(stamp, local_pose)| {
            CorrectedKeyframe { stamp, local_pose, old: self.building_correction(stamp),
                new: rebuilt.update.correction_at(stamp) }
        }).collect() };
        rebuilt.cloud.adopt_publication(&mut self.cloud)?;
        self.cloud = rebuilt.cloud;
        self.fine_root.send_replace(self.cloud.surface());
        self.kernel = Some(rebuilt.kernel);
        self.grid = rebuilt.grid;
        self.grid_stamp = rebuilt.stamp;
        self.map_from_odom = rebuilt.update.latest;
        // 重放地图的去动态证据在校正后的坐标里，之后的在线帧按这次校正的最新值放入同一坐标。
        self.evidence_from_odom = rebuilt.update.latest;
        self.built_until = rebuilt.stamp;
        self.built_corrections.clear();
        self.built_corrections.push((rebuilt.stamp, self.map_from_odom));
        self.built_timeline = Some(rebuilt.update);
        self.transform.send_replace(self.map_from_odom);
        self.publish_pose();
        if let Err(error) = self.publish_grid() {
            tracing::warn!(%error,"禁行区尚未刷新，等待输入");
        }
        self.sequence += 1;
        let bytes = nuwax_codec::lcm::recent_cloud::encode_borrowed(
            &self.session_id.to_string(), self.sequence as i64, None, None, &[]).map_err(|e| e.to_string())?;
        self.recent_cloud.send_replace(Some(RecentPublication { clear: Some((self.sequence, bytes)), frame: None }));
        let _ = self.correction_events.send(event);
        Ok(())
    }
    fn publish_pose(&self) {
        if let Some(odom) = &self.latest_odometry {
            self.poses.send_replace(Some(pose_from_isometry(
                self.map_from_odom * pose_isometry(odom),
                &self.config.map_frame_id,
                odom.timestamp_ns,
            )));
        }
    }
    fn snapshot(&mut self) -> Result<MapSnapshot, String> {
        let stamp = self.poses.borrow().as_ref().map_or(0, |p| p.timestamp_ns);
        let mut flushed = false;
        if let (Some(kernel), Some(input)) = (&mut self.kernel, &self.last_input) {
            if let Some(candidate) =
                mapping_backend::flush_candidate(kernel, input, self.config.compute)?
            {
                PersistentOccupancyGrid::merge_or_create(&mut self.grid, &candidate.latest_frame_occupancy)?;
                self.grid_stamp = input.source_timestamp_ns;
                flushed = true;
            }
        }
        if !self.read_only {
            self.cloud.commit(stamp)?;
            self.fine_root.send_replace(self.cloud.surface());
            if flushed {
                if let Err(error) = self.publish_grid() {
                    tracing::warn!(%error,"禁行区尚未刷新，等待输入");
                }
            }
        }
        Ok(MapSnapshot {
            occupancy: self.grid.as_ref().map(|grid| {
                let mut value = kernel_grid_to_proto(&grid.grid, &self.config.map_name);
                value.meta.as_mut().unwrap().timestamp_ns = self.grid_stamp;
                value
            }),
            backend_checkpoint: self
                .kernel
                .as_ref()
                .map(mapping_backend::export_mapping_kernel_checkpoint)
                .transpose()?
                .unwrap_or_default(),
            planar_checkpoint: None,
            planar_resume_unavailable_reason: self.planar_resume_unavailable_reason.clone(),
            map_from_odom: isometry_matrix(&self.map_from_odom),
            last_pose: self.poses.borrow().clone(),
            cloud: self.cloud.snapshot(),
            scans: self.cloud.scan_prefix()?,
            final_poses: self.cloud.final_poses(),
        })
    }
    fn publish_grid(&mut self) -> Result<(), String> {
        let Some(grid) = self.grid.as_ref() else {
            return Ok(());
        };
        self.forbidden_status.borrow().clone()?;
        let mut raw = kernel_grid_to_proto(&grid.grid, &self.config.map_name);
        let meta = raw.meta.as_mut().unwrap();
        meta.timestamp_ns = self.grid_stamp;
        let layout = (meta.origin_x, meta.origin_y, meta.width, meta.height);
        // 禁行区内容与栅格原点、尺寸都没变时沿用上次的掩码，掩码通道也不重发。
        if self.forbidden_cells.as_ref().is_none_or(|cells| cells.layout != layout) {
            let mut mask = vec![0; raw.data.len()];
            if self.forbidden_path.is_some() {
                crate::forbidden_zones::overlay_zones(&self.forbidden_zones, &grid.grid, &mut mask)?;
            }
            let blocked = mask.iter().enumerate().filter(|(_, value)| **value == 100)
                .map(|(index, _)| index).collect();
            self.forbidden.send_replace(Some(proto::OccupancyGrid {
                map_name: raw.map_name.clone(),
                meta: raw.meta.clone(),
                data: mask,
            }));
            self.forbidden_cells = Some(ForbiddenCells { layout, blocked });
        }
        // 有效栅格 = 物理栅格的一份拷贝再叠加禁行格；没有禁行格时只有这一次拷贝。
        let mut data = raw.data.clone();
        for &index in &self.forbidden_cells.as_ref().unwrap().blocked {
            data[index] = 100;
        }
        self.maps.send_replace(Some(proto::MapUpdate {
            map_name: raw.map_name,
            meta: raw.meta,
            data,
            raw_data: raw.data,
        }));
        Ok(())
    }
}
/// 上次栅格化禁行掩码时的栅格原点与尺寸，以及掩码里的禁行格下标。
struct ForbiddenCells {
    layout: (f64, f64, u32, u32),
    blocked: Vec<usize>,
}
impl Drop for Worker {
    fn drop(&mut self) { self.corrector.take(); }
}
pub fn pose_isometry(pose: &proto::Pose) -> Isometry3<f64> {
    let p = pose.position.clone().unwrap_or_default();
    let q = pose.orientation.clone().unwrap_or(proto::Quat {
        x: 0.,
        y: 0.,
        z: 0.,
        w: 1.,
    });
    Isometry3::from_parts(
        Translation3::new(p.x, p.y, p.z),
        UnitQuaternion::new_normalize(Quaternion::new(q.w, q.x, q.y, q.z)),
    )
}
pub fn pose_from_isometry(value: Isometry3<f64>, frame: &str, stamp: u64) -> proto::Pose {
    let p = value.translation.vector;
    let q = value.rotation.quaternion();
    proto::Pose {
        position: Some(proto::Vec3 {
            x: p.x,
            y: p.y,
            z: p.z,
        }),
        orientation: Some(proto::Quat {
            x: q.i,
            y: q.j,
            z: q.k,
            w: q.w,
        }),
        frame_id: frame.into(),
        timestamp_ns: stamp,
    }
}
pub(crate) fn isometry_matrix(value: &Isometry3<f64>) -> [f64; 16] {
    let matrix = value.to_homogeneous();
    let mut result = [0.; 16];
    result.copy_from_slice(matrix.as_slice());
    result
}
pub(crate) fn matrix_isometry(value: &[f64; 16]) -> Isometry3<f64> {
    let matrix = nalgebra::Matrix4::from_column_slice(value);
    Isometry3::from_parts(
        Translation3::new(matrix[(0, 3)], matrix[(1, 3)], matrix[(2, 3)]),
        UnitQuaternion::from_matrix(&matrix.fixed_view::<3, 3>(0, 0).into_owned()),
    )
}
pub(crate) fn kernel_grid_to_proto(
    grid: &nuwax_mapping::OccupancyGrid,
    name: &str,
) -> proto::OccupancyGrid {
    proto::OccupancyGrid {
        map_name: name.into(),
        meta: Some(proto::OccupancyGridMeta {
            width: grid.width as u32,
            height: grid.height as u32,
            resolution: grid.resolution as f32,
            origin_x: grid.origin[0],
            origin_y: grid.origin[1],
            frame_id: grid.frame_id.clone(),
            timestamp_ns: (grid.ts * 1e9) as u64,
        }),
        data: grid.data.iter().map(|v| *v as u8).collect(),
    }
}
pub(crate) fn proto_grid_to_kernel(
    grid: &proto::OccupancyGrid,
) -> Result<nuwax_mapping::OccupancyGrid, String> {
    let meta = grid.meta.as_ref().ok_or("栅格缺少元数据")?;
    if meta.width > 8192 || meta.height > 8192 {
        return Err("栅格尺寸超过8192上限".into());
    }
    Ok(nuwax_mapping::OccupancyGrid {
        width: meta.width,
        height: meta.height,
        resolution: f64::from(meta.resolution),
        origin: [meta.origin_x, meta.origin_y, 0.],
        origin_orientation: [0., 0., 0., 1.],
        frame_id: meta.frame_id.clone(),
        ts: meta.timestamp_ns as f64 / 1e9,
        data: grid.data.iter().map(|v| *v as i8).collect(),
    })
}
fn encode_recent_cloud(
    sequence: u64,
    frame: &MappingFrame,
    points: &[Point3<f64>],
    ground: &[u32],
    map_from_body: Isometry3<f64>,
    origin: [f64; 3],
    config: &MapperConfig,
) -> Result<Vec<u8>, String> {
    use nuwax_codec::lcm::{
        recent_cloud::{self, RegisteredCloudPose},
        schema::{Header, PointCloud2, PointField},
    };
    let cloud = PointCloud2 {
        header: Header {
            seq: 1,
            stamp_sec: (frame.odom_pose.timestamp_ns / 1_000_000_000) as i32,
            stamp_nsec: (frame.odom_pose.timestamp_ns % 1_000_000_000) as i32,
            frame_id: config.map_frame_id.clone(),
        },
        height: 1,
        width: points.len() as i32,
        fields: ["x", "y", "z"]
            .iter()
            .enumerate()
            .map(|(i, name)| PointField {
                name: (*name).into(),
                offset: (i * 4) as i32,
                datatype: 7,
                count: 1,
            })
            .collect(),
        is_bigendian: false,
        point_step: 12,
        row_step: (points.len() * 12) as i32,
        data: points
            .iter()
            .flat_map(|p| {
                [p.x, p.y, p.z]
                    .into_iter()
                    .flat_map(|v| (v as f32).to_le_bytes())
            })
            .collect(),
        is_dense: true,
    };
    let q = map_from_body.rotation.quaternion();
    let registration = RegisteredCloudPose {
        sensor_origin_map: origin,
        body_position_map: map_from_body.translation.vector.into(),
        body_orientation_xyzw: [q.i, q.j, q.k, q.w],
        paired_odometry_timestamp_ns: frame.odom_pose.timestamp_ns as i64,
    };
    recent_cloud::encode_borrowed(
        &frame.session_id.to_string(),
        sequence as i64,
        Some(&cloud),
        Some(&registration),
        ground,
    )
    .map_err(|e| e.to_string())
}

#[cfg(test)]
#[path = "mapper_tests.rs"]
pub(crate) mod tests;
