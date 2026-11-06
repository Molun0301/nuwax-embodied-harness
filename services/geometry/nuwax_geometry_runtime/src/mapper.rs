//! 当前会话的地图拥有方；实时帧直达本对象，控制操作用自身队列形成检查点。
use crate::{
    corrector::{Corrector, PgoUpdate},
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
    pub fn from_env() -> Result<Self, String> {
        let mut config = Self::default();
        for (key, target) in [
            ("NUWAX_MAP_FRAME", &mut config.map_frame_id),
            ("NUWAX_GEOMETRY_ODOM_FRAME", &mut config.odom_frame_id),
            ("NUWAX_BASE_FRAME", &mut config.base_frame_id),
            (
                "NUWAX_GEOMETRY_CORRECTION_BACKEND",
                &mut config.correction_backend,
            ),
        ] {
            if let Ok(value) = std::env::var(key) {
                *target = value;
            }
        }
        let kind = match std::env::var("NUWAX_GEOMETRY_MAPPING_BACKEND")
            .unwrap_or_else(|_| "voxel".into())
            .as_str()
        {
            "voxel" => MappingBackendKind::Voxel,
            "raytracing" => MappingBackendKind::Raytracing,
            "slam_toolbox" => MappingBackendKind::SlamToolbox,
            value => return Err(format!("未知建图后端：{value}")),
        };
        config.backend =
            if std::env::var("NUWAX_GEOMETRY_PIPELINE").ok().as_deref() == Some("slam_2d") {
                config.correction_backend = "none".into();
                MappingBackendSpec::SlamToolbox
            } else {
                MappingBackendSpec::from_env(kind)?
            };
        config.compute.resolution = std::env::var("NUWAX_MAPPING_RESOLUTION")
            .ok()
            .and_then(|value| value.trim().parse::<f64>().ok())
            .filter(|v| v.is_finite() && *v > 0.)
            .unwrap_or(0.05);
        config.compute.initial_safe_radius_meters =
            std::env::var("NUWAX_MAPPING_INITIAL_SAFE_RADIUS_METERS")
                .ok()
                .and_then(|value| value.trim().parse::<f32>().ok())
                .filter(|v| v.is_finite() && *v >= 0.)
                .unwrap_or(0.);
        config.cloudmap_shm_path = Some(crate::pointcloud::cloud_map_shm_path());
        Ok(config)
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
#[derive(Clone)]
pub struct Mapper {
    session_id: u64,
    sender: mpsc::Sender<Command>,
    latest_input: Arc<parking_lot::Mutex<Option<Input>>>,
    poses: watch::Receiver<Option<proto::Pose>>,
    maps: watch::Receiver<Option<proto::MapUpdate>>,
    errors: watch::Receiver<Option<String>>,
    transform: watch::Receiver<Isometry3<f64>>,
    config: MapperConfig,
    forbidden: watch::Receiver<Option<proto::OccupancyGrid>>,
    forbidden_status: watch::Receiver<Result<(), String>>,
    recent_cloud: watch::Receiver<Option<Vec<u8>>>,
    raw_grid: watch::Receiver<Option<proto::OccupancyGrid>>,
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
    Block(
        nuwax_map_chunks::RootIdentity,
        nuwax_map_chunks::BlockRef,
        Option<(usize, usize)>,
        oneshot::Sender<Result<nuwax_map_chunks::ObjectRangeRead, String>>,
    ),
}
impl Mapper {
    pub fn start(session_id: u64, config: MapperConfig) -> Result<Arc<Self>, String> {
        Self::spawn(session_id, config, None, false, true)
    }
    pub fn load(
        session_id: u64,
        config: MapperConfig,
        snapshot: MapSnapshot,
        read_only: bool,
    ) -> Result<Arc<Self>, String> {
        Self::spawn(session_id, config, Some(snapshot), read_only, true)
    }
    pub fn prepare_load(
        session_id: u64,
        config: MapperConfig,
        snapshot: MapSnapshot,
        read_only: bool,
    ) -> Result<Arc<Self>, String> {
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
    ) -> Result<Arc<Self>, String> {
        let (sender, receiver) = mpsc::channel();
        let latest_input = Arc::new(parking_lot::Mutex::new(None::<Input>));
        let pending_input = latest_input.clone();
        let (forbidden_status_tx, forbidden_status) = watch::channel(Ok(()));
        let (pose_tx, poses) = watch::channel(None);
        let (map_tx, maps) = watch::channel(None);
        let (error_tx, errors) = watch::channel(None);
        let (transform_tx, transform) = watch::channel(Isometry3::identity());
        let (forbidden_tx, forbidden) = watch::channel(None);
        let (recent_cloud_tx, recent_cloud) = watch::channel(None);
        let (raw_grid_tx, raw_grid) = watch::channel(None);
        let mut worker = Worker::new(
            session_id,
            config.clone(),
            snapshot,
            read_only,
            pose_tx,
            map_tx,
            transform_tx,
            forbidden_tx,
            recent_cloud_tx,
            raw_grid_tx,
            forbidden_status_tx,
            attach_shared,
        )?;
        let worker_handle = std::thread::Builder::new()
            .name(format!("geometry-map-{session_id}"))
            .spawn(move || loop {
                worker.poll_forbidden();
                let command = match receiver.recv_timeout(Duration::from_millis(50)) {
                    Ok(command) => command,
                    Err(mpsc::RecvTimeoutError::Timeout) => continue,
                    Err(mpsc::RecvTimeoutError::Disconnected) => break,
                };
                if matches!(command, Command::Shutdown) {
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
            .map_err(|e| e.to_string())?;
        Ok(Arc::new(Self {
            session_id,
            sender,
            latest_input,
            poses,
            maps,
            errors,
            transform,
            config,
            forbidden,
            forbidden_status,
            recent_cloud,
            raw_grid,
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
    pub fn occupancy(&self, include_forbidden: bool) -> Option<proto::OccupancyGrid> {
        if !include_forbidden {
            return self.raw_grid.borrow().clone();
        }
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
    pub fn recent_cloud_rx(&self) -> watch::Receiver<Option<Vec<u8>>> {
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
    pub async fn navigation_map_block(
        &self,
        root: nuwax_map_chunks::RootIdentity,
        reference: nuwax_map_chunks::BlockRef,
        range: Option<(usize, usize)>,
    ) -> Result<nuwax_map_chunks::ObjectRangeRead, String> {
        let (tx, rx) = oneshot::channel();
        self.sender
            .send(Command::Block(root, reference, range, tx))
            .map_err(|_| "建图任务已结束")?;
        rx.await.map_err(|_| "建图任务已结束")?
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
    poses: watch::Sender<Option<proto::Pose>>,
    maps: watch::Sender<Option<proto::MapUpdate>>,
    transform: watch::Sender<Isometry3<f64>>,
    corrector: Corrector,
    sequence: u64,
    forbidden_path: Option<PathBuf>,
    forbidden: watch::Sender<Option<proto::OccupancyGrid>>,
    failure: Option<String>,
    anchored: bool,
    base_anchor: Isometry3<f64>,
    forbidden_zones: Vec<crate::forbidden_zones::ForbiddenZone>,
    forbidden_metadata: Option<(SystemTime, u64)>,
    forbidden_checked_at: Instant,
    forbidden_status: watch::Sender<Result<(), String>>,
    recent_cloud: watch::Sender<Option<Vec<u8>>>,
    last_input: Option<MappingBackendInput>,
    raw_grid: watch::Sender<Option<proto::OccupancyGrid>>,
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
        transform: watch::Sender<Isometry3<f64>>,
        forbidden: watch::Sender<Option<proto::OccupancyGrid>>,
        recent_cloud: watch::Sender<Option<Vec<u8>>>,
        raw_grid: watch::Sender<Option<proto::OccupancyGrid>>,
        forbidden_status: watch::Sender<Result<(), String>>,
        attach_shared: bool,
    ) -> Result<Self, String> {
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
                    )?
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
        )?;
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
            .transpose()?
            .map(PersistentOccupancyGrid::from_grid)
            .transpose()?;
        let corrector = Corrector::new(&config.correction_backend)?;
        let mut value = Self {
            session_id,
            config,
            read_only,
            kernel,
            cloud,
            grid,
            map_from_odom,
            poses,
            maps,
            transform,
            corrector,
            sequence: 0,
            forbidden_path: None,
            forbidden,
            failure: None,
            anchored: snapshot.is_some(),
            base_anchor: map_from_odom,
            forbidden_zones: Vec::new(),
            forbidden_metadata: None,
            forbidden_checked_at: Instant::now(),
            forbidden_status,
            recent_cloud,
            last_input: None,
            raw_grid,
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
        value.transform.send_replace(map_from_odom);
        value.publish_grid()?;
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
            Command::Block(root, reference, range, reply) => {
                let _ = reply.send(self.cloud.navigation_map_block(&root, &reference, range));
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
        self.forbidden_zones = crate::forbidden_zones::read(path)?;
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
        if !self.anchored && !self.read_only {
            let odom = pose_isometry(&frame.odom_pose);
            let (_, _, yaw) = odom.rotation.euler_angles();
            let rotation = UnitQuaternion::from_euler_angles(0., 0., -yaw);
            self.map_from_odom = Isometry3::from_parts(
                Translation3::from(-(rotation * odom.translation.vector)),
                rotation,
            );
            self.base_anchor = self.map_from_odom;
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
                self.recent_cloud.send_replace(Some(encode_recent_cloud(
                    &frame,
                    &points,
                    &ground,
                    map_from_body,
                    [origin.x, origin.y, origin.z],
                    &self.config,
                )?));
            }
            return Ok(());
        }
        let (dynamic, pgo, ground) = self.cloud.accumulate(
            self.sequence,
            &frame,
            &points,
            [origin.x, origin.y, origin.z],
            self.map_from_odom,
        )?;
        if !frame.mapping_only {
            self.recent_cloud.send_replace(Some(encode_recent_cloud(
                &frame,
                &points,
                &ground,
                map_from_body,
                [origin.x, origin.y, origin.z],
                &self.config,
            )?));
        }
        let input = MappingBackendInput {
            source_timestamp_ns: stamp,
            received_at_ns: stamp,
            registered_frame_id: self.config.map_frame_id.clone(),
            sensor_origin_map: [origin.x, origin.y, origin.z],
            dynamic_frame: Some(dynamic),
            registered_points_map: points,
        };
        if let Some(candidate) = mapping_backend::compute_candidate_in_place(
            self.kernel.as_mut().unwrap(),
            &input,
            self.config.compute,
        )? {
            self.grid = Some(PersistentOccupancyGrid::stage_observation(
                self.grid.as_ref(),
                &candidate.latest_frame_occupancy,
            )?);
            self.grid_stamp = stamp;
            self.cloud.commit(stamp)?;
            if let Err(error) = self.publish_grid() {
                tracing::warn!(%error,"禁行区尚未刷新，等待输入");
            }
        }
        self.last_input = Some(input);
        if !frame.mapping_only {
            if let Some(update) = self.corrector.process_frame(
                &frame,
                &pgo,
                self.grid.as_ref().map(|g| &g.grid),
                self.base_anchor,
            )? {
                self.apply_pgo(update)?;
            }
        }
        Ok(())
    }
    fn apply_pgo(&mut self, update: PgoUpdate) -> Result<(), String> {
        // 原始扫描按优化后的关键帧轨迹逐帧重建，数值内核仍为同一组内核。
        let frames = self.cloud.replay_frames()?;
        let mut rebuilt = CloudMap::new(self.session_id, None, None)?;
        rebuilt.disable_recording();
        let mut kernel = self
            .config
            .backend
            .into_kernel(self.config.compute.window_capacity);
        let mut persistent = None;
        for (index, frame) in frames.iter().enumerate() {
            let correction = update.correction_at(frame.odom_pose.timestamp_ns);
            let body = correction * pose_isometry(&frame.odom_pose);
            let points: Vec<_> = frame
                .points_body
                .iter()
                .map(|p| body * Point3::from(*p))
                .collect();
            let origin = body * Point3::from(frame.ray_origin_body);
            let (dynamic, _, _) = rebuilt.accumulate(
                index as u64 + 1,
                frame,
                &points,
                [origin.x, origin.y, origin.z],
                correction,
            )?;
            let input = MappingBackendInput {
                source_timestamp_ns: frame.odom_pose.timestamp_ns,
                received_at_ns: frame.odom_pose.timestamp_ns,
                registered_frame_id: self.config.map_frame_id.clone(),
                sensor_origin_map: [origin.x, origin.y, origin.z],
                dynamic_frame: Some(dynamic),
                registered_points_map: points,
            };
            if let Some(candidate) = mapping_backend::compute_candidate_in_place_with_interval(
                &mut kernel,
                &input,
                self.config.compute,
                0,
            )? {
                persistent = Some(PersistentOccupancyGrid::stage_observation(
                    persistent.as_ref(),
                    &candidate.latest_frame_occupancy,
                )?);
            }
        }
        rebuilt.commit(frames.last().map_or(0, |f| f.odom_pose.timestamp_ns))?;
        rebuilt.adopt_publication(&mut self.cloud)?;
        self.cloud = rebuilt;
        self.kernel = Some(kernel);
        self.grid = persistent;
        self.grid_stamp = frames.last().map_or(0, |f| f.odom_pose.timestamp_ns);
        self.map_from_odom = update.latest;
        self.transform.send_replace(update.latest);
        self.publish_pose();
        self.publish_grid()
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
                self.grid = Some(PersistentOccupancyGrid::stage_observation(
                    self.grid.as_ref(),
                    &candidate.latest_frame_occupancy,
                )?);
                self.grid_stamp = input.source_timestamp_ns;
                flushed = true;
            }
        }
        if !self.read_only {
            self.cloud.commit(stamp)?;
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
        let mut raw = kernel_grid_to_proto(&grid.grid, &self.config.map_name);
        raw.meta.as_mut().unwrap().timestamp_ns = self.grid_stamp;
        self.raw_grid.send_replace(Some(raw.clone()));
        let mut data = raw.data.clone();
        let mut mask = vec![0; data.len()];
        self.forbidden_status.borrow().clone()?;
        if self.forbidden_path.is_some() {
            crate::forbidden_zones::overlay_zones(&self.forbidden_zones, &grid.grid, &mut mask)?;
            for (index, value) in mask.iter().enumerate() {
                if *value == 100 {
                    data[index] = 100
                }
            }
        }
        self.forbidden.send_replace(Some(proto::OccupancyGrid {
            map_name: raw.map_name.clone(),
            meta: raw.meta.clone(),
            data: mask,
        }));
        self.maps.send_replace(Some(proto::MapUpdate {
            map_name: raw.map_name,
            meta: raw.meta,
            data,
            raw_data: raw.data,
        }));
        Ok(())
    }
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
    frame: &MappingFrame,
    points: &[Point3<f64>],
    ground: &[u32],
    map_from_body: Isometry3<f64>,
    origin: [f64; 3],
    config: &MapperConfig,
) -> Result<Vec<u8>, String> {
    use nuwax_codec::lcm::{
        lio::{LioCloudEnvelope, LIO_REGISTRATION_REGISTERED_MAP, LIO_SCHEMA_VERSION_V1},
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
    // B3 删除旧结构前，这些线上必填字段只在编码边界填常量，不形成核心身份。
    let envelope = LioCloudEnvelope {
        schema_version: LIO_SCHEMA_VERSION_V1,
        backend: match std::env::var("NUWAX_GEOMETRY_LIO_BACKEND").ok().as_deref() {
            Some("pointlio") => 3,
            Some("fastlio2") => 2,
            _ => 1,
        },
        runtime_generation: 1,
        source_sequence: 1,
        source_timestamp_ns: frame.odom_pose.timestamp_ns as i64,
        correlated_odometry_source_sequence: 1,
        correlated_odometry_source_timestamp_ns: frame.odom_pose.timestamp_ns as i64,
        sensor_frame_id: frame.sensor_id.clone(),
        body_frame_id: config.base_frame_id.clone(),
        odom_frame_id: config.odom_frame_id.clone(),
        map_frame_id: config.map_frame_id.clone(),
        registration_state: LIO_REGISTRATION_REGISTERED_MAP,
        source_point_count: points.len() as i64,
        emitted_point_count: points.len() as i64,
        cloud,
    };
    let q = map_from_body.rotation.quaternion();
    let registration = RegisteredCloudPose {
        sensor_origin_map: origin,
        body_position_map: map_from_body.translation.vector.into(),
        body_orientation_xyzw: [q.i, q.j, q.k, q.w],
        registration_correction_revision: 0,
    };
    recent_cloud::encode_borrowed(
        1,
        &frame.session_id.to_string(),
        1,
        0,
        [1; 16],
        Some(&envelope),
        Some(&registration),
        ground,
    )
    .map_err(|e| e.to_string())
}

#[cfg(test)]
#[path = "mapper_tests.rs"]
pub(crate) mod tests;
