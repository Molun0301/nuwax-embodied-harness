//! 单一原生线程拥有算法。控制回执可重试，扫描只保留最新待处理帧。
use crate::{
    input::{compose, deskew_scan, inverse, normalize, raw_scan, PoseHistory, TimedPose},
    protocol::*,
};
use nuwax_codec::lcm::planar::PlanarScan;
use nuwax_rpc::{RpcHandler, RpcRequest, RpcResponse};
use nuwax_slam_toolbox::{Config, Disposition, Pose, ScanInputMode, Slam};
use serde_json::Value;
use std::{
    collections::VecDeque,
    fs::OpenOptions,
    io::Write,
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, AtomicU64, Ordering},
        Arc, Condvar, Mutex,
    },
    thread::JoinHandle,
    time::Instant,
};
use tokio::sync::{oneshot, watch};

struct PendingControl {
    request: Request,
    reply: oneshot::Sender<Response>,
}
struct ScanPacket {
    generation: u64,
    scan: PlanarScan,
}
#[derive(Default)]
struct Inbox {
    commands: VecDeque<PendingControl>,
    scan: Option<ScanPacket>,
    odometry_changed: bool,
    closed: bool,
}
struct Shared {
    identity: Identity,
    status: Mutex<Status>,
    history: Mutex<PoseHistory>,
    inbox: Mutex<Inbox>,
    wake: Condvar,
    session_generation: AtomicU64,
    stopped: AtomicBool,
    exit: watch::Sender<bool>,
    updates: watch::Sender<Status>,
}
#[derive(Clone)]
pub struct WorkerHandle {
    shared: Arc<Shared>,
}
pub struct WorkerOwner {
    handle: WorkerHandle,
    thread: Option<JoinHandle<()>>,
}
impl WorkerOwner {
    pub fn start(identity: Identity, directory: PathBuf) -> Result<Self, String> {
        identity.validate()?;
        let shared = Arc::new(Shared {
            identity,
            status: Mutex::new(Status::default()),
            history: Mutex::new(PoseHistory::default()),
            inbox: Mutex::new(Inbox::default()),
            wake: Condvar::new(),
            session_generation: AtomicU64::new(0),
            stopped: AtomicBool::new(false),
            exit: watch::channel(false).0,
            updates: watch::channel(Status::default()).0,
        });
        let handle = WorkerHandle {
            shared: shared.clone(),
        };
        let thread = std::thread::Builder::new()
            .name("slam2d-algorithm".into())
            .spawn(move || run(shared, directory))
            .map_err(|e| e.to_string())?;
        Ok(Self {
            handle,
            thread: Some(thread),
        })
    }
    pub fn handle(&self) -> WorkerHandle {
        self.handle.clone()
    }
    pub fn status_receiver(&self) -> watch::Receiver<Status> { self.handle.shared.updates.subscribe() }
    pub fn exit_receiver(&self) -> watch::Receiver<bool> {
        self.handle.shared.exit.subscribe()
    }
    pub fn close_and_join(&mut self) -> Result<(), String> {
        {
            let mut inbox = self.handle.shared.inbox.lock().unwrap();
            inbox.closed = true;
            inbox.scan = None;
            self.handle.shared.wake.notify_all();
        }
        if let Some(thread) = self.thread.take() {
            thread
                .join()
                .map_err(|_| "二维算法线程异常退出".to_owned())?;
        }
        Ok(())
    }
}
impl Drop for WorkerOwner {
    fn drop(&mut self) {
        let _ = self.close_and_join();
    }
}
// 历史与就绪状态只在这段短临界区共同核对，原生匹配/构图不持该锁。
fn accept_odometry(shared:&Shared,sample:&TimedPose)->Result<(),String> {
    let mut history=shared.history.lock().unwrap();
    let status=shared.status.lock().unwrap();
    if !status.active || status.stopping || shared.stopped.load(Ordering::Acquire) {
        return Err("二维会话未开始或已停止".into());
    }
    let session=status.session.as_ref().ok_or("二维会话未绑定")?;
    if shared.session_generation.load(Ordering::Acquire)!=session.session_generation
        || sample.odom_frame_id!=session.odom_frame_id || sample.body_frame_id!=session.body_frame_id
        || sample.clock_domain!=session.clock_domain || sample.clock_id!=session.clock_id {
        return Err("二维里程计坐标、时钟或活动代次不属于当前会话".into());
    }
    history.push(sample.clone())
}

impl WorkerHandle {
    pub fn reject_input(&self, error: String) {
        let mut status = self.shared.status.lock().unwrap();
        status.rejected_scans += 1;
        status.last_error = Some(error);
    }
    pub fn accept_scan(&self, channel: &str, scan: PlanarScan) {
        let generation = self.shared.session_generation.load(Ordering::Acquire);
        if generation == 0
            || channel != self.shared.identity.scan_channel(generation)
            || scan.runtime_generation != self.shared.identity.runtime_generation as i64
        {
            return;
        }
        let mut inbox = self.shared.inbox.lock().unwrap();
        if inbox.closed {
            return;
        }
        let replaced = inbox
            .scan
            .replace(ScanPacket { generation, scan })
            .is_some();
        {
            let mut status = self.shared.status.lock().unwrap();
            status.received_scans += 1;
            if replaced {
                status.dropped_scans += 1;
            }
        }
        self.shared.wake.notify_one();
    }
    fn response(&self, sequence: u64, error: Option<String>) -> Response {
        Response {
            version: VERSION,
            identity: self.shared.identity.clone(),
            command_sequence: sequence,
            status: self.shared.status.lock().unwrap().clone(),
            snapshot: None,
            artifact: None,
            error,
        }
    }
}
#[async_trait::async_trait]
impl RpcHandler for WorkerHandle {
    async fn handle(&self, rpc: RpcRequest) -> RpcResponse {
        let parse = || -> Result<Request, String> {
            if rpc.args.0.len() != 1 || !rpc.args.1.is_empty() {
                return Err("二维控制参数必须为一个命令对象".into());
            }
            let request: Request =
                serde_json::from_value(rpc.args.0[0].clone()).map_err(|e| e.to_string())?;
            if request.version != VERSION || request.identity != self.shared.identity {
                return Err("二维控制版本或拥有方身份不匹配".into());
            }
            Ok(request)
        };
        let request = match parse() {
            Ok(request) => request,
            Err(e) => return RpcResponse::error(rpc.id, Value::String(e)),
        };
        if matches!(request.operation, Operation::Status) {
            return RpcResponse::ok(
                rpc.id,
                serde_json::to_value(self.response(request.command_sequence, None)).unwrap(),
            );
        }
        let sequence = request.command_sequence;
        if let Operation::Odometry{sample}=&request.operation {
            let error={
                let mut inbox=self.shared.inbox.lock().unwrap();
                if inbox.closed {Some("二维输入拥有方已关闭".into())}
                else {
                    let result=accept_odometry(&self.shared,sample);
                    if result.is_ok() {inbox.odometry_changed=true;self.shared.wake.notify_one();}
                    result.err()
                }
            };
            return RpcResponse::ok(rpc.id,serde_json::to_value(self.response(sequence,error)).unwrap());
        }
        let (tx, rx) = oneshot::channel();
        {
            let mut inbox = self.shared.inbox.lock().unwrap();
            if inbox.closed || inbox.commands.len() >= 8 {
                return RpcResponse::ok(
                    rpc.id,
                    serde_json::to_value(
                        self.response(sequence, Some("二维控制队列已关闭或达到上限".into())),
                    )
                    .unwrap(),
                );
            }
            inbox
                .commands
                .push_back(PendingControl { request, reply: tx });
            self.shared.wake.notify_one();
        }
        match rx.await {
            Ok(result) => RpcResponse::ok(rpc.id, serde_json::to_value(result).unwrap()),
            Err(_) => RpcResponse::error(rpc.id, Value::String("二维算法拥有方已退出".into())),
        }
    }
}
struct Algorithm {
    slam: Option<Slam>,
    session: Option<Session>,
    device_anchor: Option<[f64; 3]>,
    waiting: Option<(ScanPacket, Instant)>,
    last_graph_estimate: Option<Estimate>,
    commands: VecDeque<(Request, Response)>,
    last_command: u64,
    highest_session: u64,
    snapshot_counter: u64,
    snapshot: Option<Snapshot>,
    pending_grid: Option<Estimate>,
    directory: PathBuf,
    artifacts: Vec<FileArtifact>,
    loaded: Option<crate::archive::ArchiveMetadata>,
}
impl Algorithm {
    fn matches(&self, id: &str, generation: u64) -> bool {
        self.session
            .as_ref()
            .is_some_and(|s| s.session_id == id && s.session_generation == generation)
    }
    fn clear_snapshot(&mut self) -> Result<(), String> {
        self.pending_grid = None;
        if let Some(slam) = self.slam.as_mut() { slam.cancel_grid().map_err(|e| e.to_string())?; }
        if let Some(snapshot) = self.snapshot.as_ref() {
            match std::fs::remove_file(self.directory.join(&snapshot.file_name)) {
                Ok(()) => {}
                Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
                Err(error) => return Err(error.to_string()),
            }
            self.snapshot = None;
        }
        Ok(())
    }
    fn control(&mut self, request: &Request, shared: &Shared) -> Response {
        let mut response = Response {
            version: VERSION,
            identity: shared.identity.clone(),
            command_sequence: request.command_sequence,
            status: Status::default(),
            snapshot: None,
            artifact: None,
            error: None,
        };
        if request.operation.mutates() {
            if let Some((original, result)) = self
                .commands
                .iter()
                .find(|(r, _)| r.command_sequence == request.command_sequence)
            {
                return if original == request {
                    result.clone()
                } else {
                    response.error = Some("同一控制序号的参数冲突".into());
                    response.status = shared.status.lock().unwrap().clone();
                    response
                };
            }
            if request.command_sequence == 0 || request.command_sequence <= self.last_command {
                response.error = Some("控制序号过期，不能重复执行".into());
                response.status = shared.status.lock().unwrap().clone();
                return response;
            }
        }
        let result = (|| -> Result<(), String> {
            match &request.operation {
                Operation::Status => {}
                Operation::Start { session } => {
                    session.validate()?;
                    if shared.stopped.load(Ordering::Acquire) {
                        return Err("二维拥有方正在关闭".into());
                    }
                    let active = shared.status.lock().unwrap().active;
                    if self.session.as_ref() == Some(session) && self.slam.is_some() && active {
                        return Ok(());
                    }
                    if active {
                        return Err("二维算法已有活动会话".into());
                    }
                    if session.session_generation <= self.highest_session {
                        return Err("二维会话代次未递增".into());
                    }
                    let config: Config = serde_json::from_value(session.core_config.clone())
                        .map_err(|e| e.to_string())?;
                    config.validate().map_err(|e| e.to_string())?;
                    self.clear_snapshot()?;
                    self.slam.take();
                    let slam = Slam::new(config).map_err(|e| e.to_string())?;
                    self.slam = Some(slam);
                    self.session = Some(session.clone());
                    self.highest_session = session.session_generation;
                    shared.history.lock().unwrap().clear();
                    self.device_anchor = None;
                    self.waiting = None;
                    self.last_graph_estimate = None;
                    self.loaded = None;
                    let mut status = shared.status.lock().unwrap();
                    *status = Status {
                        session: Some(session.clone()),
                        active: true,
                        ..Status::default()
                    };
                    shared
                        .session_generation
                        .store(session.session_generation, Ordering::Release);
                }
                Operation::Stop {
                    session_id,
                    session_generation,
                } => {
                    if !self.matches(session_id, *session_generation) {
                        return Err("停止请求不属于当前二维会话".into());
                    }
                    shared.session_generation.store(0, Ordering::Release);
                    self.waiting = None;
                    shared.history.lock().unwrap().clear();
                    // 停止接纳后保留静止图，允许最后快照和后续资产导出。
                    shared.status.lock().unwrap().active = false;
                }
                Operation::Odometry { sample } => {
                    accept_odometry(shared,sample)?;
                }
                Operation::Snapshot {
                    session_id,
                    session_generation,
                } => {
                    if !self.matches(session_id, *session_generation) {
                        return Err("快照不属于当前二维会话".into());
                    }
                    if let Some(snapshot) = &self.snapshot {
                        response.snapshot = Some(snapshot.clone());
                        return Ok(());
                    }
                    if self.pending_grid.is_none() {
                        let estimate = self.last_graph_estimate.clone().ok_or("没有已接纳的二维图")?;
                        self.slam.as_mut().ok_or("二维会话已停止")?.begin_grid(estimate.graph_revision).map_err(|e| e.to_string())?;
                        self.pending_grid = Some(estimate);
                    }
                    let Some(grid) = self.slam.as_mut().ok_or("二维会话已停止")?.poll_grid().map_err(|e| e.to_string())? else {
                        return Ok(());
                    };
                    let estimate = self.pending_grid.take().unwrap();
                    if grid.metadata.graph_revision != estimate.graph_revision {
                        return Err("二维图和位姿修订不一致".into());
                    }
                    self.snapshot_counter = self
                        .snapshot_counter
                        .checked_add(1)
                        .ok_or("二维快照序号耗尽")?;
                    let file_name = format!("candidate-{:016x}.bin", self.snapshot_counter);
                    let bytes: Vec<u8> = grid.cells.iter().map(|v| *v as u8).collect();
                    let path = self.directory.join(&file_name);
                    let mut file = OpenOptions::new()
                        .write(true)
                        .create_new(true)
                        .open(&path)
                        .map_err(|e| e.to_string())?;
                    // 与三维共享映射一致：临时候选只保证完整可读，不承担资产断电持久性。
                    let write = file.write_all(&bytes).map_err(|e| e.to_string());
                    if let Err(error) = write {
                        let _ = std::fs::remove_file(path);
                        return Err(error);
                    }
                    let snapshot = Snapshot {
                        snapshot_id: self.snapshot_counter,
                        session: self.session.clone().unwrap(),
                        estimate,
                        width: grid.metadata.width,
                        height: grid.metadata.height,
                        resolution: grid.metadata.resolution,
                        origin: [grid.metadata.origin_x, grid.metadata.origin_y],
                        payload_size: bytes.len() as u64,
                        file_name,
                    };
                    self.snapshot = Some(snapshot.clone());
                    response.snapshot = Some(snapshot);
                }
                Operation::ReleaseSnapshot { snapshot_id } => {
                    if let Some(snapshot) = &self.snapshot {
                        if snapshot.snapshot_id != *snapshot_id {
                            return Err("二维快照确认身份错误".into());
                        }
                    }
                    self.clear_snapshot()?;
                }
                Operation::SetPaused {
                    session_id,
                    session_generation,
                    paused,
                } => {
                    if !self.matches(session_id, *session_generation) {
                        return Err("暂停请求会话不匹配".into());
                    }
                    if !paused {
                        self.slam
                            .as_mut()
                            .ok_or("二维核心未加载")?
                            .resume_after_pause()
                            .map_err(|e| e.to_string())?;
                    }
                    shared.session_generation.store(
                        if *paused { 0 } else { *session_generation },
                        Ordering::Release,
                    );
                    self.waiting = None;
                    shared.inbox.lock().unwrap().scan = None;
                    let mut status = shared.status.lock().unwrap();
                    status.paused = *paused;
                    status.active = !*paused;
                }
                Operation::ExportGraph {
                    session_id,
                    session_generation,
                    unix_offset_ns,
                } => {
                    if !self.matches(session_id, *session_generation) {
                        return Err("图导出会话不匹配".into());
                    }
                    if shared.status.lock().unwrap().active {
                        return Err("图导出前必须暂停接纳".into());
                    }
                    if self.artifacts.len() >= 4 {
                        return Err("未释放二维图制品达到上限".into());
                    }
                    self.clear_snapshot()?;
                    let artifact = self.export_graph(*unix_offset_ns)?;
                    self.artifacts.push(artifact.clone());
                    response.artifact = Some(artifact);
                }
                Operation::ReleaseArtifact { artifact } => {
                    artifact.validate()?;
                    if let Some(index) = self.artifacts.iter().position(|a| a == artifact) {
                        std::fs::remove_file(self.directory.join(&artifact.file_name))
                            .map_err(|e| e.to_string())?;
                        self.artifacts.remove(index);
                    } else if self.directory.join(&artifact.file_name).exists() {
                        return Err("二维制品身份不属于导出记录".into());
                    }
                }
                Operation::LoadGraph {
                    session,
                    artifact,
                    localization,
                    seed,
                } => {
                    session.validate()?;
                    artifact.validate()?;
                    if self.slam.is_some()
                        || self.session.is_some()
                        || shared.stopped.load(Ordering::Acquire)
                    {
                        return Err("图加载必须使用空候选进程".into());
                    }
                    if seed.iter().any(|v| !v.is_finite()) {
                        return Err("二维图加载初值非法".into());
                    }
                    let mut file =
                        crate::archive::open_regular(&self.directory.join(&artifact.file_name))?;
                    let (metadata, _) = crate::archive::read_bound_structure(
                        &mut file, artifact.payload_size, &artifact.identity)?;
                    metadata.compatible_with(session)?;
                    let config: Config = serde_json::from_value(session.core_config.clone())
                        .map_err(|e| e.to_string())?;
                    config.validate().map_err(|e| e.to_string())?;
                    if config.resolution != metadata.resolution() {
                        return Err("图加载分辨率与档案不一致".into());
                    }
                    let directory = self.directory.join("loaded");
                    std::fs::create_dir(&directory).map_err(|e| e.to_string())?;
                    let loaded = (|| {
                        let saved = crate::archive::unpack_parsed(&mut file, &directory, &metadata)?;
                        let mut slam = Slam::new(config).map_err(|e| e.to_string())?;
                        slam.load(
                            &directory.join("graph"),
                            *localization,
                            Pose {
                                x: seed[0],
                                y: seed[1],
                                yaw: seed[2],
                            },
                        )
                        .map_err(|e| e.to_string())?;
                        let grid = slam.occupancy_grid().map_err(|e| e.to_string())?;
                        if grid.metadata.width != metadata.dimensions().0
                            || grid.metadata.height != metadata.dimensions().1
                            || grid.metadata.resolution != metadata.resolution()
                            || [grid.metadata.origin_x, grid.metadata.origin_y] != metadata.origin()
                            || grid.cells != saved
                        {
                            return Err("恢复核心的物理栅格与完整图档案不一致".into());
                        }
                        Ok(slam)
                    })();
                    match loaded {
                        Ok(slam) => self.slam = Some(slam),
                        Err(error) => {
                            let _ = std::fs::remove_dir_all(directory);
                            return Err(error);
                        }
                    }
                    self.session = Some(session.clone());
                    self.highest_session = session.session_generation;
                    let mut estimate = metadata.estimate().clone();
                    estimate.graph_revision = 1;
                    self.last_graph_estimate = Some(estimate);
                    self.loaded = Some(metadata);
                    *shared.status.lock().unwrap() = Status {
                        session: Some(session.clone()),
                        paused: true,
                        localization: *localization,
                        graph_revision: 1,
                        ..Default::default()
                    };
                }
                Operation::Shutdown => {
                    shared.session_generation.store(0, Ordering::Release);
                    self.slam.take();
                    self.waiting = None;
                    shared.history.lock().unwrap().clear();
                    let mut status = shared.status.lock().unwrap();
                    status.stopping = true;
                    status.active = false;
                    shared.stopped.store(true, Ordering::Release);
                }
            }
            Ok(())
        })();
        response.error = result.err();
        response.status = shared.status.lock().unwrap().clone();
        if request.operation.mutates() {
            shared.updates.send_replace(response.status.clone());
            self.last_command = request.command_sequence;
            self.commands.push_back((request.clone(), response.clone()));
            while self.commands.len() > 128 {
                self.commands.pop_front();
            }
        }
        response
    }
    fn export_graph(&mut self, unix_offset_ns: i64) -> Result<FileArtifact, String> {
        let estimate = self.last_graph_estimate.clone().ok_or("尚无完整二维图")?;
        let localization = self.loaded.is_some()
            && self.slam.as_ref().is_some()
            && self.loaded.as_ref().is_some_and(|m| {
                estimate.graph_revision == 1 || m.estimate().graph_revision == estimate.graph_revision
            });
        // This one freeze ID is returned and retained in the existing command receipt.
        let checkpoint_id = uuid::Uuid::new_v4().simple().to_string();
        let temporary = self.directory.join(format!("export-{checkpoint_id}"));
        std::fs::create_dir(&temporary).map_err(|e| e.to_string())?;
        let result = (|| {
            let slam = self.slam.as_mut().ok_or("二维核心未加载")?;
            let grid = slam.occupancy_grid().map_err(|e| e.to_string())?;
            let unchanged_loaded = localization && estimate.graph_revision == 1;
            if unchanged_loaded {
                for name in ["graph.posegraph", "graph.data"] {
                    std::fs::copy(
                        self.directory.join("loaded").join(name),
                        temporary.join(name),
                    )
                    .map_err(|e| e.to_string())?;
                }
            } else {
                slam.save(&temporary.join("graph"))
                    .map_err(|e| e.to_string())?;
            }
            let cells: Vec<u8> = grid.cells.iter().map(|v| *v as u8).collect();
            std::fs::write(temporary.join("grid.i8"), &cells).map_err(|e| e.to_string())?;
            let mut members = Vec::new();
            for name in ["graph.posegraph", "graph.data", "grid.i8"] {
                let file = crate::archive::open_regular(&temporary.join(name))?;
                members.push(crate::archive::AssignedMember {
                    name: name.into(),
                    size: file.metadata().map_err(|e| e.to_string())?.len(),
                });
            }
            let source_unix_timestamp_ns = if unchanged_loaded {
                self.loaded.as_ref().unwrap().source_unix_timestamp_ns()
            } else {
                u64::try_from(estimate.source_timestamp_ns as i128 + unix_offset_ns as i128)
                    .map_err(|_| "图来源时间越界")?
            };
            let metadata = crate::archive::AssignedMetadata {
                checkpoint_id: checkpoint_id.clone(),
                source: crate::archive::Metadata {
                version: 2,
                session: self.session.clone().unwrap(),
                estimate,
                source_unix_timestamp_ns,
                width: grid.metadata.width,
                height: grid.metadata.height,
                resolution: grid.metadata.resolution,
                origin: [grid.metadata.origin_x, grid.metadata.origin_y],
                members: members.try_into().unwrap(),
                },
            };
            let filename = format!("graph-{checkpoint_id}.nws2d");
            let assigned = crate::archive::write_assigned(
                &self.directory.join(&filename),
                &metadata,
                [
                    &temporary.join("graph.posegraph"),
                    &temporary.join("graph.data"),
                    &temporary.join("grid.i8"),
                ],
            )?;
            Ok(FileArtifact {
                file_name: filename,
                payload_size: assigned.payload_size,
                identity: ArtifactIdentity::Assigned { checkpoint_id: assigned.checkpoint_id },
            })
        })();
        let cleanup = std::fs::remove_dir_all(temporary).map_err(|e| e.to_string());
        match (result, cleanup) {
            (Ok(a), Ok(())) => Ok(a),
            (Err(e), _) => Err(e),
            (Ok(a), Err(e)) => {
                let _ = std::fs::remove_file(self.directory.join(a.file_name));
                Err(e)
            }
        }
    }
    fn scan(&mut self, packet: ScanPacket, shared: &Shared) {
        let Some(session) = self.session.as_ref() else {
            return;
        };
        if packet.generation != session.session_generation
            || shared.session_generation.load(Ordering::Acquire) != packet.generation
            || self.slam.is_none()
        {
            return;
        }
        let scan = &packet.scan;
        let process = (|| -> Result<Option<Estimate>, String> {
            if scan.sensor_id != session.sensor_id
                || scan.frame_id != session.frame_id
                || scan.calibration_ref != session.calibration_ref
            {
                return Err("二维扫描来源或标定与会话不符".into());
            }
            if scan.clock_domain != session.clock_domain || scan.clock_id != session.clock_id {
                return Err("二维扫描时钟与已绑定来源不符".into());
            }
            if scan.timestamp_quality == 3 {
                return Err("接收时间不能冒充二维扫描采样时间".into());
            }
            let history=shared.history.lock().unwrap();
            let odometry = match session.odometry_source {
                OdometrySource::Laser => None,
                OdometrySource::Device => match history.at(
                    scan.timestamp_ns as u64,
                    &scan.clock_domain,
                    &scan.clock_id,
                )? {
                    Some(p) => Some(Pose {
                        x: p[0],
                        y: p[1],
                        yaw: p[2],
                    }),
                    None => return Ok(None),
                },
            };
            let raw_odometry = odometry.map(|p| [p.x, p.y, p.yaw]);
            let candidate_anchor =
                raw_odometry.map(|p| self.device_anchor.unwrap_or_else(|| inverse(p)));
            let odometry = raw_odometry.zip(candidate_anchor).map(|(p, a)| {
                let p = compose(a, p);
                Pose {
                    x: p[0],
                    y: p[1],
                    yaw: p[2],
                }
            });
            let slam = self.slam.as_mut().unwrap();
            let c = slam.config();
            let prepared = if session.odometry_source == OdometrySource::Device {
                match deskew_scan(scan, &history, c.base_from_laser)? {
                    Some(scan) => scan,
                    None => return Ok(None),
                }
            } else {
                std::borrow::Cow::Borrowed(scan)
            };
            drop(history);
            shared.status.lock().unwrap().motion_compensation = if scan.motion_compensated {
                "upstream"
            } else if prepared.motion_compensated {
                "device_odometry"
            } else {
                "rigid_uncompensated"
            }
            .into();
            let processing_started = Instant::now();
            let (result, processed_beams, mode) = if c.scan_input_mode == ScanInputMode::Raw {
                let (ranges, angles) = raw_scan(&prepared)?;
                (slam.process_raw(scan.timestamp_ns as u64, scan.source_sequence as u64,
                    &ranges, &angles, odometry), ranges.len(), "raw")
            } else {
                let ranges = normalize(&prepared, c.angle_min, c.angle_increment, c.scan_count as usize)?;
                (slam.process(scan.timestamp_ns as u64, scan.source_sequence as u64,
                    &ranges, odometry), ranges.len(), "uniform")
            };
            let result = result.map_err(|e| e.to_string())?;
            {
                let mut status = shared.status.lock().unwrap();
                status.scan_input_mode = mode.into();
                status.input_beams = scan.beam_count as u32;
                status.processed_beams = processed_beams as u32;
                status.valid_beams = result.output.valid_beams;
                status.processing_ms = processing_started.elapsed().as_secs_f64() * 1000.0;
            }
            let o = result.output;
            let pose = |p: Pose| [p.x, p.y, p.yaw];
            let estimate = Estimate {
                source_timestamp_ns: o.timestamp_ns,
                clock_domain: scan.clock_domain.clone(),
                clock_id: scan.clock_id.clone(),
                source_sequence: o.sequence,
                graph_revision: o.graph_revision,
                optimizations: o.optimizations,
                anchor_odom_from_base: candidate_anchor.map(inverse).unwrap_or([0.0; 3]),
                odom_from_base: raw_odometry.unwrap_or_else(|| pose(o.odom_from_base)),
                map_from_base: pose(o.map_from_base),
                map_from_odom: raw_odometry
                    .map(|p| compose(pose(o.map_from_base), inverse(p)))
                    .unwrap_or_else(|| pose(o.map_from_odom)),
                covariance: o.covariance,
                matching_response: o.matching_response,
                local_response: o.local_response,
                disposition: match result.disposition {
                    Disposition::Initialized => "initialized",
                    Disposition::Accepted => "accepted",
                    Disposition::MotionSkipped => "motion_skipped",
                    Disposition::MatchRejected => "match_rejected",
                }
                .into(),
                node_count: o.node_count,
                edge_count: o.edge_count,
            };
            if result.accepted() {
                if self.device_anchor.is_none() {
                    self.device_anchor = candidate_anchor;
                }
                if !shared.status.lock().unwrap().localization {
                    self.last_graph_estimate = Some(estimate.clone());
                }
            }
            Ok(Some(estimate))
        })();
        match process {
            Ok(Some(estimate)) => {
                let mut status = shared.status.lock().unwrap();
                status.last_error = None;
                status.graph_revision = estimate.graph_revision;
                status.last_estimate = Some(estimate);
            }
            Ok(None) => {
                self.waiting = Some((packet, Instant::now()));
            }
            Err(error) => {
                let mut status = shared.status.lock().unwrap();
                status.rejected_scans += 1;
                status.last_error = Some(error);
            }
        }
        let state = shared.status.lock().unwrap().clone();
        shared.updates.send_replace(state);
    }
}
struct WorkerExitGuard {
    shared: Arc<Shared>,
    normal: bool,
}
impl Drop for WorkerExitGuard {
    fn drop(&mut self) {
        let mut inbox = self.shared.inbox.lock().unwrap_or_else(|e| e.into_inner());
        inbox.closed = true;
        inbox.scan = None;
        inbox.commands.clear();
        let mut status = self.shared.status.lock().unwrap_or_else(|e| e.into_inner());
        status.active = false;
        if !self.normal {
            status.last_error = Some("二维算法线程异常退出".into());
        }
        self.shared.session_generation.store(0, Ordering::Release);
        self.shared.stopped.store(true, Ordering::Release);
        self.shared.exit.send_replace(true);
        self.shared.wake.notify_all();
    }
}
fn run(shared: Arc<Shared>, directory: PathBuf) {
    let mut exit_guard = WorkerExitGuard {
        shared: shared.clone(),
        normal: false,
    };
    let mut algorithm = Algorithm {
        slam: None,
        session: None,
        device_anchor: None,
        waiting: None,
        last_graph_estimate: None,
        commands: VecDeque::new(),
        last_command: 0,
        highest_session: 0,
        snapshot_counter: 0,
        snapshot: None,
        pending_grid: None,
        directory,
        artifacts: Vec::new(),
        loaded: None,
    };
    loop {
        let (command, scan, odometry_changed) = {
            let mut inbox = shared.inbox.lock().unwrap();
            while !inbox.closed && inbox.commands.is_empty() && inbox.scan.is_none() && !inbox.odometry_changed {
                if let Some((_, started)) = &algorithm.waiting {
                    let remaining =
                        std::time::Duration::from_millis(250).saturating_sub(started.elapsed());
                    let (next, timed) = shared.wake.wait_timeout(inbox, remaining).unwrap();
                    inbox = next;
                    if timed.timed_out() {
                        break;
                    }
                } else {
                    inbox = shared.wake.wait(inbox).unwrap();
                }
            }
            if inbox.closed {
                break;
            }
            let command = inbox.commands.pop_front();
            let scan = if command.is_none() {
                inbox.scan.take()
            } else {
                None
            };
            let odometry_changed=std::mem::take(&mut inbox.odometry_changed);
            (command, scan, odometry_changed)
        };
        if algorithm
            .waiting
            .as_ref()
            .is_some_and(|(_, started)| started.elapsed().as_millis() >= 250)
        {
            algorithm.waiting = None;
            let mut status = shared.status.lock().unwrap();
            status.rejected_scans += 1;
            status.last_error = Some("扫描等待里程计超时".into());
        }
        let mut retry_waiting=odometry_changed && scan.is_none();
        if let Some(command) = command {
            retry_waiting|=matches!(command.request.operation,Operation::Odometry{..});
            let response=algorithm.control(&command.request,&shared);
            let _=command.reply.send(response);
        }
        if retry_waiting {
            if let Some((packet,started))=algorithm.waiting.take() {
                if started.elapsed().as_millis()<=250 {
                    algorithm.scan(packet,&shared);
                    if let Some((_,retry_started))=algorithm.waiting.as_mut(){*retry_started=started;}
                } else {
                    let mut status=shared.status.lock().unwrap();status.rejected_scans+=1;
                    status.last_error=Some("扫描等待里程计超时".into());
                }
            }
        }
        if let Some(scan) = scan {
            if algorithm.waiting.take().is_some() {
                shared.status.lock().unwrap().dropped_scans += 1;
            }
            algorithm.scan(scan, &shared);
        }
    }
    algorithm.slam.take();
    let _ = algorithm.clear_snapshot();
    exit_guard.normal = true;
}

#[cfg(test)]
mod odometry_ingress_tests {
    use super::*;
    // 不运行算法命令消费线程，模拟原生匹配/构图正在占用计算线程。
    // 接纳必须真正写入同一历史，不能只把请求排队并返回假完成。
    #[tokio::test]
    async fn odometry_receipt_does_not_wait_for_algorithm_and_keeps_source_fences() {
        let identity=Identity{runtime_generation:7,owner_nonce:"a".repeat(32)};
        let session=Session{session_id:"current".into(),map_name:"测试地图".into(),session_generation:3,
            sensor_id:"laser".into(),frame_id:"laser".into(),map_frame_id:"map".into(),odom_frame_id:"odom".into(),body_frame_id:"body".into(),
            clock_domain:"unix_ns".into(),clock_id:String::new(),calibration_ref:"校验".into(),odometry_source:OdometrySource::Device,core_config:serde_json::json!({})};
        let shared=Arc::new(Shared{identity:identity.clone(),status:Mutex::new(Status{active:true,session:Some(session),..Default::default()}),
            history:Mutex::new(PoseHistory::default()),inbox:Mutex::new(Inbox::default()),wake:Condvar::new(),
            session_generation:AtomicU64::new(3),stopped:AtomicBool::new(false),exit:watch::channel(false).0,updates:watch::channel(Status::default()).0});
        let handle=WorkerHandle{shared:shared.clone()};
        let request=|sample:TimedPose|RpcRequest{name:identity.rpc_name(),id:Some(1.0),args:(vec![serde_json::to_value(Request{version:VERSION,identity:identity.clone(),command_sequence:1,operation:Operation::Odometry{sample}}).unwrap()],Default::default())};
        let sample=TimedPose{timestamp_ns:1_000_000_000,clock_domain:"unix_ns".into(),clock_id:String::new(),odom_frame_id:"odom".into(),body_frame_id:"body".into(),pose:[1.0,2.0,0.1]};
        let reply=tokio::time::timeout(std::time::Duration::from_millis(100),handle.handle(request(sample.clone()))).await.expect("原生线程没有消费请求也必须完成接纳");
        let reply:Response=serde_json::from_value(reply.res.unwrap()).unwrap();assert_eq!(reply.error,None);
        assert_eq!(shared.history.lock().unwrap().at(sample.timestamp_ns,"unix_ns","").unwrap(),Some(sample.pose));
        assert!(shared.inbox.lock().unwrap().commands.is_empty());assert!(shared.inbox.lock().unwrap().odometry_changed);
        for fault in 0..4 {
            let mut bad=sample.clone();bad.timestamp_ns+=1;
            match fault {
                0=>bad.timestamp_ns-=2,
                1=>bad.clock_id="另一时钟".into(),
                2=>{shared.session_generation.store(0,Ordering::Release);},
                _=>{shared.session_generation.store(3,Ordering::Release);shared.inbox.lock().unwrap().closed=true;},
            }
            let reply:Response=serde_json::from_value(handle.handle(request(bad)).await.res.unwrap()).unwrap();
            assert!(reply.error.is_some(),"故障{fault}");
        }
        assert_eq!(shared.history.lock().unwrap().at(sample.timestamp_ns,"unix_ns","").unwrap(),Some(sample.pose));
    }
}
