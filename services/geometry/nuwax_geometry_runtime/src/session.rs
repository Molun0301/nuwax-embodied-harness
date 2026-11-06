//! 唯一会话状态拥有方。控制命令顺序执行，传感器帧直接进入当前地图对象。
use crate::{
    corrector,
    explorer::{Explorer, ExplorerTick},
    lio::Lio,
    map_store::MapStore,
    mapper::{pose_from_isometry, pose_isometry, Mapper, MapperConfig, MappingFrame},
    nav_bridge::NavBridge,
    proto::*,
};
use std::{
    collections::{HashSet, VecDeque},
    path::PathBuf,
    sync::Arc,
    time::{Duration, Instant},
};
use tokio::sync::{mpsc, oneshot, watch};
use tonic::Status;

#[path = "explorer_worker.rs"]
mod explorer_worker;
use explorer_worker::ExplorerWorker;

fn trim_recent(recent: &mut VecDeque<(Instant, MappingFrame)>) {
    while recent.len() > 8
        || recent.front().is_some_and(|(time, _)| time.elapsed() > Duration::from_secs(2))
    {
        recent.pop_front();
    }
}

#[derive(Clone)]
pub(crate) struct Route {
    pub(crate) id: u64,
    pub(crate) mapper: Arc<Mapper>,
    map_pose: bool,
    accept_frames: bool,
}

/// 此处只有当前对象与输出投影；会话阶段只在 Session 中改变。
#[derive(Clone)]
pub struct FrameRouter {
    route: watch::Receiver<Option<Route>>,
    pose: watch::Sender<Option<Pose>>,
    odom: watch::Sender<Option<Pose>>,
    faults: mpsc::UnboundedSender<(u64, SessionIssue)>,
    // 近期主雷达输入归持续输入拥有方，不随地图会话替换。
    recent: Arc<parking_lot::Mutex<VecDeque<(Instant, MappingFrame)>>>,
}
impl FrameRouter {
    pub(crate) fn mapper(&self) -> Option<Arc<Mapper>> {
        self.route
            .borrow()
            .as_ref()
            .map(|route| route.mapper.clone())
    }
    pub fn session_id(&self) -> u64 {
        self.route.borrow().as_ref().map_or(0, |route| route.id)
    }
    pub fn publish_pose(&self, session_id: u64, pose: Pose) {
        let route = self.route.borrow().clone();
        if session_id != route.as_ref().map_or(0, |route| route.id) {
            return;
        }
        self.odom.send_replace(Some(pose.clone()));
        let pose = route
            .filter(|route| route.map_pose)
            .map_or_else(|| pose.clone(), |route| route.mapper.transform_pose(&pose));
        self.pose.send_replace(Some(pose));
    }
    fn refresh_transform(&self) {
        let odom = self.odom.borrow().clone();
        let route = self.route.borrow().clone();
        if let Some(odom) = odom {
            let projected = route
                .filter(|route| route.map_pose)
                .map_or_else(|| odom.clone(), |route| route.mapper.transform_pose(&odom));
            self.pose.send_if_modified(|current| {
                if current
                    .as_ref()
                    .is_some_and(|pose| pose.timestamp_ns > projected.timestamp_ns)
                {
                    return false;
                }
                let changed = current.as_ref() != Some(&projected);
                if changed {
                    *current = Some(projected);
                }
                changed
            });
        }
    }
    pub fn publish_frame(&self, frame: MappingFrame) {
        if !frame.mapping_only {
            let mut recent = self.recent.lock();
            recent.push_back((Instant::now(), frame.clone()));
            trim_recent(&mut recent);
        }
        let route = self.route.borrow().clone();
        if let Some(route) =
            route.filter(|route| route.id == frame.session_id && route.accept_frames)
        {
            route.mapper.feed(frame);
        }
    }
    pub(crate) fn recent_frames(&self) -> Vec<MappingFrame> {
        let mut recent = self.recent.lock();
        trim_recent(&mut recent);
        recent.iter().map(|(_, frame)| frame.clone()).collect()
    }
    pub(crate) fn clear_recent_frames(&self) {
        self.recent.lock().clear();
    }
    pub fn publish_planar(&self, session_id: u64, grid: OccupancyGrid) {
        if let Some(route) = self
            .route
            .borrow()
            .as_ref()
            .filter(|route| route.id == session_id && route.accept_frames)
        {
            route.mapper.feed_grid(session_id, grid);
        }
    }
    pub async fn publish_map_from_odom(&self, session_id: u64, pose: Pose) {
        let route = self.route.borrow().clone();
        if let Some(route) = route.filter(|route| route.id == session_id) {
            if let Err(reason) = route.mapper.set_map_from_odom(pose_isometry(&pose)).await {
                let _ = self.faults.send((session_id, SessionIssue::Fault(reason)));
            } else {
                self.refresh_transform();
            }
        }
    }
    pub fn fault(&self, reason: String) {
        let _ = self.faults.send((self.session_id(), SessionIssue::Fault(reason)));
    }
    pub(crate) fn output_error(&self, session_id: u64, reason: String) {
        tracing::warn!(%reason, "二维导航观测失败");
        let _ = self.faults.send((session_id, SessionIssue::Output(reason)));
    }
}

pub(crate) enum SessionIssue {
    Fault(String),
    Output(String),
}

#[derive(Clone)]
pub struct SessionHandle {
    commands: mpsc::UnboundedSender<Command>,
    state: watch::Receiver<GeometryState>,
    route: watch::Receiver<Option<Route>>,
    pose: watch::Receiver<Option<Pose>>,
    maps: watch::Receiver<Option<MapUpdate>>,
    lio: watch::Receiver<Option<Arc<Lio>>>,
    faults: mpsc::UnboundedSender<(u64, SessionIssue)>,
}
const PLANAR_RESUME_UNAVAILABLE: &str = "无完整二维 SLAM 图，不能用于二维续建";
const PLANAR_VIEW_ONLY: &str = "缺少二维定位图，只能查看";
type Reply<T> = oneshot::Sender<Result<T, Status>>;
enum Command {
    Start(StartMappingRequest, Reply<GeometryState>),
    Stop(Reply<GeometryState>),
    Save(SaveMapRequest, Reply<SaveMapResponse>),
    Load(LoadMapRequest, Reply<GeometryState>),
    Relocalize(RelocalizeRequest, Reply<RelocalizationResult>),
    Goal(NavigationGoal, Reply<SetGoalResponse>),
    Cancel(CancelGoalRequest, Reply<()>),
    Explore(ExplorationRequest, Reply<StartExplorationResponse>),
    StopExplore(StopExplorationRequest, Reply<()>),
    Progress(ExplorationProgressRequest, Reply<ExplorationProgress>),
    Shutdown(oneshot::Sender<()>),
}
impl SessionHandle {
    pub async fn start() -> Result<Self, String> {
        let config = MapperConfig::from_env()?;
        Self::start_with(config, MapStore::new(catalog_root())).await
    }
    async fn start_with(config: MapperConfig, store: MapStore) -> Result<Self, String> {
        let (commands, command_rx) = mpsc::unbounded_channel();
        let (route_tx, route) = watch::channel(None);
        let (pose_tx, pose) = watch::channel(None);
        let (odom, _) = watch::channel(None);
        let (faults, fault_rx) = mpsc::unbounded_channel();
        let router = FrameRouter {
            route: route.clone(),
            pose: pose_tx,
            odom,
            faults: faults.clone(),
            recent: Default::default(),
        };
        let (maps_tx, maps) = watch::channel(None);
        tokio::spawn(forward_maps(
            route.clone(),
            maps_tx,
            faults.clone(),
            router.clone(),
        ));
        let (nav_tx, nav_rx) = mpsc::unbounded_channel();
        let nav = NavBridge::start(nav_tx).await?;
        let mut initial = GeometryState {
            phase: SessionPhase::Idle as i32,
            info: Some(RuntimeInfo {
                pipeline: "lio_3d".into(),
                map_frame_id: config.map_frame_id.clone(),
                odom_frame_id: config.odom_frame_id.clone(),
                base_frame_id: config.base_frame_id.clone(),
                ..Default::default()
            }),
            ..Default::default()
        };
        let lio = match Lio::start(router.clone()).await {
            Ok(lio) => {
                initial.info = Some(lio.info());
                Some(Arc::new(lio))
            }
            Err(reason) => {
                initial.phase = SessionPhase::Fault as i32;
                initial.reason = reason;
                None
            }
        };
        let (lio_tx, lio_rx) = watch::channel(lio);
        let (state_tx, state) = watch::channel(initial.clone());
        let actor = Session {
            state: initial,
            publisher: state_tx,
            route: route_tx,
            router,
            config,
            store: Arc::new(store),
            save_sources: HashSet::new(),
            next_session: 0,
            lio: lio_tx,
            nav,
            explorer: None,
        };
        tokio::spawn(actor.run(command_rx, fault_rx, nav_rx));
        Ok(Self {
            commands,
            state,
            route,
            pose,
            maps,
            lio: lio_rx,
            faults,
        })
    }
    async fn request<T>(&self, build: impl FnOnce(Reply<T>) -> Command) -> Result<T, Status> {
        let (tx, rx) = oneshot::channel();
        self.commands
            .send(build(tx))
            .map_err(|_| Status::unavailable("会话拥有方已停止"))?;
        rx.await
            .map_err(|_| Status::unavailable("会话拥有方已停止"))?
    }
    pub async fn start_mapping(
        &self,
        request: StartMappingRequest,
    ) -> Result<GeometryState, Status> {
        self.request(|tx| Command::Start(request, tx)).await
    }
    pub async fn stop_session(&self) -> Result<GeometryState, Status> {
        self.request(Command::Stop).await
    }
    pub async fn save_map(&self, request: SaveMapRequest) -> Result<SaveMapResponse, Status> {
        self.request(|tx| Command::Save(request, tx)).await
    }
    pub async fn load_map(&self, request: LoadMapRequest) -> Result<GeometryState, Status> {
        self.request(|tx| Command::Load(request, tx)).await
    }
    pub async fn relocalize(
        &self,
        request: RelocalizeRequest,
    ) -> Result<RelocalizationResult, Status> {
        self.request(|tx| Command::Relocalize(request, tx)).await
    }
    pub async fn set_goal(&self, request: NavigationGoal) -> Result<SetGoalResponse, Status> {
        self.request(|tx| Command::Goal(request, tx)).await
    }
    pub async fn cancel_goal(&self, request: CancelGoalRequest) -> Result<(), Status> {
        self.request(|tx| Command::Cancel(request, tx)).await
    }
    pub async fn start_exploration(
        &self,
        request: ExplorationRequest,
    ) -> Result<StartExplorationResponse, Status> {
        self.request(|tx| Command::Explore(request, tx)).await
    }
    pub async fn stop_exploration(&self, request: StopExplorationRequest) -> Result<(), Status> {
        self.request(|tx| Command::StopExplore(request, tx)).await
    }
    pub async fn exploration_progress(
        &self,
        request: ExplorationProgressRequest,
    ) -> Result<ExplorationProgress, Status> {
        self.request(|tx| Command::Progress(request, tx)).await
    }
    pub(crate) fn route_rx(&self) -> watch::Receiver<Option<Route>> {
        self.route.clone()
    }
    pub(crate) fn report_output_error(&self, id: u64, reason: String) {
        tracing::warn!(%reason, "Geometry 旁路输出失败");
        let _ = self.faults.send((id, SessionIssue::Output(reason)));
    }
    pub fn state(&self) -> GeometryState {
        let mut state = self.state.borrow().clone();
        state.pose = self.pose.borrow().clone();
        state
    }
    pub fn state_rx(&self) -> watch::Receiver<GeometryState> {
        self.state.clone()
    }
    pub fn pose_rx(&self) -> watch::Receiver<Option<Pose>> {
        self.pose.clone()
    }
    pub fn map_rx(&self) -> watch::Receiver<Option<MapUpdate>> {
        self.maps.clone()
    }
    pub(crate) fn lio(&self) -> Result<Arc<Lio>, Status> {
        self.lio
            .borrow()
            .clone()
            .ok_or_else(|| Status::unavailable("输入子系统未运行"))
    }
    pub(crate) fn mapper(&self) -> Result<Arc<Mapper>, Status> {
        self.route
            .borrow()
            .as_ref()
            .map(|route| route.mapper.clone())
            .ok_or_else(|| Status::failed_precondition("当前没有地图"))
    }
    pub fn occupancy(&self, include_forbidden: bool) -> Result<OccupancyGrid, Status> {
        self.mapper()?
            .occupancy(include_forbidden)
            .ok_or_else(|| Status::failed_precondition("当前没有占据栅格"))
    }
    pub async fn dense_chunk(
        &self,
        request: DenseCloudChunkRequest,
    ) -> Result<DenseCloudChunk, Status> {
        self.mapper()?
            .dense_chunk(request)
            .await
            .map_err(Status::failed_precondition)
    }
    pub fn frame_tree(&self) -> FrameTree {
        let info = self.state.borrow().info.clone().unwrap();
        let mut transforms = Vec::new();
        if let Some(route) = self.route.borrow().as_ref().filter(|route| route.map_pose) {
            transforms.push(transform(
                route.mapper.map_from_odom(),
                &info.map_frame_id,
                &info.odom_frame_id,
                self.pose
                    .borrow()
                    .as_ref()
                    .map_or(0, |pose| pose.timestamp_ns),
            ));
        }
        if let Some(pose) = self.pose.borrow().as_ref() {
            // 地图位姿先还原为里程计位姿，保持坐标树边的真实含义。
            let odom_from_base = self
                .route
                .borrow()
                .as_ref()
                .filter(|route| route.map_pose)
                .map_or_else(
                    || pose_isometry(pose),
                    |route| route.mapper.map_from_odom().inverse() * pose_isometry(pose),
                );
            transforms.push(transform(
                odom_from_base,
                &info.odom_frame_id,
                &info.base_frame_id,
                pose.timestamp_ns,
            ));
        }
        if let Ok(lio) = self.lio() {
            transforms.extend(lio.static_transforms());
        }
        FrameTree { transforms }
    }
    pub(crate) async fn closed(&self) {
        self.commands.closed().await;
    }
    pub async fn shutdown(&self) {
        let (tx, rx) = oneshot::channel();
        if self.commands.send(Command::Shutdown(tx)).is_ok() {
            let _ = rx.await;
        }
    }
}

struct Session {
    state: GeometryState,
    publisher: watch::Sender<GeometryState>,
    route: watch::Sender<Option<Route>>,
    router: FrameRouter,
    config: MapperConfig,
    store: Arc<MapStore>,
    save_sources: HashSet<String>,
    next_session: u64,
    lio: watch::Sender<Option<Arc<Lio>>>,
    nav: NavBridge,
    explorer: Option<ExplorerWorker>,
}
impl Session {
    async fn run(
        mut self,
        mut commands: mpsc::UnboundedReceiver<Command>,
        mut faults: mpsc::UnboundedReceiver<(u64, SessionIssue)>,
        mut navigation: mpsc::UnboundedReceiver<NavigationStatus>,
    ) {
        let mut tick = tokio::time::interval(Duration::from_millis(200));
        tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        loop {
            tokio::select! {
                command = commands.recv() => match command {
                    Some(Command::Start(request, reply)) => { let result = self.start_mapping(request).await; self.publish(); let _ = reply.send(result.map(|_| self.snapshot())); },
                    Some(Command::Stop(reply)) => { let result = self.end_current().await; self.publish(); let _ = reply.send(result.map(|_| self.snapshot())); },
                    Some(Command::Save(request, reply)) => { let result = self.save(request).await; self.publish(); let _ = reply.send(result); },
                    Some(Command::Load(request, reply)) => { let result = self.load(request).await; self.publish(); let _ = reply.send(result.map(|_| self.snapshot())); },
                    Some(Command::Relocalize(request, reply)) => { let result = self.relocalize(request).await; self.publish(); let _ = reply.send(result); },
                    Some(Command::Goal(request, reply)) => { let result = self.goal(request).await; self.publish(); let _ = reply.send(result); },
                    Some(Command::Cancel(request, reply)) => { let result = self.cancel(&request.goal_id, GoalStatus::Cancelled).await; self.publish(); let _ = reply.send(result); },
                    Some(Command::Explore(request, reply)) => { let result = self.explore(request).await; self.publish(); let _ = reply.send(result); },
                    Some(Command::StopExplore(request, reply)) => {
                        let result = if request.exploration_id.is_empty() || self.state.exploration.as_ref().is_some_and(|e| e.exploration_id == request.exploration_id) { self.stop_explorer().await } else { Ok(()) };
                        self.publish(); let _ = reply.send(result);
                    },
                    Some(Command::Progress(request, reply)) => self.progress(request, reply),
                    Some(Command::Shutdown(reply)) => { commands.close(); let _ = self.end_current().await; let lio = self.lio.borrow().clone(); if let Some(lio) = lio { lio.shutdown().await; } self.nav.shutdown().await; let _ = reply.send(()); return; },
                    None => break,
                },
                Some((id, issue)) = faults.recv() => {
                    if id != self.router.session_id() { continue; }
                    match issue {
                        SessionIssue::Fault(reason) => self.fail(reason).await,
                        SessionIssue::Output(reason)
                            if self.phase() != SessionPhase::Fault
                                && !self.route.borrow().as_ref().is_some_and(|route| route.mapper.planar_view_only()) =>
                        {
                            self.state.reason = reason;
                        }
                        SessionIssue::Output(_) => {}
                    }
                    self.publish();
                },
                Some(status) = navigation.recv() => { self.navigation(status).await; self.publish(); },
                update = async {
                    match &mut self.explorer {
                        Some(explorer) => explorer.next().await,
                        None => std::future::pending().await,
                    }
                }, if self.state.phase == SessionPhase::Exploring as i32 => {
                    if let Some(update) = update {
                        match update {
                            Ok(update) => self.exploration_update(update).await,
                            Err(reason) => self.fail_exploration(reason).await,
                        }
                        self.publish();
                    }
                },
                _ = tick.tick() => { self.refresh_navigation_input(); self.publish(); },
            }
        }
        let _ = self.end_current().await;
        let lio = self.lio.borrow().clone();
        if let Some(lio) = lio {
            lio.shutdown().await;
        }
        self.nav.shutdown().await;
    }
    fn phase(&self) -> SessionPhase {
        SessionPhase::try_from(self.state.phase).unwrap()
    }
    fn snapshot(&self) -> GeometryState {
        let mut state = self.state.clone();
        state.pose = self.router.pose.borrow().clone();
        state
    }
    fn publish(&self) {
        let state = self.snapshot();
        self.publisher.send_if_modified(|current| {
            // 位姿独立发布；只有业务状态改变才唤醒状态订阅。
            let pose = current.pose.take();
            let mut next = state.clone();
            next.pose = None;
            let changed = *current != next;
            current.pose = pose;
            if changed {
                *current = state;
            }
            changed
        });
    }
    fn mapper(&self) -> Result<Arc<Mapper>, Status> {
        self.route
            .borrow()
            .as_ref()
            .map(|route| route.mapper.clone())
            .ok_or_else(|| Status::failed_precondition("当前没有地图"))
    }
    fn lio(&self) -> Result<Arc<Lio>, Status> {
        self.lio
            .borrow()
            .clone()
            .ok_or_else(|| Status::unavailable("输入子系统未运行"))
    }
    fn install(&mut self, mapper: Arc<Mapper>, phase: SessionPhase) {
        self.state.phase = phase as i32;
        self.route.send_replace(Some(Route {
            id: mapper.session_id(),
            accept_frames: !mapper.planar_view_only(),
            mapper,
            map_pose: matches!(
                phase,
                SessionPhase::Mapping | SessionPhase::Exploring | SessionPhase::Localized
            ),
        }));
        let latest_odom = self.router.odom.borrow().clone();
        if let Some(pose) = latest_odom {
            self.router.publish_pose(self.router.session_id(), pose);
        }
    }
    async fn recover(&mut self) -> Result<(), Status> {
        let existing = self.lio.borrow().clone();
        let result = match existing {
            Some(lio) => lio.recover().await.map(|_| lio),
            None => Lio::start(self.router.clone()).await.map(Arc::new),
        };
        match result {
            Ok(lio) => {
                self.state.info = Some(lio.info());
                self.lio.send_replace(Some(lio));
                Ok(())
            }
            Err(reason) => {
                self.fail(reason.clone()).await;
                Err(Status::unavailable(reason))
            }
        }
    }
    async fn end_current(&mut self) -> Result<(), Status> {
        self.stop_explorer().await?;
        self.cancel("", GoalStatus::Cancelled).await?;
        let old = self.route.send_replace(None);
        self.nav.set_input_ready(false);
        let map_result = if let Some(route) = old {
            route.mapper.shutdown().await.map_err(Status::unavailable)
        } else {
            Ok(())
        };
        let lio = self.lio.borrow().clone();
        let input_result = if let Some(lio) = lio {
            lio.stop_session().await.map_err(Status::unavailable)
        } else {
            Ok(())
        };
        self.save_sources.clear();
        self.explorer = None;
        self.state.phase = SessionPhase::Idle as i32;
        self.state.map_name.clear();
        self.state.reason.clear();
        self.state.exploration = None;
        self.state.relocalization = None;
        let latest_odom = self.router.odom.borrow().clone();
        if let Some(pose) = latest_odom {
            self.router.publish_pose(0, pose);
        }
        map_result.and(input_result)
    }
    async fn start_mapping(&mut self, request: StartMappingRequest) -> Result<(), Status> {
        if matches!(
            self.phase(),
            SessionPhase::Mapping | SessionPhase::Exploring
        ) && self.state.map_name == request.map_name
        {
            return Ok(());
        }
        let continued = if request.continue_saved {
            let exists = self.store.exists(&request.map_name);
            if exists && self.state.info.as_ref().unwrap().pipeline == "slam_2d" {
                if let Some(reason) = self
                    .store
                    .planar_resume_unavailable_reason(&request.map_name)?
                {
                    return Err(Status::failed_precondition(reason));
                }
            }
            if self.config.correction_backend == "library_pgo"
                || self.config.correction_backend == "native_pgo"
            {
                return Err(Status::failed_precondition("PGO 配置不支持续建"));
            }
            if !exists {
                return Err(Status::not_found("地图不存在"));
            }
            if self.phase() != SessionPhase::Localized || self.state.map_name != request.map_name {
                return Err(Status::failed_precondition("续建要求已定位且加载同名地图"));
            }
            let mut snapshot = self
                .mapper()?
                .snapshot()
                .await
                .map_err(Status::unavailable)?;
            snapshot.planar_checkpoint = self
                .lio()?
                .planar_checkpoint()
                .await
                .map_err(Status::unavailable)?;
            Some(snapshot)
        } else {
            None
        };
        if !request.continue_saved && self.store.exists(&request.map_name) {
            return Err(Status::already_exists("地图已存在"));
        }
        self.recover().await?;
        self.end_current().await?;
        self.next_session += 1;
        let mut config = self.config.clone();
        config.map_name = request.map_name.clone();
        let checkpoint = continued
            .as_ref()
            .and_then(|snapshot| snapshot.planar_checkpoint.clone());
        let mapper = match continued {
            Some(snapshot) => Mapper::load(self.next_session, config, snapshot, false),
            None => Mapper::start(self.next_session, config),
        };
        let mapper = self.subsystem(mapper).await?;
        self.state.map_name = request.map_name.clone();
        if request.continue_saved {
            self.save_sources.insert(request.map_name.clone());
        }
        self.install(mapper.clone(), SessionPhase::Mapping);
        let seed = self.router.pose.borrow().clone();
        self.subsystem(
            self.lio()?
                .start_session(
                    self.next_session,
                    &request.map_name,
                    checkpoint,
                    false,
                    seed,
                )
                .await,
        )
        .await?;
        if let Err(reason) = mapper
            .set_forbidden_zones(
                self.store
                    .root()
                    .join(&request.map_name)
                    .join("forbidden_zones.json"),
            )
            .await
        {
            tracing::warn!(%reason, "禁行区等待刷新");
        }
        Ok(())
    }
    async fn load(&mut self, request: LoadMapRequest) -> Result<(), Status> {
        let store = self.store.clone();
        let name = request.map_name.clone();
        let snapshot = tokio::task::spawn_blocking(move || store.load(&name))
            .await
            .map_err(|e| Status::internal(e.to_string()))??;
        let checkpoint = snapshot.planar_checkpoint.clone();
        let seed = snapshot.last_pose.clone();
        let id = self.next_session + 1;
        let mut config = self.config.clone();
        config.map_name = request.map_name.clone();
        let mapper = Mapper::prepare_load(id, config, snapshot, true)?;
        let view_only = mapper.planar_view_only();
        if !view_only {
            self.recover().await?;
        }
        self.end_current().await?;
        self.next_session = id;
        self.state.map_name = request.map_name.clone();
        self.save_sources.insert(request.map_name.clone());
        self.install(mapper.clone(), SessionPhase::Loaded);
        self.subsystem(mapper.activate_shared_cloud().await).await?;
        if view_only {
            self.state.reason = PLANAR_VIEW_ONLY.into();
        } else {
            self.subsystem(
                self.lio()?
                    .start_session(self.next_session, &request.map_name, checkpoint, true, seed)
                    .await,
            )
            .await?;
        }
        if let Err(reason) = mapper
            .set_forbidden_zones(
                self.store
                    .root()
                    .join(&request.map_name)
                    .join("forbidden_zones.json"),
            )
            .await
        {
            tracing::warn!(%reason, "禁行区等待刷新");
        }
        Ok(())
    }
    async fn save(&mut self, request: SaveMapRequest) -> Result<SaveMapResponse, Status> {
        if !matches!(
            self.phase(),
            SessionPhase::Mapping | SessionPhase::Exploring | SessionPhase::Fault
        ) {
            return Err(Status::failed_precondition("当前没有正在建的地图"));
        }
        let name = if request.map_name.is_empty() {
            self.state.map_name.clone()
        } else {
            request.map_name
        };
        let mapper = self.mapper()?;
        let mut snapshot = self.subsystem(mapper.snapshot().await).await?;
        if self.phase() == SessionPhase::Fault
            && self.state.info.as_ref().unwrap().pipeline == "slam_2d"
        {
            snapshot.planar_checkpoint = None;
            snapshot.planar_resume_unavailable_reason = Some(PLANAR_RESUME_UNAVAILABLE.into());
        } else {
            let lio = self.lio.borrow().clone();
            if let Some(lio) = lio {
                snapshot.planar_checkpoint = self.subsystem(lio.planar_checkpoint().await).await?;
            }
        }
        let store = self.store.clone();
        let source = self.save_sources.contains(&name).then(|| name.clone());
        let saved_name = name.clone();
        let replaced = tokio::task::spawn_blocking(move || {
            store.save(&saved_name, source.as_deref(), snapshot)
        })
        .await
        .map_err(|e| Status::internal(e.to_string()))??;
        mapper.set_map_name(name.clone());
        if let Err(reason) = mapper
            .set_forbidden_zones(self.store.root().join(&name).join("forbidden_zones.json"))
            .await
        {
            self.navigation_input_error(reason);
        }
        if let Some(explorer) = &mut self.explorer {
            explorer.set_map_name(name.clone());
        }
        self.state.map_name = name.clone();
        self.save_sources.insert(name.clone());
        Ok(SaveMapResponse {
            map_name: name,
            replaced,
        })
    }
    async fn relocalize(
        &mut self,
        request: RelocalizeRequest,
    ) -> Result<RelocalizationResult, Status> {
        if !matches!(self.phase(), SessionPhase::Loaded | SessionPhase::Localized) {
            return Err(Status::failed_precondition("重定位要求已加载地图"));
        }
        let mapper = self.mapper()?;
        if mapper.planar_view_only() {
            return Err(Status::failed_precondition(PLANAR_VIEW_ONLY));
        }
        if matches!(
            self.config.correction_backend.as_str(),
            "library_pgo" | "native_pgo"
        ) {
            return Err(Status::failed_precondition("PGO 配置下不能重定位"));
        }
        let planar = self.state.info.as_ref().unwrap().pipeline == "slam_2d";
        let result = if planar {
            let seed = request.seed.map(|seed| {
                pose_from_isometry(
                    nalgebra::Isometry3::new(
                        nalgebra::Vector3::new(seed.x, seed.y, 0.0),
                        nalgebra::Vector3::new(0.0, 0.0, seed.yaw),
                    ),
                    &self.config.map_frame_id,
                    0,
                )
            });
            let mut result = self
                .subsystem(self.lio()?.relocalize_planar(seed).await)
                .await?;
            if result.succeeded {
                mapper
                    .set_map_from_odom(pose_isometry(result.pose.as_ref().unwrap()))
                    .await
                    .map_err(Status::unavailable)?;
                result.pose = self
                    .router
                    .odom
                    .borrow()
                    .as_ref()
                    .map(|pose| mapper.transform_pose(pose));
            }
            result.finished_at_ns = now_ns();
            result
        } else {
            corrector::Corrector::relocalize(&mapper, self.router.recent_frames()).await?
        };
        self.state.reason = if result.succeeded {
            String::new()
        } else {
            result.message.clone()
        };
        self.state.relocalization = Some(result.clone());
        self.install(
            mapper,
            if result.succeeded {
                SessionPhase::Localized
            } else {
                SessionPhase::Loaded
            },
        );
        Ok(result)
    }
    async fn goal(&mut self, goal: NavigationGoal) -> Result<SetGoalResponse, Status> {
        if !matches!(
            self.phase(),
            SessionPhase::Mapping | SessionPhase::Exploring | SessionPhase::Localized
        ) {
            return Err(Status::failed_precondition("未定位"));
        }
        self.stop_explorer().await?;
        self.replace_navigation().await?;
        let mapper = self.mapper()?;
        self.nav.set_input_ready(false);
        let id = uuid::Uuid::new_v4().to_string();
        self.state.navigation = Some(NavigationStatus {
            goal_id: id.clone(),
            status: GoalStatus::WaitingInput as i32,
            target: goal.pose.clone(),
            ..Default::default()
        });
        // 当前地图的禁区缓存由地图任务按秒刷新，目标采用它发布的读取结果。
        let refreshed = mapper
            .set_forbidden_zones(
                self.store
                    .root()
                    .join(&self.state.map_name)
                    .join("forbidden_zones.json"),
            )
            .await;
        if let Err(reason) = &refreshed {
            self.state.navigation.as_mut().unwrap().message = reason.clone();
        }
        if let Err(error) = self.nav.submit(id.clone(), goal, false).await {
            self.navigation_failed(error.message().into());
        }
        self.nav.set_input_ready(
            refreshed.is_ok()
                && mapper.has_occupancy()
                && self.router.pose.borrow().is_some(),
        );
        Ok(SetGoalResponse { goal_id: id })
    }
    async fn replace_navigation(&mut self) -> Result<(), Status> {
        if let Some(old) = self.state.navigation.as_ref() {
            if !terminal(old.status) {
                let id = old.goal_id.clone();
                self.cancel(&id, GoalStatus::Replaced).await?;
                self.state.navigation.as_mut().unwrap().message = "被新目标替换".into();
            }
            self.state.previous_navigation = self.state.navigation.clone();
        }
        Ok(())
    }
    async fn cancel(&mut self, id: &str, status: GoalStatus) -> Result<(), Status> {
        if let Some(current) =
            self.state.navigation.as_mut().filter(|current| {
                (id.is_empty() || current.goal_id == id) && !terminal(current.status)
            })
        {
            if let Err(error) = self.nav.cancel(&current.goal_id).await {
                current.status = GoalStatus::Failed as i32;
                current.message = error.message().into();
                current.planned_path.clear();
                return Err(error);
            }
            current.status = status as i32;
            current.message = "导航已取消".into();
            current.planned_path.clear();
        }
        Ok(())
    }
    async fn explore(
        &mut self,
        request: ExplorationRequest,
    ) -> Result<StartExplorationResponse, Status> {
        if !matches!(
            self.phase(),
            SessionPhase::Mapping | SessionPhase::Exploring
        ) {
            return Err(Status::failed_precondition("探索要求正在建图"));
        }
        self.stop_explorer().await?;
        self.cancel("", GoalStatus::Cancelled).await?;
        let mut explorer = Explorer::new(
            self.state.map_name.clone(),
            self.config.map_frame_id.clone(),
            self.state.info.as_ref().unwrap().pipeline == "lio_3d",
        )?;
        let id = uuid::Uuid::new_v4().to_string();
        self.state.exploration = Some(explorer.start(id.clone(), request)?);
        self.explorer = Some(ExplorerWorker::start(
            explorer,
            self.mapper()?,
            self.router.pose.subscribe(),
        ));
        self.state.phase = SessionPhase::Exploring as i32;
        Ok(StartExplorationResponse { exploration_id: id })
    }
    async fn stop_explorer(&mut self) -> Result<(), Status> {
        if self.phase() == SessionPhase::Exploring {
            if let Some(explorer) = &mut self.explorer {
                explorer.stop();
            }
            if let Some(status) = &mut self.state.exploration {
                status.phase = ExplorationPhase::Stopped as i32;
                status.message = "探索已停止".into();
            }
            if self
                .state
                .navigation
                .as_ref()
                .is_some_and(|goal| goal.exploration_child)
            {
                let result = self.cancel("", GoalStatus::Cancelled).await;
                self.state.phase = SessionPhase::Mapping as i32;
                return result;
            }
            self.state.phase = SessionPhase::Mapping as i32;
        }
        Ok(())
    }
    fn progress(&self, request: ExplorationProgressRequest, reply: Reply<ExplorationProgress>) {
        if !matches!(
            self.phase(),
            SessionPhase::Mapping | SessionPhase::Exploring
        ) || self.explorer.is_none()
            || (!request.exploration_id.is_empty()
                && self
                    .state
                    .exploration
                    .as_ref()
                    .is_none_or(|e| e.exploration_id != request.exploration_id))
        {
            let _ = reply.send(Err(Status::failed_precondition("本会话没有该探索进度")));
            return;
        }
        self.explorer.as_ref().unwrap().progress(reply);
    }
    async fn navigation(&mut self, mut status: NavigationStatus) {
        let Some(current) = self
            .state
            .navigation
            .as_ref()
            .filter(|current| current.goal_id == status.goal_id && !terminal(current.status))
        else {
            return;
        };
        status.target = current.target.clone();
        status.exploration_child = current.exploration_child;
        if self.phase() == SessionPhase::Exploring {
            if let Some(explorer) = &self.explorer {
                explorer.feedback(status.clone());
            }
        }
        self.state.navigation = Some(status);
    }
    fn navigation_input_error(&mut self, reason: String) {
        if let Some(goal) = self
            .state
            .navigation
            .as_mut()
            .filter(|goal| !terminal(goal.status))
        {
            goal.message = reason;
        }
    }
    fn refresh_navigation_input(&mut self) {
        if self.phase() == SessionPhase::Fault {
            return;
        }
        let Some(route) = self.route.borrow().clone() else {
            return;
        };
        let refreshed = route.mapper.forbidden_status();
        self.nav.set_input_ready(
            route.accept_frames
                && refreshed.is_ok()
                && route.mapper.has_occupancy()
                && self.router.pose.borrow().is_some(),
        );
        if let Err(reason) = refreshed {
            self.navigation_input_error(reason);
        }
    }
    async fn exploration_update(&mut self, update: ExplorerTick) {
        if let Some(id) = update.cancel_goal {
            let _ = self.cancel(&id, GoalStatus::Cancelled).await;
        }
        if let Some((id, goal)) = update.goal {
            if let Err(error) = self.replace_navigation().await {
                self.fail_exploration(error.message().into()).await;
                return;
            }
            self.state.navigation = Some(NavigationStatus {
                goal_id: id.clone(),
                status: GoalStatus::WaitingInput as i32,
                target: goal.pose.clone(),
                exploration_child: true,
                ..Default::default()
            });
            if let Err(error) = self
                .nav
                .submit_with_boundary(id, goal, true, update.boundary)
                .await
            {
                self.navigation_failed(error.message().into());
                if let (Some(explorer), Some(status)) = (&self.explorer, &self.state.navigation) {
                    explorer.feedback(status.clone());
                }
                return;
            }
        }
        let finished = update.status.phase != ExplorationPhase::Running as i32;
        self.state.exploration = Some(update.status);
        if finished {
            self.state.phase = SessionPhase::Mapping as i32;
            if self
                .state
                .navigation
                .as_ref()
                .is_some_and(|goal| goal.exploration_child)
            {
                let _ = self.cancel("", GoalStatus::Cancelled).await;
            }
        }
    }
    fn navigation_failed(&mut self, reason: String) {
        if let Some(goal) = &mut self.state.navigation {
            goal.status = GoalStatus::Failed as i32;
            goal.message = reason;
            goal.planned_path.clear();
        }
    }
    async fn fail_exploration(&mut self, reason: String) {
        if let Some(explorer) = &self.explorer {
            explorer.fail(reason.clone());
        }
        if let Some(status) = &mut self.state.exploration {
            status.phase = ExplorationPhase::Failed as i32;
            status.message = reason;
        }
        self.state.phase = SessionPhase::Mapping as i32;
        if self
            .state
            .navigation
            .as_ref()
            .is_some_and(|goal| goal.exploration_child)
        {
            let _ = self.cancel("", GoalStatus::Cancelled).await;
        }
    }
    async fn subsystem<T>(&mut self, result: Result<T, String>) -> Result<T, Status> {
        match result {
            Ok(value) => Ok(value),
            Err(reason) => {
                self.fail(reason.clone()).await;
                Err(Status::unavailable(reason))
            }
        }
    }
    async fn fail(&mut self, reason: String) {
        tracing::error!(%reason, "Geometry 会话进入故障");
        let _ = self.stop_explorer().await;
        let _ = self.cancel("", GoalStatus::Cancelled).await;
        self.route.send_if_modified(|route| {
            if let Some(route) = route.as_mut().filter(|route| route.accept_frames) {
                route.accept_frames = false;
                return true;
            }
            false
        });
        self.nav.set_input_ready(false);
        self.state.phase = SessionPhase::Fault as i32;
        self.state.reason = reason;
    }
}
fn terminal(status: i32) -> bool {
    matches!(
        GoalStatus::try_from(status),
        Ok(GoalStatus::Succeeded
            | GoalStatus::Failed
            | GoalStatus::Cancelled
            | GoalStatus::Replaced
            | GoalStatus::Stopped)
    )
}
fn catalog_root() -> PathBuf {
    std::env::var_os("NUWAX_MAP_CATALOG_ROOT")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("runtime_data/maps"))
}
fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64
}
fn transform(
    value: nalgebra::Isometry3<f64>,
    parent: &str,
    child: &str,
    timestamp_ns: u64,
) -> Transform {
    let pose = pose_from_isometry(value, parent, timestamp_ns);
    Transform {
        parent_frame: parent.into(),
        child_frame: child.into(),
        translation: pose.position,
        rotation: pose.orientation,
        timestamp_ns,
    }
}
async fn forward_maps(
    mut routes: watch::Receiver<Option<Route>>,
    maps: watch::Sender<Option<MapUpdate>>,
    faults: mpsc::UnboundedSender<(u64, SessionIssue)>,
    router: FrameRouter,
) {
    loop {
        let route = routes.borrow_and_update().clone();
        maps.send_replace(None);
        if let Some(route) = route {
            let mut updates = route.mapper.map_rx();
            let mut errors = route.mapper.error_rx();
            let mut poses = route.mapper.pose_rx();
            maps.send_replace(updates.borrow_and_update().clone());
            loop {
                tokio::select! {
                    changed = routes.changed() => { if changed.is_err() { return; } break; },
                    changed = updates.changed() => {
                        if changed.is_err() { break; }
                        if routes.borrow().as_ref().is_some_and(|current| current.id == route.id) { maps.send_replace(updates.borrow_and_update().clone()); }
                    },
                    changed = poses.changed() => {
                        if changed.is_err() { break; }
                        if routes.borrow().as_ref().is_some_and(|current| current.id == route.id) { router.refresh_transform(); }
                    },
                    changed = errors.changed() => {
                        if changed.is_err() { break; }
                        if let Some(reason) = errors.borrow_and_update().clone() { let _ = faults.send((route.id, SessionIssue::Fault(reason))); }
                    },
                }
            }
        } else if routes.changed().await.is_err() {
            return;
        }
    }
}

#[cfg(test)]
pub(crate) fn test_router(
    mapper: Arc<Mapper>,
) -> (
    FrameRouter,
    watch::Receiver<Option<Pose>>,
    mpsc::UnboundedReceiver<(u64, SessionIssue)>,
) {
    let (_, route) = watch::channel(Some(Route {
        id: mapper.session_id(),
        mapper,
        map_pose: true,
        accept_frames: true,
    }));
    let (pose, pose_rx) = watch::channel(None);
    let (odom, _) = watch::channel(None);
    let (faults, fault_rx) = mpsc::unbounded_channel();
    (
        FrameRouter {
            route,
            pose,
            odom,
            faults,
            recent: Default::default(),
        },
        pose_rx,
        fault_rx,
    )
}

#[cfg(test)]
#[path = "session_tests.rs"]
mod tests;
