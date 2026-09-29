//! 三维导航生产拥有方：共享地图，交付PCT全局参考，并由Scan近场持续执行。

use std::collections::VecDeque;

use nuwax_nav_mls_planner::SurfaceSnapshot;
use crate::surface_map::SurfaceMapCache as SurfaceGeometryCache;
use nuwax_nav_types_messages::Path3D;
use sha2::{Digest, Sha256};

use super::*;
use crate::runner::StackRunner;
use crate::surface_config::SurfaceNavigationConfig;

/// 本地计算结果保留原消息接收时刻，不以表面构建结束延长地图有效期。
#[derive(Clone)]
pub(super) struct PreparedSurfaceMap {
    pub(super) authority: GuardedMapAuthority,
    pub(super) source_timestamp_ns: i64,
    pub(super) received_at: Instant,
    pub(super) awaiting_build: bool,
    pub(super) snapshot: Result<Arc<SurfaceSnapshot>, String>,
}

#[derive(Clone)]
struct SurfaceMapHub {
    latest: Arc<Mutex<Option<PreparedSurfaceMap>>>,
    events: tokio::sync::broadcast::Sender<PreparedSurfaceMap>,
    planning: tokio::sync::watch::Sender<Option<(String,[u8;32],String)>>,
}

impl SurfaceMapHub {
    fn new() -> Self {
        Self {
            latest: Arc::new(Mutex::new(None)),
            events: tokio::sync::broadcast::channel(32).0,
            planning: tokio::sync::watch::channel(None).0,
        }
    }

    fn publish(&self, update: PreparedSurfaceMap) {
        let mut latest = self
            .latest
            .lock()
            .unwrap_or_else(|error| error.into_inner());
        *latest = Some(update.clone());
        let _ = self.events.send(update);
    }

    fn stream(&self) -> Pin<Box<dyn Stream<Item = TransportResult<GuardedMapFrame>> + Send>> {
        let latest = self
            .latest
            .lock()
            .unwrap_or_else(|error| error.into_inner());
        let receiver = self.events.subscribe();
        let initial = latest.clone();
        drop(latest);
        Box::pin(futures::stream::unfold(
            (receiver, initial),
            |(mut receiver, initial)| async move {
                if let Some(update) = initial {
                    return Some((Ok(GuardedMapFrame::Surface(update)), (receiver, None)));
                }
                match receiver.recv().await {
                    Ok(update) => Some((Ok(GuardedMapFrame::Surface(update)), (receiver, None))),
                    Err(tokio::sync::broadcast::error::RecvError::Closed) => None,
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(count)) => Some((
                        Err(nuwax_transport::TransportError::Io(format!(
                            "三维地图事件缺口: {count}条，不能跳过来源失效"
                        ))),
                        (receiver, None),
                    )),
                }
            },
        ))
    }
}

/// 每个StackRunner捕获一次配置并拥有私有共享通道。
pub(crate) fn register_surface_owners(runner: &mut StackRunner) {
    if runner.graph().find("surface_nav_scheduler").is_none() {
        return;
    }
    register_surface_owners_with_config(runner, SurfaceNavigationConfig::from_env());
}

fn register_surface_owners_with_config(
    runner: &mut StackRunner,
    config: NavStackResult<SurfaceNavigationConfig>,
) {
    let config = Arc::new(config.map_err(|error| error.to_string()));
    let hub = SurfaceMapHub::new();
    let planner_hub = hub.clone();
    let planner_config = Arc::clone(&config);
    runner.register_in_process_factory("mls_planner", move |node: &ModuleNode| {
        validate_ports(node, 5, &[1])?;
        let node = node.clone();
        let hub = planner_hub.clone();
        let config = captured_config(&planner_config)?;
        Ok(boxed_owner(
            "surface-planner",
            Box::new(move |shutdown, ready| {
                Box::pin(run_surface_planner(node, hub, config, shutdown, ready))
            }),
        ))
    });
    runner.register_in_process_factory("surface_nav_scheduler", move |node: &ModuleNode| {
        validate_ports(node, 5, &[4])?;
        let node = node.clone();
        let hub = hub.clone();
        let config = captured_config(&config)?;
        Ok(boxed_owner(
            "surface-nav-scheduler",
            Box::new(move |shutdown, ready| {
                Box::pin(run_surface_scheduler(node, hub, config, shutdown, ready))
            }),
        ))
    });
}

fn captured_config(
    config: &Result<SurfaceNavigationConfig, String>,
) -> NavStackResult<SurfaceNavigationConfig> {
    config
        .as_ref()
        .cloned()
        .map_err(|reason| NavStackError::InvalidConfig(reason.clone()))
}

async fn run_surface_scheduler(
    node: ModuleNode,
    hub: SurfaceMapHub,
    config: SurfaceNavigationConfig,
    shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let transport = lcm_transport()?;
    let socket = std::env::var("NUWAX_MOTION_GUARD_SOCKET")
        .unwrap_or_else(|_| "/run/nuwax/motion-guard.sock".into());
    let mut scheduler_config = nuwax_nav_runtime::SchedulerConfig {
        robot_width_m: config.robot_width_m,
        ..nuwax_nav_runtime::SchedulerConfig::default()
    };
    if let Some(age)=config.follower.max_observation_age_sec {
        scheduler_config.costmap_ttl=scheduler_config.costmap_ttl.min(Duration::from_secs_f64(age));
    }
    run_surface_scheduler_with_transport(node, hub, transport, shutdown, ready, async move {
        nuwax_nav_runtime::spawn_production_surface_scheduler(
            socket,
            scheduler_config,
            config.follower,
        )
        .await
    })
    .await
}

/// 先建立输入，再创建受管调度任务；联验仅替换显式运动端，复用同一生产接线。
async fn run_surface_scheduler_with_transport<F>(
    node: ModuleNode,
    hub: SurfaceMapHub,
    transport: LcmTransport,
    shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
    create_scheduler: F,
) -> NavStackResult<()>
where
    F: Future<
            Output = Result<
                (
                    nuwax_nav_runtime::NavSchedulerHandle,
                    tokio::task::JoinHandle<Result<(), nuwax_nav_runtime::RuntimeError>>,
                ),
                nuwax_nav_runtime::RuntimeError,
            >,
        > + Send,
{
    let goals = typed_stream(&transport, &node.inputs[0], "nuwax_msgs.GoalEnvelope").await?;
    let poses = typed_stream(&transport, &node.inputs[1], "geometry_msgs.PoseStamped").await?;
    let paths = typed_stream(&transport, &node.inputs[2], "nuwax_msgs.PathEnvelope").await?;
    let cancels = typed_stream(&transport, &node.inputs[4], "nuwax_msgs.CancelNavigation").await?;
    let (scheduler, task) = create_scheduler
        .await
        .map_err(|error| module_error("surface scheduler connect", error))?;
    let status = scheduler.subscribe_status();
    let mut planning=hub.planning.subscribe();
    let diagnostic_loop=async {
        loop {
            planning.changed().await.map_err(|error|module_error("planning diagnostics",error))?;
            let update=planning.borrow_and_update().clone();
            if let Some((id,hash,message))=update {
                scheduler.report_planning_diagnostic(id,hash,message).await.map_err(|error|module_error("planning diagnostics",error))?;
            }
        }
        #[allow(unreachable_code)] Ok::<(),NavStackError>(())
    };
    signal_ready(ready)?;
    let execution_loop = run_guarded_nav_loop(
        &node,
        &transport,
        &scheduler,
        shutdown,
        goals,
        poses,
        paths,
        hub.stream(),
        cancels,
        status,
        None,
        false,
    );
    let loop_future=async {tokio::select! {
        result=execution_loop=>result,
        result=diagnostic_loop=>result,
    }};
    let task_future = async move {
        task.await
            .map_err(|error| module_error("surface scheduler join", error))?
            .map_err(|error| module_error("surface scheduler task", error))
    };
    coordinate_guarded_scheduler(loop_future, task_future, || async {
        scheduler
            .shutdown()
            .await
            .map_err(|error| module_error("surface scheduler shutdown", error))
    })
    .await
}

/// 同一Geometry进程与点云修订不可变，进程代次由现有持久分配器排序。
#[derive(Default)]
struct SurfaceMapOrder {
    latest: Option<(GuardedMapAuthority, [u8; 32])>,
}

#[derive(Debug, PartialEq, Eq)]
enum MapIngress {
    New,
    Duplicate,
    Rejected(&'static str),
    Invalidated(&'static str),
}

impl SurfaceMapOrder {
    fn observe(&mut self, authority: &GuardedMapAuthority, payload: &[u8]) -> MapIngress {
        let digest: [u8; 32] = Sha256::digest(payload).into();
        if let Some((previous, previous_digest)) = &self.latest {
            if authority.producer_runtime_generation < previous.producer_runtime_generation {
                return MapIngress::Rejected("retired geometry map producer");
            }
            if authority.producer_runtime_generation == previous.producer_runtime_generation {
                if authority.commit_boot_nonce != previous.commit_boot_nonce {
                    return MapIngress::Invalidated(
                        "map commit nonce changed inside one producer generation",
                    );
                }
                if authority.map_frame_generation < previous.map_frame_generation {
                    return MapIngress::Rejected("retired map coordinate generation");
                }
                if authority.same_domain(previous)
                    && authority.committed_map.revision < previous.committed_map.revision
                {
                    return MapIngress::Rejected("stale committed map revision");
                }
                if authority.cloud_revision < previous.cloud_revision {
                    return MapIngress::Rejected("stale surface cloud revision");
                }
                if authority.cloud_revision == previous.cloud_revision {
                    return if authority == previous && digest == *previous_digest {
                        MapIngress::Duplicate
                    } else {
                        MapIngress::Invalidated("surface map revision payload mutation")
                    };
                }
            }
        }
        self.latest = Some((authority.clone(), digest));
        MapIngress::New
    }
}

fn map_authority(map: &schema::NavigationMap3D) -> GuardedMapAuthority {
    GuardedMapAuthority {
        committed_map: map.committed_map.clone(),
        producer_runtime_generation: map.producer_runtime_generation,
        map_frame_generation: map.map_frame_generation,
        cloud_revision: map.cloud_revision,
        commit_boot_nonce: map.commit_boot_nonce,
    }
}


fn prepare_surface_map(
    message: schema::NavigationMap3D,
    received_at: Instant,
    config: &SurfaceNavigationConfig,
    cache: &mut SurfaceGeometryCache,
) -> PreparedSurfaceMap {
    let authority = map_authority(&message);
    let source_timestamp_ns = message.source_timestamp_ns;
    let snapshot = if message.cleared {
        cache.reset();
        Err("surface map cleared by mapping owner".into())
    } else if message.committed_map.frame_id != config.geometry.world_frame {
        Err("surface map frame differs from configured map frame".into())
    } else {
        cache.build(&message)
    };
    PreparedSurfaceMap {
        awaiting_build: !message.cleared && snapshot.is_err(),
        authority,
        source_timestamp_ns,
        received_at,
        snapshot,
    }
}

fn body_path(
    surface: &SurfaceSnapshot,
    start: &PoseStamped,
    goal: &schema::GoalEnvelope,
    config: &SurfaceNavigationConfig,
) -> NavStackResult<Path3D> {
    body_path_with_budget(surface, start, goal, config, 30_000)
}

/// 快速检查共用相同几何核验，但不展开完整朝向搜索空间。
fn body_path_with_budget(
    surface: &SurfaceSnapshot, start: &PoseStamped, goal: &schema::GoalEnvelope,
    config: &SurfaceNavigationConfig, search_budget: usize,
) -> NavStackResult<Path3D> {
    let start_quat = [
        start.pose.orientation.x,
        start.pose.orientation.y,
        start.pose.orientation.z,
        start.pose.orientation.w,
    ];
    if surface.body_envelope().is_some() {
        let p=start.pose.position;
        let boundary:Vec<_>=goal.boundary_points.iter().map(|p|[p[0],p[1]]).collect();
        let _ = search_budget;
        let route=config.follower.pct.body_route(surface,([p.x,p.y,p.z],start_quat),
            (goal.goal.pose.position,goal.goal.pose.orientation_xyzw),goal.options,
            &boundary,config.planning_cancelled.as_deref())
            .map_err(|e|module_error("pct global planning",e))?;
        return Ok(Path3D{poses:route.into_iter().map(|(p,q)|PoseStamped {frame_id:start.frame_id.clone(),stamp:start.stamp,pose:Pose3 {position:Point3::new(p[0],p[1],p[2]),orientation:Quaternion{x:q[0],y:q[1],z:q[2],w:q[3]}}}).collect()});
    }
    let start_support = surface
        .locate_support_from_body(
            [
                start.pose.position.x,
                start.pose.position.y,
                start.pose.position.z,
            ],
            start_quat,
            config.follower.body_to_support_z_m,
        )
        .map_err(|error| module_error("surface start support", error))?;
    let goal_support = surface
        .locate_support_from_body(
            goal.goal.pose.position,
            goal.goal.pose.orientation_xyzw,
            config.follower.body_to_support_z_m,
        )
        .map_err(|error| module_error("surface goal support", error))?;
    let boundary: Vec<_> = goal
        .boundary_points
        .iter()
        .map(|point| [point[0], point[1]])
        .collect();
    let raw_supports = surface
        .plan_in_boundary(
            start_support.support_xyz,
            goal_support.support_xyz,
            &boundary,
        )
        .map_err(|error| module_error("surface planning", error))?;
    // 原始端点与float32图节点可能只差舍入尾数，这不是可执行的竖直路径段。
    // 容差远小于一个体素，真实台阶及不同楼层的高度差不会被合并。
    let duplicate_tolerance = f64::from(config.geometry.voxel_size) * 1e-4;
    let mut supports: Vec<[f64; 3]> = Vec::with_capacity(raw_supports.len());
    for support in raw_supports {
        if supports.last().is_some_and(|previous| {
            (previous[0] - support[0])
                .hypot(previous[1] - support[1])
                .hypot(previous[2] - support[2])
                <= duplicate_tolerance
        }) {
            continue;
        }
        supports.push(support);
    }
    let mut poses = Vec::with_capacity(supports.len());
    for (index, support) in supports.iter().enumerate() {
        let orientation = supports
            .get(index + 1)
            .map_or(goal.goal.pose.orientation_xyzw, |next| {
                let yaw = (next[1] - support[1]).atan2(next[0] - support[0]);
                let q = Quaternion::from_yaw(yaw);
                [q.x, q.y, q.z, q.w]
            });
        let position = surface
            .body_position_from_support(*support, orientation, config.follower.body_to_support_z_m)
            .map_err(|error| module_error("surface body reference", error))?;
        poses.push(PoseStamped {
            frame_id: start.frame_id.clone(),
            stamp: start.stamp,
            pose: Pose3 {
                position: Point3::new(position[0], position[1], position[2]),
                orientation: Quaternion {
                    x: orientation[0],
                    y: orientation[1],
                    z: orientation[2],
                    w: orientation[3],
                },
            },
        });
    }
    if poses.is_empty() {
        return Err(NavStackError::InvalidConfig("empty surface path".into()));
    }
    poses[0] = start.clone();
    *poses.last_mut().expect("nonempty path") = pose_from_schema(&goal.goal)?;
    Ok(Path3D { poses })
}

async fn publish_surface_path(
    transport: &LcmTransport,
    channel: &str,
    goal: &schema::GoalEnvelope,
    generation: i64,
    revision: i64,
    path: &Path3D,
) -> NavStackResult<()> {
    let stamp = current_unix_timestamp_ns()?;
    let header = schema_header(
        Timestamp::from_unix(stamp / 1_000_000_000, (stamp % 1_000_000_000) as u32),
        goal.committed_map_frame_id.as_str(),
    )?;
    let poses = path
        .poses
        .iter()
        .map(|pose| {
            let mut pose = schema_pose(pose)?;
            pose.header = header.clone();
            Ok(pose)
        })
        .collect::<NavStackResult<Vec<_>>>()?;
    let envelope = build_versioned_path_from_schema(
        goal,
        generation,
        revision,
        stamp,
        schema::Path { header, poses },
    )?;
    let body = schema::encode_path_envelope_body(&envelope)
        .map_err(|error| module_error("surface path encoding", error))?;
    publish_body(transport, channel, "nuwax_msgs.PathEnvelope", body).await
}

/// 离线诊断独立复核完整路线；生产执行只核对局部前方范围。
#[cfg(test)]
fn body_path_remains_traversable(
    surface: &SurfaceSnapshot,
    path: &Path3D,
    current: &PoseStamped,
    config: &SurfaceNavigationConfig,
) -> bool {
    if surface.body_envelope().is_some() {
        let position = current.pose.position;
        let q = current.pose.orientation;
        let Ok(view) = surface.with_current_body_support(
            [position.x,position.y,position.z], [q.x,q.y,q.z,q.w]) else { return false; };
        let Some((nearest, _)) = path.poses.iter().enumerate().min_by(|(_,a),(_,b)| {
            a.pose.position.distance(position).total_cmp(&b.pose.position.distance(position))
        }) else { return false; };
        // 已走过的路线不参与下一步准入。用当前实际占据视图复核剩余段，
        // 避免原图的自体回波把仍可行的路径反复废弃并打断迈步。
        let poses:Vec<_>=std::iter::once(current).chain(path.poses[nearest..].iter())
            .map(|p| {let q=p.pose.orientation;([p.pose.position.x,p.pose.position.y,p.pose.position.z],[q.x,q.y,q.z,q.w])}).collect();
        return poses.iter().all(|p|view.validate_body_pose(p.0,p.1).is_ok())
            && poses.windows(2).all(|w|view.validate_body_motion(w[0],w[1],&[]).is_ok());
    }
    let supports = path.poses.iter().map(|pose| {
        let position = pose.pose.position;
        let q = pose.pose.orientation;
        surface.support_position_from_body(
            [position.x, position.y, position.z],
            [q.x, q.y, q.z, q.w],
            config.follower.body_to_support_z_m,
        )
    }).collect::<Result<Vec<_>, _>>();
    supports.is_ok_and(|supports| surface.validate_path(&supports).is_ok())
}

async fn run_surface_planner(
    node: ModuleNode,
    hub: SurfaceMapHub,
    config: SurfaceNavigationConfig,
    shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let transport = lcm_transport()?;
    run_surface_planner_with_transport(node, hub, config, transport, shutdown, ready).await
}

fn state_matches_goal(state: &schema::NavigationState, goal: &schema::GoalEnvelope) -> bool {
    state.session_id == goal.session_id
        && state.owner_kind == goal.owner_kind
        && state.parent_request_id == goal.parent_request_id
        && state.goal_generation == goal.goal_generation
        && state.goal_payload_sha256 == goal.payload_sha256
        && state.guard_generation == goal.motion_guard_generation
}

fn retire_goal(goal: &mut Option<schema::GoalEnvelope>, retired: &mut VecDeque<[u8; 32]>) {
    if let Some(goal) = goal.take() {
        retired.push_back(goal.payload_sha256);
        if retired.len() > 32 {
            retired.pop_front();
        }
    }
}

/// 联验可注入真实但隔离的LCM连接，生产包装仍执行同一循环。
type SurfacePathComputer = Arc<dyn Fn(&SurfaceSnapshot, &PoseStamped, &schema::GoalEnvelope,
    &SurfaceNavigationConfig) -> NavStackResult<Path3D> + Send + Sync>;

async fn run_surface_planner_with_transport(
    node: ModuleNode,
    hub: SurfaceMapHub,
    config: SurfaceNavigationConfig,
    transport: LcmTransport,
    shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    run_surface_planner_with_computer(node,hub,config,transport,shutdown,ready,Arc::new(body_path)).await
}

async fn run_surface_planner_with_computer(
    node: ModuleNode,
    hub: SurfaceMapHub,
    config: SurfaceNavigationConfig,
    transport: LcmTransport,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
    computer: SurfacePathComputer,
) -> NavStackResult<()> {
    let mut maps = typed_stream(&transport, &node.inputs[0], "nuwax_msgs.NavigationMap3D").await?;
    let mut poses = typed_stream(&transport, &node.inputs[1], "geometry_msgs.PoseStamped").await?;
    let mut goals = typed_stream(&transport, &node.inputs[2], "nuwax_msgs.GoalEnvelope").await?;
    let mut cancels =
        typed_stream(&transport, &node.inputs[3], "nuwax_msgs.CancelNavigation").await?;
    let mut states =
        typed_stream(&transport, &node.inputs[4], "nuwax_msgs.NavigationState").await?;
    let mut order = SurfaceMapOrder::default();
    let cache = Arc::new(std::sync::Mutex::new(SurfaceGeometryCache::configured(&config)));
    let mut computations = tokio::task::JoinSet::new();
    let mut path_computations = tokio::task::JoinSet::new();
    let mut quick_checks = tokio::task::JoinSet::new();
    let mut quick_cancellation: Option<(u64, [u8;32], Arc<std::sync::atomic::AtomicBool>)> = None;
    let mut checked_surface: Option<std::sync::Weak<SurfaceSnapshot>> = None;
    let mut path_cancellation: Option<(u64, [u8; 32], Arc<std::sync::atomic::AtomicBool>)> = None;
    let mut pending_map: Option<(schema::NavigationMap3D, Instant, u64)> = None;
    let mut accepted_domain: Option<GuardedMapAuthority> = None;
    let mut map_epoch = 0_u64;
    let mut latest_map: Option<PreparedSurfaceMap> = None;
    let mut pose: Option<PoseStamped> = None;
    let mut goal: Option<schema::GoalEnvelope> = None;
    let mut newest_goal: Option<schema::GoalEnvelope> = None;
    let mut retired_goals = VecDeque::<[u8; 32]>::new();
    let generation = saturating_i64(runtime_generation());
    let mut path_revision = 0_i64;
    let mut last_path: Option<Path3D> = None;
    let mut retry_after = Instant::now();
    let mut failed_input: Option<(Arc<SurfaceSnapshot>, Pose3)> = None;
    let mut needs_plan = false;
    let mut execution_admitted = false;
    let mut diagnostics = InputDiagnosticLimiter::default();
    let mut ticker = tokio::time::interval(Duration::from_millis(10));
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    signal_ready(ready)?;
    let terminal: NavStackResult<()> = async {
    loop {
        for ticket in [&path_cancellation, &quick_cancellation].into_iter().flatten() {
            let (epoch,digest,flag)=ticket;
            if *epoch != map_epoch || !execution_admitted
                || goal.as_ref().is_none_or(|active|active.payload_sha256 != *digest) {
                flag.store(true, std::sync::atomic::Ordering::Relaxed);
            }
        }
        if computations.is_empty() {
            if let Some((message, received_at, epoch)) = pending_map.take() {
                let cache = Arc::clone(&cache);
                let config = config.clone();
                computations.spawn(async move {
                    let computed = tokio::task::spawn_blocking(move || {
                        let mut cache = cache.lock().map_err(|_| "表面计算缓存锁失效".to_string())?;
                        Ok::<_, String>(prepare_surface_map(message, received_at, &config, &mut cache))
                    }).await.map_err(|e| e.to_string()).and_then(|v| v);
                    (epoch, computed)
                });
            }
        }
        tokio::select! {
            _ = &mut shutdown => break,
            frame = cancels.next() => {
                let frame = required_frame(frame, "surface planner cancellation")?;
                let cancel = match schema::decode_cancel_navigation_body(&frame.payload) {
                    Ok(cancel) => cancel,
                    Err(error) => { diagnostics.reject("surface cancel", &error.to_string()); continue; }
                };
                if goal.as_ref().is_some_and(|active| validate_cancel_goal_correlation(&cancel, active).is_ok()) {
                    retire_goal(&mut goal, &mut retired_goals);
                    last_path = None; failed_input = None; needs_plan = false; execution_admitted = false;
                }
            }
            frame = states.next() => {
                let frame = required_frame(frame, "surface execution state")?;
                let state = match schema::decode_navigation_state_body(&frame.payload) {
                    Ok(state) => state,
                    Err(error) => { diagnostics.reject("surface state", &error.to_string()); continue; }
                };
                if !goal.as_ref().is_some_and(|active| state_matches_goal(&state, active)) { continue; }
                let zero_proven = !state.zero_ack_id.is_empty() && state.zero_applied_monotonic_ns > 0;
                if state.state == 4 || (matches!(state.state, 0 | 3) && zero_proven) {
                    retire_goal(&mut goal, &mut retired_goals);
                    last_path = None; failed_input = None; needs_plan = false; execution_admitted = false;
                } else if matches!(state.state, 1 | 2) && state.sink_ready != 0 {
                    execution_admitted = true;
                    // 等待旧版路径的迟到帧不能废弃刚交付的新路线。
                    if state.state == 1 && (last_path.is_none()
                        || state.path_revision == path_revision && path_revision > 0) { needs_plan = true; }
                }
            }
            computed = computations.join_next(), if !computations.is_empty() => {
                let (epoch, update) = computed.expect("已有表面任务").map_err(|e| NavStackError::Internal(e.to_string()))?;
                let update = update.map_err(NavStackError::Internal)?;
                if epoch != map_epoch || latest_map.as_ref().is_some_and(|old| old.authority.same_domain(&update.authority)
                    && (old.authority.cloud_revision > update.authority.cloud_revision || (old.authority.cloud_revision == update.authority.cloud_revision && old.snapshot.is_ok()))) { continue; }
                if update.snapshot.is_err() {
                    // 构建失败会撤销执行地图；尚未取得租约的目标仍归原拥有方，
                    // 待同域新图恢复或定向取消，不能只在规划端丢弃身份。
                    last_path = None; execution_admitted = false;
                }
                hub.publish(update.clone()); latest_map = Some(update);
                needs_plan = goal.is_some() && (needs_plan || last_path.is_none());
            }
            computed = path_computations.join_next(), if !path_computations.is_empty() => {
                let cancelled=path_cancellation.take().is_some_and(|(_,_,flag)|flag.load(std::sync::atomic::Ordering::Relaxed));
                let (epoch, planned_goal, surface, start, result): (u64, schema::GoalEnvelope, Arc<SurfaceSnapshot>, Pose3, NavStackResult<Path3D>) =
                    computed.expect("已有路径任务").map_err(|error| module_error("surface planning join",error))?;
                // 计算期间继续处理地图、位姿及取消；退休目标不能被迟到路径复活。
                if cancelled || epoch != map_epoch || !execution_admitted
                    || goal.as_ref().is_none_or(|active| active.payload_sha256 != planned_goal.payload_sha256)
                    || latest_map.as_ref().is_none_or(|current| !current.authority.permits_goal(&planned_goal) || current.snapshot.is_err()) {
                    continue;
                }
                match result {
                    Ok(path) => {
                        path_revision = path_revision.checked_add(1).ok_or_else(|| NavStackError::Internal("surface path revision exhausted".into()))?;
                        publish_surface_path(&transport, &node.outputs[0], &planned_goal, generation, path_revision, &path).await?;
                        last_path = Some(path); needs_plan = false; failed_input = None;
                    }
                    Err(error) => {
                        diagnostics.reject("surface planning", &error.to_string());
                        // 完整路线暂缺时限频重试；输入与取消仍持续处理。
                        retry_after = Instant::now() + Duration::from_secs(1);
                        failed_input = Some((surface, start));
                        hub.planning.send_replace(Some((planned_goal.session_id.clone(),planned_goal.payload_sha256,
                            format!("等待可通行路径：{error}"))));
                    }
                }
            }
            computed = quick_checks.join_next(), if !quick_checks.is_empty() => {
                let cancelled=quick_cancellation.take().is_some_and(|(_,_,flag)|flag.load(std::sync::atomic::Ordering::Relaxed));
                let (epoch, planned_goal, _surface, result): (u64, schema::GoalEnvelope, Arc<SurfaceSnapshot>, NavStackResult<Path3D>) =
                    computed.expect("已有快速路径检查").map_err(|error|module_error("quick surface planning",error))?;
                if cancelled || epoch != map_epoch || !execution_admitted
                    || goal.as_ref().is_none_or(|active|active.payload_sha256!=planned_goal.payload_sha256)
                    || (last_path.is_some() && !needs_plan)
                    || latest_map.as_ref().is_none_or(|current|!current.authority.permits_goal(&planned_goal)||current.snapshot.is_err()) {
                    continue;
                }
                if let Ok(path)=result {
                    // 新地图的可执行路线已经就绪，旧地图上的慢搜索立即退休。
                    if let Some((_,_,flag))=&path_cancellation {flag.store(true,std::sync::atomic::Ordering::Relaxed);}
                    path_revision=path_revision.checked_add(1).ok_or_else(||module_error("surface revision","overflow"))?;
                    publish_surface_path(&transport,&node.outputs[0],&planned_goal,generation,path_revision,&path).await?;
                    last_path=Some(path);needs_plan=false;failed_input=None;
                }
            }
            frame = maps.next() => {
                let frame = required_frame(frame, "surface planner map")?;
                let received_at = Instant::now();
                let message = match schema::decode_navigation_map_3d_body(&frame.payload) {
                    Ok(message) => message,
                    Err(error) => { diagnostics.reject("surface map", &error.to_string()); continue; }
                };
                match order.observe(&map_authority(&message), &frame.payload) {
                    MapIngress::Rejected(reason) => { diagnostics.reject("surface map", reason); continue; }
                    MapIngress::Duplicate => continue,
                    MapIngress::Invalidated(reason) => {
                        map_epoch = map_epoch.checked_add(1).ok_or_else(|| NavStackError::Internal("表面构建票据耗尽".into()))?;
                        pending_map.take();
                        let authority = order.latest.as_ref().map_or_else(|| map_authority(&message), |(authority,_)| authority.clone());
                        let update = PreparedSurfaceMap { authority, source_timestamp_ns:message.source_timestamp_ns,
                            received_at, awaiting_build: false, snapshot:Err(reason.into()) };
                        retire_goal(&mut goal, &mut retired_goals);
                        pose = None; last_path = None; failed_input = None; needs_plan = false; execution_admitted = false;
                        hub.publish(update.clone()); latest_map = Some(update);
                        diagnostics.reject("surface map integrity",reason);continue;
                    }
                    MapIngress::New => {}
                }
                let authority = map_authority(&message);
                let changed_domain = accepted_domain.as_ref().is_some_and(|old| !old.same_domain(&authority));
                accepted_domain = Some(authority.clone());
                if changed_domain || message.cleared {
                    map_epoch = map_epoch.checked_add(1).ok_or_else(|| NavStackError::Internal("表面构建票据耗尽".into()))?;
                    let future_goal_matches = !message.cleared
                        && goal.as_ref().is_some_and(|pending| authority.permits_goal(pending));
                    if !future_goal_matches { retire_goal(&mut goal, &mut retired_goals); }
                    pose = None; last_path = None; failed_input = None; execution_admitted = false;
                    let invalid = PreparedSurfaceMap { authority: authority.clone(), source_timestamp_ns: message.source_timestamp_ns,
                        received_at, awaiting_build: !message.cleared, snapshot: Err("规范地图清空或切换，等待该域完整精细源".into()) };
                    hub.publish(invalid.clone()); latest_map = Some(invalid);
                }
                // 只保留最新待构建输入，已经开始的计算仍由本拥有方收尾。
                pending_map = Some((message, received_at, map_epoch));
            }
            frame = goals.next() => {
                let frame = required_frame(frame, "surface planner goal")?;
                let incoming = match schema::decode_goal_envelope_body(&frame.payload) {
                    Ok(goal) => goal,
                    Err(error) => { diagnostics.reject("surface goal", &error.to_string()); continue; }
                };
                if retired_goals.contains(&incoming.payload_sha256) { continue; }
                if goal.as_ref().is_some_and(|active|active == &incoming) { continue; }
                if newest_goal.as_ref().is_some_and(|current| validate_goal_envelope_order(&incoming, current).is_err()) { continue; }
                if let Err(error) = navigation_boundary_from_goal(&incoming) { diagnostics.reject("surface goal boundary",&error.to_string());continue; }
                config.follower.pct.clear_local_costs();
                newest_goal = Some(incoming.clone()); goal = Some(incoming);
                path_revision = 0; last_path = None; needs_plan = true; execution_admitted = false;
                retry_after = Instant::now(); checked_surface = None; failed_input = None;
            }
            frame = poses.next() => {
                let Some(input)=drain_guarded_pose_input(frame,&mut poses,&mut diagnostics,None)? else {continue;};
                let incoming=input.pose;
                if pose.as_ref().is_none_or(|previous| incoming.stamp.nanos>previous.stamp.nanos) {
                    pose=Some(incoming);
                }
            }
            _ = ticker.tick() => {
                if !quick_checks.is_empty() || (!needs_plan && last_path.is_some()) { continue; }
                let (Some(map), Some(pose), Some(active_goal)) = (latest_map.as_ref(), pose.as_ref(), goal.as_ref()) else { continue; };
                if !execution_admitted || !map.authority.permits_goal(active_goal) { continue; }
                let Ok(surface) = map.snapshot.as_ref() else { continue; };
                if !path_computations.is_empty() {
                    let unchanged=checked_surface.as_ref().and_then(std::sync::Weak::upgrade)
                        .is_some_and(|previous|previous.same_geometry(surface));
                    if unchanged || (last_path.is_some() && !needs_plan) {continue;}
                    checked_surface=Some(Arc::downgrade(surface));
                    let surface=Arc::clone(surface);let pose=pose.clone();let planned_goal=active_goal.clone();
                    let epoch=map_epoch;let mut quick_config=config.clone();
                    let flag=Arc::new(std::sync::atomic::AtomicBool::new(false));
                    quick_config.planning_cancelled=Some(Arc::clone(&flag));
                    quick_cancellation=Some((epoch,planned_goal.payload_sha256,flag));
                    quick_checks.spawn_blocking(move || {
                        let started=Instant::now();
                        let result=body_path_with_budget(&surface,&pose,&planned_goal,&quick_config,0);
                        if result.is_ok() {tracing::info!(goal_id=%planned_goal.session_id,mode="latest_map_quick",
                            elapsed_ms=started.elapsed().as_secs_f64()*1000.0,
                            points=result.as_ref().ok().map(|path|path.poses.len()),"三维路径计算完成");}
                        (epoch,planned_goal,surface,result)
                    });
                    continue;
                }
                if Instant::now() < retry_after { continue; }
                // 同一失败输入不会产生新答案；精细观察也影响机体通行，更新后允许重试。
                if failed_input.as_ref().is_some_and(|(previous, start)| {
                    let same_observations = match (previous.observations(), surface.observations()) {
                        (Some(a), Some(b)) => Arc::ptr_eq(a, b),
                        (None, None) => true,
                        _ => false,
                    };
                    previous.same_geometry(surface) && same_observations && *start == pose.pose
                }) { continue; }
                // 同一几何不并行重复快速搜索；新几何才允许一次完整直连抢先接续。
                checked_surface = Some(Arc::downgrade(surface));
                let surface = Arc::clone(surface);
                let pose = pose.clone();
                let planned_goal = active_goal.clone();
                let mut config = config.clone();
                let epoch = map_epoch;
                let cancelled = Arc::new(std::sync::atomic::AtomicBool::new(false));
                config.planning_cancelled = Some(Arc::clone(&cancelled));
                path_cancellation = Some((epoch, planned_goal.payload_sha256, cancelled));
                let computer = Arc::clone(&computer);
                hub.planning.send_replace(Some((planned_goal.session_id.clone(),planned_goal.payload_sha256,
                    "正在计算三维路径".into())));
                // 全局搜索独立执行；已交付路线的局部有效性由跟随器持续判断。
                let queued=Instant::now();
                path_computations.spawn_blocking(move || {
                    let started = Instant::now();
                    let span=tracing::info_span!("navigation.plan",goal_id=%planned_goal.session_id,
                        goal_generation=planned_goal.goal_generation);
                    let _entered=span.enter();
                    let result = computer(&surface,&pose,&planned_goal,&config);
                    tracing::info!(goal_id=%planned_goal.session_id, queue_ms=started.duration_since(queued).as_secs_f64()*1000.0, elapsed_ms=started.elapsed().as_secs_f64()*1000.0, points=result.as_ref().ok().map(|p|p.poses.len()), error=?result.as_ref().err(), "三维路径计算完成");
                    (epoch, planned_goal, surface, pose.pose, result)
                });
            }
        }
    }
    Ok(())
    }.await;
    for (_,_,flag) in [path_cancellation,quick_cancellation].into_iter().flatten() {
        flag.store(true, std::sync::atomic::Ordering::Relaxed);
    }
    pending_map.take();
    // 关闭/错误不遗留阻塞构建，晚到结果不再交付执行。
    while let Some(joined) = computations.join_next().await {
        if let Err(error) = joined {
            if terminal.is_ok() { return Err(NavStackError::Internal(error.to_string())); }
        }
    }
    while let Some(joined) = path_computations.join_next().await {
        if let Err(error) = joined {
            if terminal.is_ok() { return Err(module_error("surface planning shutdown",error)); }
        }
    }
    while let Some(joined) = quick_checks.join_next().await {
        if let Err(error)=joined {if terminal.is_ok() {return Err(module_error("quick planning shutdown",error));}}
    }
    terminal
}

#[cfg(test)]
mod tests {
    use super::super::guarded_tests::RecordingMotion;
    use super::*;
    use nuwax_nav_runtime::{NavSchedulerHandle, SchedulerPhase, TerminalOutcome};

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    #[ignore = "需要本轮冻结的实机规范地图、机体位姿及实机配置"]
    async fn recorded_live_surface_navigation_diagnostic() {
        nuwax_core::logging::init_component_logging("navigation-diagnostic", "nuwax_nav_mls_planner=debug");
        struct Blocks(std::path::PathBuf);
        #[async_trait::async_trait]
        impl nuwax_rpc::RpcHandler for Blocks {
            async fn handle(&self, request: nuwax_rpc::RpcRequest) -> nuwax_rpc::RpcResponse {
                let hash = request.args.0[0]["sha256_hex"].as_str().unwrap();
                let bytes = std::fs::read(self.0.join(hash)).unwrap();
                nuwax_rpc::RpcResponse::ok(request.id, serde_json::json!({"data_hex": bytes.iter().map(|v|format!("{v:02x}")).collect::<String>()}))
            }
        }
        let directory = std::path::PathBuf::from(std::env::var("NUWAX_NAV_DIAGNOSTIC_INPUT").unwrap());
        let broker = nuwax_rpc::RpcBroker::new();
        let method = "GeometryRuntimeService/get_navigation_map_block";
        broker.register(method, Box::new(Blocks(directory.join("blocks")))).await.unwrap();
        let transport = Arc::new(nuwax_rpc::LcmRpcTransport::from_provider(&std::env::var("NUWAX_LCM_URL").unwrap()).unwrap());
        let server = broker.serve(transport, method).await.unwrap();
        let bytes = std::fs::read(directory.join("navigation-map.lcm")).unwrap();
        let message = schema::decode_navigation_map_3d_body(&bytes[8..]).unwrap();
        let pose_file = std::env::var("NUWAX_NAV_DIAGNOSTIC_POSE_FILE").map(std::path::PathBuf::from).unwrap_or_else(|_|directory.join("pose.json"));
        let value: serde_json::Value = serde_json::from_slice(&std::fs::read(pose_file).unwrap()).unwrap();
        let position: [f64; 3] = serde_json::from_value(value["position"].clone()).unwrap();
        let quaternion: [f64; 4] = serde_json::from_value(value["quaternion"].clone()).unwrap();
        let mut config = SurfaceNavigationConfig::from_env().unwrap();
        if let Ok(clearance) = std::env::var("NUWAX_NAV_DIAGNOSTIC_CLEARANCE") {
            config.geometry.wall_clearance_m = clearance.parse().unwrap();
        }
        let mut cache = SurfaceGeometryCache::configured(&config);
        let start = Instant::now();
        let captured_message = message.clone();
        let map = tokio::task::spawn_blocking(move || cache.build(&captured_message)).await.unwrap().unwrap();
        eprintln!("实机表面构建耗时 {:?}，机体 {:?}，模型 {:?}", start.elapsed(), position, map.body_envelope());
        eprintln!("原图机体检查 {:?}", map.validate_body_pose(position, quaternion));
        let view = map.with_current_body_support(position, quaternion);
        eprintln!("当前足迹派生结果 {:?}", view.as_ref().map(|_| ()));
        let view = view.unwrap();
        view.validate_body_pose(position, quaternion).unwrap();
        if std::env::var_os("NUWAX_NAV_DIAGNOSTIC_ROTATION").is_some() {
            let model=view.body_envelope().unwrap();
            let support=view.locate_support_from_body(position,quaternion,model.body_to_support_z_m);
            eprintln!("实际支撑 {support:?}");
            if let Ok(support)=support {
                for index in -12..=12 {
                    let yaw=f64::from(index)*std::f64::consts::PI/12.0;
                    let target=view.body_pose_on_surface([position[0],position[1]],yaw,support.support_xyz);
                    let swept=target.as_ref().map(|target|view.validate_body_motion((position,quaternion),*target,&[]));
                    eprintln!("原地转向 yaw={yaw:.3} 投影={target:?} 扫掠={swept:?}");
                }
            }
            server.stop().await.unwrap();
            return;
        }
        if std::env::var_os("NUWAX_NAV_DIAGNOSTIC_EDGES").is_some() {
            let mut printed=std::collections::BTreeSet::new();
            for x in -1..=1 { for y in -1..=1 { for z in -11..=-4 {
                if let Ok(location)=view.locate_support([(f64::from(x)+0.5)*0.1,(f64::from(y)+0.5)*0.1,f64::from(z)*0.05]) {
                    if printed.insert(location.cell) { eprintln!("近场支撑 {:?} 邻接 {:?}",location,view.neighbors(location.cell)); }
                }
            }}}
        }
        let pose = PoseStamped {
            frame_id: nuwax_nav_types_messages::NavFrameId::Map, stamp: nuwax_nav_types_messages::Timestamp { nanos: value["timestamp_ns"].as_i64().unwrap(), clock: nuwax_nav_types_messages::ClockKind::Unix },
            pose: Pose3 { position: Point3::new(position[0], position[1], position[2]),
                orientation: Quaternion { x: quaternion[0], y: quaternion[1], z: quaternion[2], w: quaternion[3] } },
        };
        if let Ok(path_file) = std::env::var("NUWAX_NAV_DIAGNOSTIC_PUBLISHED_PATH") {
            use nuwax_nav_runtime::{NavPlanner, ExecutionMap, ExecutionPath, RouteFollower, SurfaceFollower};
            let points: Vec<[f64; 3]> = serde_json::from_slice(&std::fs::read(path_file).unwrap()).unwrap();
            let mut path = nuwax_nav_types_messages::Path3D { poses: points.into_iter().map(|p| {
                let mut point = pose.clone();
                point.pose.position = Point3::new(p[0], p[1], p[2]);
                point
            }).collect() };
            let rotation = std::env::var_os("NUWAX_NAV_DIAGNOSTIC_ROTATION_CONTROL").is_some();
            if rotation {
                let mut target = pose.clone();
                let (s, c) = (-0.07_f64 / 2.0).sin_cos();
                let q = pose.pose.orientation;
                target.pose.orientation = Quaternion { x:c*q.x-s*q.y, y:s*q.x+c*q.y, z:c*q.z+s*q.w, w:c*q.w-s*q.z };
                path.poses = vec![pose.clone(), target];
            }
            let goal = path.poses.last().unwrap().clone();
            let mut follower: Box<dyn NavPlanner> = if rotation {
                Box::new(SurfaceFollower::new(config.follower.clone()).unwrap())
            } else {
                Box::new(RouteFollower::new(config.follower.clone()).unwrap())
            };
            follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&map))).unwrap();
            follower.update_odometry(pose.clone());
            follower.configure_goal(&goal, nuwax_nav_types_messages::GoalOptions {
                allow_reverse: false, position_tolerance_m: 0.2,
                yaw_tolerance_rad: if rotation {0.01} else {std::f64::consts::PI},
            }).unwrap();
            follower.replace_execution_path(ExecutionPath::Surface(path), None, goal.clone()).unwrap();
            if std::env::var_os("NUWAX_NAV_DIAGNOSTIC_PUBLISHED_CLOSED_LOOP").is_some() {
                let mut actual = pose.clone();
                let mut replans = 0;
                let mut commands = 0;
                let mut travelled = 0.0;
                let mut frames = Vec::new();
                let mut outcome = "控制步数耗尽".to_owned();
                for tick in 0..1600 {
                    actual.stamp.nanos += 50_000_000;
                    follower.update_odometry(actual.clone());
                    let began = Instant::now();
                    let step = follower.step(config.robot_width_m);
                    let elapsed_ms = began.elapsed().as_secs_f64() * 1000.0;
                    frames.push(serde_json::json!({"tick":tick,"elapsed_ms":elapsed_ms,
                        "position":[actual.pose.position.x,actual.pose.position.y,actual.pose.position.z],
                        "yaw":actual.pose.orientation.yaw(),"step":format!("{step:?}")}));
                    match step {
                        nuwax_nav_runtime::PlannerStep::Succeeded => {outcome="到达".into(); break;}
                        nuwax_nav_runtime::PlannerStep::Command(command) => {
                            follower.observe_applied_command(command);
                            commands += usize::from(command.linear.x != 0.0);
                            let p=actual.pose.position;let q=actual.pose.orientation;let yaw=q.yaw();
                            // 场景为现场平地，独立刚体积分保持实测高度及倾斜；不以待测地形生成运动真值。
                            let angle=command.angular_z*0.05;
                            let (a,b)=if command.angular_z.abs()<1e-9 {(0.05,0.0)} else {
                                (angle.sin()/command.angular_z,(1.0-angle.cos())/command.angular_z)
                            };
                            let dx=a*command.linear.x-b*command.linear.y;
                            let dy=b*command.linear.x+a*command.linear.y;
                            actual.pose.position.x+=yaw.cos()*dx-yaw.sin()*dy;
                            actual.pose.position.y+=yaw.sin()*dx+yaw.cos()*dy;
                            travelled+=(actual.pose.position.x-p.x).hypot(actual.pose.position.y-p.y);
                            let (s,c)=(angle*0.5).sin_cos();
                            actual.pose.orientation=Quaternion{x:c*q.x-s*q.y,y:s*q.x+c*q.y,z:c*q.z+s*q.w,w:c*q.w-s*q.z};
                        }
                        nuwax_nav_runtime::PlannerStep::ReplanRequired(reason) => {
                            replans+=1;
                            eprintln!("原路径闭环重规划{replans}: {reason}");
                            if replans>8 {outcome=format!("重规划未恢复: {reason}");break;}
                            follower.observe_applied_command(nuwax_nav_types_messages::Twist2::ZERO);
                            follower.stop();
                            // 同一目标保留PCT收到的局部阻塞代价，不重新配置目标清空反馈。
                            let p=goal.pose.position;
                            let mut envelope=goal_for(&message,1,[p.x,p.y,p.z]);
                            envelope.options.allow_reverse=false;
                            envelope.options.yaw_tolerance_rad=std::f64::consts::PI;
                            envelope.options.position_tolerance_m=0.2;
                            let q=goal.pose.orientation;
                            envelope.goal.pose.orientation_xyzw=[q.x,q.y,q.z,q.w];
                            match body_path(&map,&actual,&envelope,&config) {
                                Ok(path)=>follower.replace_execution_path(ExecutionPath::Surface(path),None,goal.clone()).unwrap(),
                                Err(reason)=>{outcome=format!("全局重规划失败: {reason}");break;}
                            }
                        }
                        other=>{outcome=format!("{other:?}");break;}
                    }
                }
                let summary=serde_json::json!({"outcome":outcome,"replans":replans,"translation_commands":commands,
                    "travelled_m":travelled,"distance_to_path_endpoint_m":actual.pose.position.distance(goal.pose.position),
                    "frames":frames});
                eprintln!("原路径闭环 outcome={outcome}, replans={replans}, commands={commands}, travelled={travelled}");
                let output=std::env::var("NUWAX_NAV_DIAGNOSTIC_OUTPUT").unwrap();
                std::fs::write(std::path::Path::new(&output).join("原路径闭环.json"),serde_json::to_vec_pretty(&summary).unwrap()).unwrap();
                server.stop().await.unwrap();
                return;
            }
            let mut results = Vec::new();
            for tick in 0..12 {
                let started = Instant::now();
                // 固定现场站姿和零速回执，量化原地重复计算及其速度候选。
                follower.update_odometry(pose.clone());
                follower.observe_applied_command(nuwax_nav_types_messages::Twist2::ZERO);
                let step = follower.step(config.robot_width_m);
                let elapsed_ms = started.elapsed().as_secs_f64() * 1000.0;
                eprintln!("冻结已发布路径 tick={tick} elapsed_ms={elapsed_ms:.3} step={step:?}");
                results.push(serde_json::json!({"tick":tick, "elapsed_ms":elapsed_ms, "step":format!("{step:?}")}));
            }
            if let Ok(output) = std::env::var("NUWAX_NAV_DIAGNOSTIC_OUTPUT") {
                std::fs::write(std::path::Path::new(&output).join("原路径控制统计.json"), serde_json::to_vec_pretty(&results).unwrap()).unwrap();
            }
            server.stop().await.unwrap();
            return;
        }
        let mut current = goal_for(&message, 1, position);
        current.goal.pose.orientation_xyzw = quaternion;
        let current_path = body_path(&map, &pose, &current, &config).expect("实录当前占据目标必须可以规划");
        assert!(body_path_remains_traversable(&map,&current_path,&pose,&config),
            "同一实际地图不能因复核丢失当前占据视图而反复重规划");
        let mut successful = 0;
        let mut measurements=Vec::new();
        let radii = std::env::var("NUWAX_NAV_DIAGNOSTIC_RADII").ok()
            .map(|v|v.split(',').map(|s|s.parse::<f64>().unwrap()).collect::<Vec<_>>())
            .unwrap_or_else(||vec![0.2,0.5,0.8]);
        let directions = if std::env::var_os("NUWAX_NAV_DIAGNOSTIC_FORWARD").is_some() {1} else {
            std::env::var("NUWAX_NAV_DIAGNOSTIC_DIRECTIONS").ok().map(|value|value.parse::<i32>().unwrap()).unwrap_or(8)
        };
        assert!((1..=360).contains(&directions));
        for (index, (radius, direction)) in radii.into_iter()
            .flat_map(|radius| (0..directions).map(move |direction| (radius, direction))).enumerate() {
            let angle = f64::from(direction) * std::f64::consts::TAU/f64::from(directions);
            let mut goal = goal_for(&message, (index+2) as i64,
                [position[0]+radius*angle.cos(), position[1]+radius*angle.sin(), position[2]]);
            goal.goal.pose.orientation_xyzw = quaternion;
            if let Ok(target) = std::env::var("NUWAX_NAV_DIAGNOSTIC_TARGET") {
                goal.goal.pose.position=serde_json::from_str(&target).unwrap();
            }
            if std::env::var_os("NUWAX_NAV_DIAGNOSTIC_POSITION_ONLY").is_some() {
                goal.options.yaw_tolerance_rad=std::f64::consts::PI;
            }
            if std::env::var_os("NUWAX_NAV_DIAGNOSTIC_REVERSE").is_some() {
                goal.options.allow_reverse=true;
            }
            if let Ok(tolerance)=std::env::var("NUWAX_NAV_DIAGNOSTIC_POSITION_TOLERANCE") {
                goal.options.position_tolerance_m=tolerance.parse().unwrap();
            }
            let timer=std::env::var("NUWAX_NAV_DIAGNOSTIC_CASE_TIMEOUT_SEC").ok().map(|value| {
                let seconds=value.parse::<f64>().unwrap();
                let flag=Arc::new(std::sync::atomic::AtomicBool::new(false));
                config.planning_cancelled=Some(flag.clone());
                tokio::spawn(async move {
                    tokio::time::sleep(Duration::from_secs_f64(seconds)).await;
                    flag.store(true,std::sync::atomic::Ordering::Relaxed);
                })
            });
            let before = Instant::now();
            let result = body_path(&map, &pose, &goal, &config);
            let search_ms=before.elapsed().as_secs_f64()*1000.0;
            if let Some(timer)=timer {timer.abort();let _=timer.await;}
            let validation_started=Instant::now();
            let validated=result.as_ref().is_ok_and(|path|body_path_remains_traversable(&map,path,&pose,&config));
            let validation_ms=validation_started.elapsed().as_secs_f64()*1000.0;
            successful += usize::from(validated);
            measurements.push(serde_json::json!({"radius":radius,"direction":direction,
                "goal":goal.goal.pose.position,"search_ms":search_ms,"validation_ms":validation_ms,
                "total_ms":search_ms+validation_ms,"success":validated,
                "points":result.as_ref().map(|path|path.poses.len()).unwrap_or(0),
                "error":result.as_ref().err().map(|error|error.to_string())}));
            eprintln!("半径{radius}米方向{direction}规划 {:?}，耗时 {:?}",
                result.as_ref().map(|path|path.poses.len()), before.elapsed());
            if let Ok(output) = std::env::var("NUWAX_NAV_DIAGNOSTIC_OUTPUT") {
                if let Ok(path) = &result {
                    let points: Vec<_> = path.poses.iter().map(|p| {
                        let b = p.pose.position;
                        serde_json::json!([b.x,b.y,b.z,p.pose.orientation.yaw()])
                    }).collect();
                    std::fs::write(std::path::Path::new(&output).join(format!("路径-{radius}-{direction}.json")),
                        serde_json::to_vec(&points).unwrap()).unwrap();
                }
            }
        }
        config.planning_cancelled=None;
        if let Ok(output)=std::env::var("NUWAX_NAV_DIAGNOSTIC_OUTPUT") {
            std::fs::write(std::path::Path::new(&output).join("规划统计.json"),serde_json::to_vec_pretty(&measurements).unwrap()).unwrap();
        }
        eprintln!("实录候选中，可规划{successful}个");
        if std::env::var_os("NUWAX_NAV_DIAGNOSTIC_CONTROL").is_some() {
            use nuwax_nav_runtime::{NavPlanner,ExecutionMap,ExecutionPath,PlannerStep,RouteFollower};
            let mut follower=RouteFollower::new(config.follower.clone()).unwrap();
            follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&map))).unwrap();
            let mut actual=pose.clone();
            let distances = std::env::var("NUWAX_NAV_DIAGNOSTIC_CONTROL_DISTANCES").ok()
                .map(|v|v.split(',').map(|s|s.parse::<f64>().unwrap()).collect::<Vec<_>>())
                .unwrap_or_else(||vec![0.2,0.4]);
            for (index, distance) in distances.into_iter().enumerate() {
                let mut goal=goal_for(&message,100+index as i64,[position[0]+distance,position[1],position[2]]);
                goal.goal.pose.orientation_xyzw=quaternion;
                goal.options.position_tolerance_m=0.04;
                goal.options.yaw_tolerance_rad=0.15;
                if let Ok(value)=std::env::var("NUWAX_NAV_DIAGNOSTIC_TARGET") {
                    goal.goal.pose.position=serde_json::from_str(&value).unwrap();
                }
                if std::env::var_os("NUWAX_NAV_DIAGNOSTIC_POSITION_ONLY").is_some() {
                    goal.options.yaw_tolerance_rad=std::f64::consts::PI;
                }
                if std::env::var_os("NUWAX_NAV_DIAGNOSTIC_REVERSE").is_some() {
                    goal.options.allow_reverse=true;
                }
                if let Ok(value)=std::env::var("NUWAX_NAV_DIAGNOSTIC_POSITION_TOLERANCE") {
                    goal.options.position_tolerance_m=value.parse().unwrap();
                }
                let target=pose_from_schema(&goal.goal).unwrap();
                follower.stop();
                follower.configure_goal(&target,goal.options).unwrap();
                follower.update_odometry(actual.clone());
                let path=body_path(&map,&actual,&goal,&config).expect("实录闭环目标可规划");
                follower.replace_execution_path(ExecutionPath::Surface(path),None,target.clone()).unwrap();
                let mut completed=false;
                let mut nonzero=0;
                let mut replans=0;
                for tick in 0..800 {
                    actual.stamp.nanos+=50_000_000;
                    follower.update_odometry(actual.clone());
                    match follower.step(config.robot_width_m) {
                        PlannerStep::Succeeded => { completed=true;break; }
                        PlannerStep::Command(twist) => {
                            follower.observe_applied_command(twist);
                            let mut actual_twist=twist;
                            // 独立响应模型模拟小角速度不足以带动步态；不是设备标定值。
                            if let Ok(value)=std::env::var("NUWAX_NAV_DIAGNOSTIC_ANGULAR_DEADBAND") {
                                if actual_twist.angular_z.abs()<value.parse::<f64>().unwrap() {
                                    actual_twist.angular_z=0.0;
                                }
                            }
                            if tick%100==0 {eprintln!("实录控制 tick={tick} command={twist:?} pose={actual:?}");}
                            nonzero+=usize::from(twist.linear.x.hypot(twist.linear.y)>1e-6 || twist.angular_z.abs()>1e-6);
                            let before=actual.pose.position;
                            let q=actual.pose.orientation;
                            let yaw=q.yaw();
                            let next=nuwax_nav_control::integrate(nuwax_nav_control::PlanarPose{x:before.x,y:before.y,yaw},actual_twist,0.05);
                            // 此份现场已确认是平地；独立运动学保持实测高度和
                            // 横滚/俯仰，不拿待测几何投影反过来生成位姿真值。
                            let half=(next.yaw-yaw)*0.5;
                            let (s,c)=half.sin_cos();
                            actual.pose.position=Point3::new(next.x,next.y,before.z);
                            actual.pose.orientation=Quaternion{x:c*q.x-s*q.y,y:s*q.x+c*q.y,
                                z:c*q.z+s*q.w,w:c*q.w-s*q.z};
                        }
                        PlannerStep::ReplanRequired(reason)=>{
                            replans+=1;assert!(replans<=8,"实录局部重规划不能无限循环: {reason}");
                            // 独立运动学中归零立即生效；不把该模型当作实机制动证明。
                            follower.stop();follower.configure_goal(&target,goal.options).unwrap();
                            follower.update_odometry(actual.clone());
                            let started=Instant::now();
                            let replacement=body_path(&map,&actual,&goal,&config).expect("实录实际位姿可重规划");
                            eprintln!("实录局部接续第{replans}次，耗时 {:?}，原因{reason}",started.elapsed());
                            follower.replace_execution_path(ExecutionPath::Surface(replacement),None,target.clone()).unwrap();
                        }
                        other=>panic!("实录闭环中断 tick={tick}, pose={actual:?}, result={other:?}"),
                    }
                }
                let error=(actual.pose.position.x-target.pose.position.x).hypot(actual.pose.position.y-target.pose.position.y);
                eprintln!("实录运动学闭环：目标{distance}米，非零指令{nonzero}，局部接续{replans}，最终位置{:?}，水平误差{error}",actual.pose.position);
                assert!(completed && nonzero>0 && error<goal.options.position_tolerance_m,"实录闭环不能停留在路径受理或零速循环");
                follower.stop();
            }
        }
        server.stop().await.unwrap();
    }

    #[test]
    fn old_source_revision_is_usable_but_a_goal_from_old_coordinates_is_not() {
        let first = map_message(1, "map");
        let goal = goal_for(&first, 1, [1.5, 1.5, 2.5]);
        let mut next = first.clone(); next.cloud_revision += 1;
        next.committed_map.revision += 1; next.committed_map.payload_sha256 = [42; 32];
        assert!(map_authority(&next).permits_goal(&goal));
        next.map_frame_generation += 1;
        assert!(!map_authority(&next).permits_goal(&goal));
    }

    fn config() -> SurfaceNavigationConfig {
        SurfaceNavigationConfig::new("map".into(), 0.4, 0.3, 0.6, 0.1, 0.2, 35.0, 0.2).unwrap()
    }

    fn map_message(revision: i64, session: &str) -> schema::NavigationMap3D {
        let stamp = current_unix_timestamp_ns().unwrap();
        let mut data = Vec::new();
        for x in 0..40 {
            for y in 0..30 {
                for value in [(x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 2.05_f32] {
                    data.extend_from_slice(&value.to_le_bytes());
                }
            }
        }
        schema::NavigationMap3D {
            schema_version: 2,
            producer_runtime_generation: 17,
            committed_map: schema::CommittedMapIdentity {
                session_id: session.into(),
                session_generation: 1,
                revision: 1,
                payload_sha256: [0x31; 32],
                frame_id: "map".into(),
            },
            map_frame_generation: 1,
            cloud_revision: revision,
            source_timestamp_ns: stamp,
            source_voxel_size: 0.1,
            commit_boot_nonce: [0x44; 16],
            commit_sequence: revision,
            cleared: false,
            virtual_obstacles: None,
            fine: None,
            cloud: schema::PointCloud2 {
                header: schema::Header {
                    seq: revision as i32,
                    stamp_sec: (stamp / 1_000_000_000) as i32,
                    stamp_nsec: (stamp % 1_000_000_000) as i32,
                    frame_id: "map".into(),
                },
                height: 1,
                width: 1200,
                fields: ["x", "y", "z"]
                    .iter()
                    .enumerate()
                    .map(|(i, name)| schema::PointField {
                        name: (*name).into(),
                        offset: (i * 4) as i32,
                        datatype: 7,
                        count: 1,
                    })
                    .collect(),
                is_bigendian: false,
                point_step: 12,
                row_step: 14400,
                data,
                is_dense: true,
            },
        }
    }

    fn goal_for(
        map: &schema::NavigationMap3D,
        generation: i64,
        position: [f64; 3],
    ) -> schema::GoalEnvelope {
        let stamp = current_unix_timestamp_ns().unwrap();
        let mut goal = schema::GoalEnvelope {
            map_frame_generation: map.map_frame_generation,
            schema_version: schema::NAVIGATION_ENVELOPE_SCHEMA_VERSION,
            session_id: format!("surface-goal-{generation}"),
            owner_kind: schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
            parent_request_id: format!("surface-request-{generation}"),
            goal_generation: generation,
            producer: "surface-integration".into(),
            producer_epoch: 1,
            producer_runtime_generation: map.producer_runtime_generation,
            committed_map_session_id: map.committed_map.session_id.clone(),
            committed_map_session_generation: map.committed_map.session_generation,
            committed_map_revision: map.committed_map.revision,
            committed_map_payload_sha256: map.committed_map.payload_sha256,
            committed_map_frame_id: "map".into(),
            source_timestamp_ns: stamp,
            motion_guard_generation: 7,
            motion_guard_boot_nonce: [7; 16],
            motion_stop_generation: 1,
            goal: schema::PoseStamped {
                header: schema::Header {
                    stamp_sec: (stamp / 1_000_000_000) as i32,
                    stamp_nsec: (stamp % 1_000_000_000) as i32,
                    frame_id: "map".into(),
                    ..schema::Header::default()
                },
                pose: schema::Pose {
                    position,
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            },
            ..schema::GoalEnvelope::default()
        };
        schema::seal_goal_envelope(&mut goal).unwrap();
        goal
    }

    fn cancel_for(goal: &schema::GoalEnvelope) -> schema::CancelNavigation {
        schema::CancelNavigation {
            session_id: goal.session_id.clone(),
            reason: 1,
            cancel_generation: goal.goal_generation,
            owner_kind: goal.owner_kind,
            parent_request_id: goal.parent_request_id.clone(),
            goal_generation: goal.goal_generation,
            producer_runtime_generation: goal.producer_runtime_generation,
        }
    }

    fn virtual_mask(map: &schema::NavigationMap3D, blocked: bool) -> schema::OccupancyGrid {
        let mut data = vec![0; 1200];
        if blocked {
            for y in 0..30 {
                data[y * 40 + 10] = 100;
            }
        }
        schema::OccupancyGrid {
            header: schema::Header {
                seq: map.committed_map.revision as i32,
                stamp_sec: (map.source_timestamp_ns / 1_000_000_000) as i32,
                stamp_nsec: (map.source_timestamp_ns % 1_000_000_000) as i32,
                frame_id: "map".into(),
            },
            info: schema::MapMetaData {
                resolution: 0.1,
                width: 40,
                height: 30,
                origin: schema::Pose {
                    position: [0.0; 3],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
                ..schema::MapMetaData::default()
            },
            data,
        }
    }

    #[test]
    fn production_fine_columns_survive_geometry_changes_and_failed_transfer() {
        use nuwax_map_chunks::*;
        use std::collections::{BTreeMap, BTreeSet};
        let configured=config();
        let mut cache=SurfaceGeometryCache::new(configured.geometry.clone());
        let mut observed_config = configured.clone();
        observed_config.follower.max_observation_age_sec = Some(1.0);
        let mut observed_cache = SurfaceGeometryCache::configured(&observed_config);
        let mut cells:BTreeSet<[i32;3]>=(0..80).flat_map(|x|(0..60).map(move|y|[x,y,40])).collect();
        let message=|cells:&BTreeSet<[i32;3]>,revision| {
            let mut message=map_message(revision,"fine");message.source_voxel_size=0.05;
            message.cloud.width=0;message.cloud.row_step=0;message.cloud.data.clear();
            let mut blocks=BTreeMap::<[i32;3],Block>::new();
            for &key in cells {blocks.entry(block_key(key)).or_default().insert(local_key(key),Evidence{observed_ns:message.source_timestamp_ns as u64,hits:1});}
            let mut included=BTreeMap::new();
            let refs=blocks.into_iter().map(|(key,block)| {
                let bytes=encode_block(&block).unwrap();let hash=digest(&bytes);included.insert(hash,bytes);
                BlockRef{key,sha256:hash,geometry_sha256:block_geometry_digest(&block),points:block.len() as u32}
            }).collect();
            let root=Manifest {version:1,domain:schema::navigation_map_3d_domain(&message),revision:revision as u64,source_timestamp_ns:message.source_timestamp_ns as u64,voxel_size_m:0.05,blocks:refs}.encode().unwrap();
            message.fine=Some(Transfer{root_sha256:digest(&root),root,included});message
        };
        let first=cache.build(&message(&cells,1)).unwrap();
        let observed_first = observed_cache.build(&message(&cells,1)).unwrap();
        let old_evidence = observed_first.observations().unwrap().get([40,30,40]).unwrap();
        assert_eq!(cache.last_build_stats().unwrap().input_points,cells.len());
        let same=cache.build(&message(&cells,2)).unwrap();assert!(Arc::ptr_eq(&first,&same));
        let second_message = message(&cells, 2);
        let observed_second = observed_cache.build(&second_message).unwrap();
        assert!(observed_first.same_geometry(&observed_second));
        assert!(!Arc::ptr_eq(&observed_first, &observed_second));
        assert_eq!(observed_second.observations().unwrap().get([40,30,40]).unwrap().observed_ns, second_message.source_timestamp_ns as u64);
        assert_eq!(observed_first.observations().unwrap().get([40,30,40]), Some(old_evidence));
        cells.retain(|c|!(40..42).contains(&c[0]) || !(28..34).contains(&c[1]));
        let mut bad=message(&cells,3);
        bad.fine.as_mut().unwrap().included.values_mut().next().unwrap()[0]^=1;
        assert!(cache.build(&bad).is_err());
        assert!(observed_cache.build(&bad).is_err());
        let next=cache.build(&message(&cells,4)).unwrap();
        let observed_next = observed_cache.build(&message(&cells,4)).unwrap();
        assert!(observed_next.observations().unwrap().get([40,30,40]).is_none());
        assert_eq!(observed_first.observations().unwrap().get([40,30,40]), Some(old_evidence));
        let points:Vec<_>=cells.iter().map(|c|((c[0] as f32+0.5)*0.05,(c[1] as f32+0.5)*0.05,(c[2] as f32+0.5)*0.05)).collect();
        let full=nuwax_nav_mls_planner::SurfaceSnapshot::from_points(&points,configured.geometry,0.05).unwrap();
        assert_eq!(next.cell_keys().collect::<BTreeSet<_>>(),full.cell_keys().collect());
        for goal in [[0.8,0.8,2.05],[3.2,2.2,2.05],[2.025,1.525,2.05]] {
            assert_eq!(next.plan([0.55,1.55,2.05],goal).is_ok(),full.plan([0.55,1.55,2.05],goal).is_ok());
        }
        assert!(cache.last_build_stats().unwrap().input_points<cells.len());
        assert!(!Arc::ptr_eq(&first,&next));
    }

    #[test]
    fn production_surface_cache_applies_column_changes_and_clear_discards_baseline() {
        use nuwax_nav_mls_planner::SurfaceBuildKind;
        let configured = config();
        let mut cache = SurfaceGeometryCache::new(configured.geometry.clone());
        let first = prepare_surface_map(map_message(1, "map"), Instant::now(), &configured, &mut cache)
            .snapshot.unwrap();
        assert_eq!(cache.last_build_stats().unwrap().kind, SurfaceBuildKind::Full);
        let next = prepare_surface_map(map_message(2, "map"), Instant::now(), &configured, &mut cache)
            .snapshot.unwrap();
        assert!(Arc::ptr_eq(&first, &next));
        assert_eq!(cache.last_build_stats().unwrap().kind, SurfaceBuildKind::Reused);
        let mut changed = map_message(3, "map");
        for value in [3.55_f32, 0.55, 2.35] { changed.cloud.data.extend_from_slice(&value.to_le_bytes()); }
        changed.cloud.width += 1;
        changed.cloud.row_step += 12;
        let next = prepare_surface_map(changed, Instant::now(), &configured, &mut cache).snapshot.unwrap();
        assert_eq!(cache.last_build_stats().unwrap().kind, SurfaceBuildKind::Incremental);
        assert_eq!(cache.last_build_stats().unwrap().changed_columns, 1);
        assert!(first.locate_support([3.55, 0.55, 2.1]).is_ok());
        assert!(next.locate_support([3.55, 0.55, 2.1]).is_err());
        let mut clear = map_message(4, "map");
        clear.cleared = true;
        clear.cloud = schema::PointCloud2::default();
        assert!(prepare_surface_map(clear, Instant::now(), &configured, &mut cache).snapshot.is_err());
        assert!(cache.last_build_stats().is_none());
        let resumed = prepare_surface_map(map_message(5, "map"), Instant::now(), &configured, &mut cache)
            .snapshot.unwrap();
        assert!(!Arc::ptr_eq(&first, &resumed));
        assert_eq!(cache.last_build_stats().unwrap().kind, SurfaceBuildKind::Full);
    }

    #[test]
    fn production_derived_surface_carries_deferred_frontier_updates() {
        let configured = config();
        let mut cache = SurfaceGeometryCache::new(configured.geometry.clone());
        let mut first_message = map_message(1, "map");
        first_message.virtual_obstacles = Some(virtual_mask(&first_message, false));
        let first = prepare_surface_map(first_message, Instant::now(), &configured, &mut cache)
            .snapshot
            .unwrap();
        first.reachable_frontiers([0.55, 1.55, 2.1], &[]).unwrap();
        let old_stats = first.frontier_cache_stats().unwrap();
        let mut changed = map_message(2, "map");
        changed.virtual_obstacles = Some(virtual_mask(&changed, false));
        for value in [3.55_f32, 0.55, 2.35] {
            changed.cloud.data.extend_from_slice(&value.to_le_bytes());
        }
        changed.cloud.width += 1;
        changed.cloud.row_step += 12;
        let next = prepare_surface_map(changed.clone(), Instant::now(), &configured, &mut cache)
            .snapshot
            .unwrap();
        assert!(next.frontier_cache_stats().is_none());
        let mut fresh_cache = SurfaceGeometryCache::new(configured.geometry.clone());
        let full = prepare_surface_map(changed, Instant::now(), &configured, &mut fresh_cache)
            .snapshot
            .unwrap();
        assert_eq!(
            next.reachable_frontiers([0.55, 1.55, 2.1], &[]).unwrap(),
            full.reachable_frontiers([0.55, 1.55, 2.1], &[]).unwrap()
        );
        assert!(next.frontier_cache_stats().unwrap().incremental);
        assert_eq!(first.frontier_cache_stats(), Some(old_stats));
    }

    #[test]
    fn virtual_mask_is_applied_to_the_surface_and_cache_ignores_only_its_clock() {
        let mut cache = SurfaceGeometryCache::new(config().geometry);
        let configured = config();
        let plain = prepare_surface_map(
            map_message(1, "map"),
            Instant::now(),
            &configured,
            &mut cache,
        );
        let physical = cache.physical_snapshot().unwrap();
        let mut message = map_message(2, "map");
        message.virtual_obstacles = Some(virtual_mask(&message, true));
        let constrained =
            prepare_surface_map(message.clone(), Instant::now(), &configured, &mut cache);
        assert!(plain
            .snapshot
            .as_ref()
            .unwrap()
            .plan([0.55, 1.55, 2.1], [3.05, 1.55, 2.1])
            .is_ok());
        assert!(constrained
            .snapshot
            .as_ref()
            .unwrap()
            .plan([0.55, 1.55, 2.1], [3.05, 1.55, 2.1])
            .is_err());
        assert!(Arc::ptr_eq(&physical, &cache.physical_snapshot().unwrap()));
        message.cloud_revision += 1;
        message.cloud.header.seq += 1;
        message.commit_sequence += 1;
        message.source_timestamp_ns += 1;
        message.cloud.header.stamp_sec = (message.source_timestamp_ns / 1_000_000_000) as i32;
        message.cloud.header.stamp_nsec = (message.source_timestamp_ns % 1_000_000_000) as i32;
        message.virtual_obstacles.as_mut().unwrap().header.stamp_sec =
            message.cloud.header.stamp_sec;
        message
            .virtual_obstacles
            .as_mut()
            .unwrap()
            .header
            .stamp_nsec = message.cloud.header.stamp_nsec;
        let repeated = prepare_surface_map(message, Instant::now(), &configured, &mut cache);
        assert!(Arc::ptr_eq(
            constrained.snapshot.as_ref().unwrap(),
            repeated.snapshot.as_ref().unwrap()
        ));
    }

    #[test]
    fn generated_body_path_does_not_turn_quantization_duplicates_into_vertical_motion() {
        use nuwax_nav_runtime::NavPlanner;
        let message = map_message(1, "map");
        let goal = goal_for(&message, 1, [3.2, 1.55, 2.5]);
        let mut cache = SurfaceGeometryCache::new(config().geometry);
        let configured = config();
        let prepared = prepare_surface_map(message, Instant::now(), &configured, &mut cache);
        let surface = prepared.snapshot.unwrap();
        let mut start = pose_from_schema(&goal.goal).unwrap();
        start.pose.position.x = 0.55;
        let path = body_path(&surface, &start, &goal, &configured).unwrap();
        let mut follower = nuwax_nav_runtime::RouteFollower::new(configured.follower).unwrap();
        follower
            .update_execution_map(nuwax_nav_runtime::ExecutionMap::Surface(surface))
            .unwrap();
        follower.update_odometry(start);
        follower
            .replace_execution_path(
                nuwax_nav_runtime::ExecutionPath::Surface(path.clone()),
                None,
                pose_from_schema(&goal.goal).unwrap(),
            )
            .unwrap();
        let step = follower.step(0.3);
        assert!(
            matches!(step,nuwax_nav_runtime::PlannerStep::Command(twist) if twist!=Twist2::ZERO),
            "真实生成路径必须能启动运动: step={step:?} path={path:?}"
        );
    }

    #[test]
    fn map_order_and_geometry_reuse_preserve_input_identity_and_receipt_age() {
        let first = map_message(1, "first-map");
        let payload = schema::encode_navigation_map_3d_body(&first).unwrap();
        let mut order = SurfaceMapOrder::default();
        assert_eq!(
            order.observe(&map_authority(&first), &payload),
            MapIngress::New
        );
        assert_eq!(
            order.observe(&map_authority(&first), &payload),
            MapIngress::Duplicate
        );
        let mut old = first.clone();
        old.producer_runtime_generation -= 1;
        assert!(matches!(
            order.observe(&map_authority(&old), &payload),
            MapIngress::Rejected(_)
        ));
        let mut mutation = first.clone();
        mutation.cloud.data[0] ^= 1;
        let mutated = schema::encode_navigation_map_3d_body(&mutation).unwrap();
        assert!(matches!(
            order.observe(&map_authority(&mutation), &mutated),
            MapIngress::Invalidated(_)
        ));
        let mut cache = SurfaceGeometryCache::new(config().geometry);
        let received = Instant::now() - Duration::from_secs(2);
        let prepared = prepare_surface_map(first, received, &config(), &mut cache);
        let next = prepare_surface_map(
            map_message(2, "second-map"),
            received,
            &config(),
            &mut cache,
        );
        assert!(Arc::ptr_eq(
            prepared.snapshot.as_ref().unwrap(),
            next.snapshot.as_ref().unwrap()
        ));
        assert_eq!(next.received_at, received);
        assert_eq!(next.authority.committed_map.session_id, "second-map");
    }

    #[tokio::test]
    async fn map_hub_never_coalesces_a_clear_between_live_updates() {
        let hub = SurfaceMapHub::new();
        let mut stream = hub.stream();
        let mut cache = SurfaceGeometryCache::new(config().geometry);
        let mut clear =
            prepare_surface_map(map_message(1, "map"), Instant::now(), &config(), &mut cache);
        clear.snapshot = Err("explicit clear".into());
        hub.publish(clear);
        hub.publish(prepare_surface_map(
            map_message(2, "map"),
            Instant::now(),
            &config(),
            &mut cache,
        ));
        let first = stream.next().await.unwrap().unwrap();
        let second = stream.next().await.unwrap().unwrap();
        assert!(matches!(first,GuardedMapFrame::Surface(update) if update.snapshot.is_err()));
        assert!(matches!(second,GuardedMapFrame::Surface(update) if update.snapshot.is_ok()));
        let other_hub = SurfaceMapHub::new();
        let mut other = other_hub.stream();
        assert!(
            tokio::time::timeout(Duration::from_millis(20), other.next())
                .await
                .is_err(),
            "不同StackRunner的地图通道不能互相污染"
        );
        let mut slow = hub.stream();
        slow.next().await.unwrap().unwrap();
        let update = hub.latest.lock().unwrap().as_ref().unwrap().clone();
        for _ in 0..40 {
            hub.publish(update.clone());
        }
        assert!(
            matches!(slow.next().await, Some(Err(_))),
            "有界队列缺口必须明确失败，不能静默丢失clear"
        );
    }

    struct FeedState {
        map: schema::NavigationMap3D,
        map_enabled: bool,
        integrate: bool,
        position: [f64; 3],
        yaw: f64,
        last_published: Option<schema::NavigationMap3D>,
    }

    struct Harness {
        transport: LcmTransport,
        planner_node: ModuleNode,
        hub: SurfaceMapHub,
        planner: Option<Box<dyn InProcessNode>>,
        executor: Option<Box<dyn InProcessNode>>,
        scheduler: NavSchedulerHandle,
        calls: Arc<Mutex<Vec<String>>>,
        paths: LcmFrameStream,
        feed: Arc<Mutex<FeedState>>,
        feeder: tokio::task::JoinHandle<()>,
    }

    impl Harness {
        async fn start() -> Self {
            Self::start_with_computer(Arc::new(body_path)).await
        }

        async fn start_with_computer(computer: SurfacePathComputer) -> Self {
            Self::start_with_rate(computer,Duration::from_millis(25)).await
        }

        async fn start_with_rate(computer:SurfacePathComputer,feed_period:Duration)->Self {
            use crate::topics::{NavTopicConfig, TopicName};
            let socket = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
            let port = socket.local_addr().unwrap().port();
            drop(socket);
            let url = format!("udpm://239.255.76.67:{port}?ttl=0");
            let transport = LcmTransport::new_real(&url).unwrap();
            let prefix = format!("/sf{port}");
            let mut topics = NavTopicConfig::empty();
            for (index, name) in [
                "navigation_map3d",
                "map_pose",
                "goal_pose",
                "nav_cancel",
                "goal_path",
                "cmd_vel",
                "nav_state",
                "goal_reached",
                "stop_movement",
            ]
            .into_iter()
            .enumerate()
            {
                topics
                    .topics
                    .insert(name.into(), TopicName::new(format!("{prefix}/{index}")));
            }
            let graph = crate::profiles::build_graph(
                crate::NavStackProfile::Mls3D,
                &topics,
                crate::features::NavStackPreset::Mls.to_flags(),
            );
            let planner_node = graph.find("mls_planner").unwrap().clone();
            let scheduler_node = graph.find("surface_nav_scheduler").unwrap().clone();
            let paths = typed_stream(
                &transport,
                &planner_node.outputs[0],
                "nuwax_msgs.PathEnvelope",
            )
            .await
            .unwrap();
            let hub = SurfaceMapHub::new();
            let planner_transport = LcmTransport::new_real(&url).unwrap();
            let planner_hub = hub.clone();
            let planner_spec = planner_node.clone();
            let mut planner = boxed_owner(
                "surface-real-lcm-test-planner",
                Box::new(move |shutdown, ready| {
                    Box::pin(run_surface_planner_with_computer(
                        planner_spec,
                        planner_hub,
                        config(),
                        planner_transport,
                        shutdown,
                        ready,
                        computer,
                    ))
                }),
            );
            planner.start().unwrap();
            let calls = Arc::new(Mutex::new(Vec::new()));
            let motion_calls = Arc::clone(&calls);
            let scheduler_transport = LcmTransport::new_real(&url).unwrap();
            let scheduler_hub = hub.clone();
            let scheduler_spec = scheduler_node.clone();
            let (sender, receiver) = std_mpsc::sync_channel(1);
            let mut executor = boxed_owner(
                "surface-real-lcm-test-executor",
                Box::new(move |shutdown, ready| {
                    Box::pin(run_surface_scheduler_with_transport(
                        scheduler_spec,
                        scheduler_hub,
                        scheduler_transport,
                        shutdown,
                        ready,
                        async move {
                            let local = nuwax_nav_runtime::RouteFollower::new(config().follower)?;
                            let scheduler = nuwax_nav_runtime::NavScheduler::new(
                                RecordingMotion(motion_calls),
                                local,
                                nuwax_nav_runtime::SchedulerConfig::default(),
                            )?;
                            let pair = nuwax_nav_runtime::spawn_scheduler(scheduler);
                            sender.send(pair.0.clone()).unwrap();
                            Ok(pair)
                        },
                    ))
                }),
            );
            executor.start().unwrap();
            let scheduler = receiver.recv_timeout(Duration::from_secs(1)).unwrap();
            let feed = Arc::new(Mutex::new(FeedState {
                map: map_message(1, "first-map"),
                map_enabled: true,
                integrate: false,
                position: [0.55, 1.55, 2.5],
                yaw: 0.0,
                last_published: None,
            }));
            let state = Arc::clone(&feed);
            let feed_transport = transport.clone();
            let feed_node = planner_node.clone();
            let control = scheduler.clone();
            let feeder = tokio::spawn(async move {
                let mut ticker = tokio::time::interval(feed_period);
                let mut count = 0;
                loop {
                    ticker.tick().await;
                    count += 1;
                    let now = current_unix_timestamp_ns().unwrap();
                    let (pose, map) = {
                        let mut state = state.lock().unwrap();
                        if state.integrate {
                            if let Some(twist) = control.status().last_applied_twist {
                                let yaw = state.yaw;
                                state.position[0] += twist.linear.x * yaw.cos() * 0.025;
                                state.position[1] += twist.linear.x * yaw.sin() * 0.025;
                                state.yaw += twist.angular_z * 0.025;
                            }
                        }
                        let q = Quaternion::from_yaw(state.yaw);
                        let pose = schema::PoseStamped {
                            header: schema::Header {
                                seq: count,
                                stamp_sec: (now / 1_000_000_000) as i32,
                                stamp_nsec: (now % 1_000_000_000) as i32,
                                frame_id: "map".into(),
                            },
                            pose: schema::Pose {
                                position: state.position,
                                orientation_xyzw: [q.x, q.y, q.z, q.w],
                            },
                        };
                        let map = if state.map_enabled && count % 4 == 1 {
                            state.map.cloud_revision += 1;
                            state.map.commit_sequence += 1;
                            state.map.source_timestamp_ns = now;
                            state.map.cloud.header.seq = state.map.cloud_revision as i32;
                            state.map.cloud.header.stamp_sec = (now / 1_000_000_000) as i32;
                            state.map.cloud.header.stamp_nsec = (now % 1_000_000_000) as i32;
                            let map_revision = state.map.committed_map.revision as i32;
                            if let Some(mask) = state.map.virtual_obstacles.as_mut() {
                                mask.header.seq = map_revision;
                                mask.header.stamp_sec = (now / 1_000_000_000) as i32;
                                mask.header.stamp_nsec = (now % 1_000_000_000) as i32;
                            }
                            let map = state.map.clone();
                            state.last_published = Some(map.clone());
                            Some(map)
                        } else {
                            None
                        };
                        (pose, map)
                    };
                    publish_body(
                        &feed_transport,
                        &feed_node.inputs[1],
                        "geometry_msgs.PoseStamped",
                        schema::encode_pose_stamped_body(&pose).unwrap(),
                    )
                    .await
                    .unwrap();
                    if let Some(map) = map {
                        publish_body(
                            &feed_transport,
                            &feed_node.inputs[0],
                            "nuwax_msgs.NavigationMap3D",
                            schema::encode_navigation_map_3d_body(&map).unwrap(),
                        )
                        .await
                        .unwrap();
                    }
                }
            });
            Self {
                transport,
                planner_node,
                hub,
                planner: Some(planner),
                executor: Some(executor),
                scheduler,
                calls,
                paths,
                feed,
                feeder,
            }
        }

        async fn wait_ready(&self) {
            tokio::time::timeout(Duration::from_secs(3), async {
                while !self.scheduler.status().planner_inputs_fresh {
                    tokio::time::sleep(Duration::from_millis(10)).await;
                }
            })
            .await
            .expect("真实地图/机体输入未进入调度器");
        }

        async fn wait_controlling(&self) {
            tokio::time::timeout(Duration::from_secs(3), async {
                loop {
                    let state = self.scheduler.status();
                    if state.phase == SchedulerPhase::Controlling
                        && state.applied_commands > 0
                        && state
                            .last_applied_twist
                            .is_some_and(|twist| twist != Twist2::ZERO)
                    {
                        break;
                    }
                    tokio::time::sleep(Duration::from_millis(10)).await;
                }
            })
            .await
            .unwrap_or_else(|_| panic!("完整三维路径未实际执行: {:?}", self.scheduler.status()));
        }

        async fn wait_phase(&self, expected: SchedulerPhase) {
            tokio::time::timeout(Duration::from_secs(3), async {
                while self.scheduler.status().phase != expected {
                    tokio::time::sleep(Duration::from_millis(10)).await;
                }
            })
            .await
            .unwrap_or_else(|_| panic!("等待{expected:?}失败: {:?}", self.scheduler.status()));
        }

        async fn goal(&self, goal: &schema::GoalEnvelope) {
            publish_body(
                &self.transport,
                &self.planner_node.inputs[2],
                "nuwax_msgs.GoalEnvelope",
                schema::encode_goal_envelope_body(goal).unwrap(),
            )
            .await
            .unwrap();
        }
        async fn map(&self, map: &schema::NavigationMap3D) {
            publish_body(
                &self.transport,
                &self.planner_node.inputs[0],
                "nuwax_msgs.NavigationMap3D",
                schema::encode_navigation_map_3d_body(map).unwrap(),
            )
            .await
            .unwrap();
        }
        async fn cancel(&self, goal: &schema::GoalEnvelope) {
            publish_body(
                &self.transport,
                &self.planner_node.inputs[3],
                "nuwax_msgs.CancelNavigation",
                schema::encode_cancel_navigation_body(&cancel_for(goal)).unwrap(),
            )
            .await
            .unwrap();
        }
        async fn path_for(&mut self, goal: &schema::GoalEnvelope) -> schema::PathEnvelope {
            tokio::time::timeout(Duration::from_secs(4), async {
                loop {
                    let frame = self.paths.next().await.unwrap().unwrap();
                    let path = schema::decode_path_envelope_body(&frame.payload).unwrap();
                    if path.goal_payload_sha256 == goal.payload_sha256 {
                        return path;
                    }
                }
            })
            .await
            .unwrap_or_else(|_| {
                panic!(
                    "真实planner没有发布路径 status={:?}",
                    self.scheduler.status()
                )
            })
        }
        async fn terminal(&self, expected: TerminalOutcome) {
            tokio::time::timeout(Duration::from_secs(4), async {
                while self.scheduler.status().last_terminal != Some(expected) {
                    tokio::time::sleep(Duration::from_millis(10)).await;
                }
            })
            .await
            .unwrap_or_else(|_| panic!("等待终态{expected:?}失败: {:?}", self.scheduler.status()));
        }
        async fn freeze_map(&self) -> schema::NavigationMap3D {
            self.feed.lock().unwrap().map_enabled = false;
            let last = self.feed.lock().unwrap().last_published.clone().unwrap();
            tokio::time::timeout(Duration::from_secs(1), async {
                while self
                    .hub
                    .latest
                    .lock()
                    .unwrap()
                    .as_ref()
                    .map(|update| update.authority.cloud_revision)
                    != Some(last.cloud_revision)
                {
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
            })
            .await
            .unwrap();
            last
        }
        async fn shutdown(&mut self) {
            self.feeder.abort();
            let _ = (&mut self.feeder).await;
            if let Some(mut executor) = self.executor.take() {
                executor.shutdown().unwrap();
                assert!(!executor.is_running().unwrap());
            }
            if let Some(mut planner) = self.planner.take() {
                planner.shutdown().unwrap();
                assert!(!planner.is_running().unwrap());
            }
        }
    }
    impl Drop for Harness {
        fn drop(&mut self) {
            self.feeder.abort();
            if let Some(mut owner) = self.executor.take() {
                let _ = owner.shutdown();
            }
            if let Some(mut owner) = self.planner.take() {
                let _ = owner.shutdown();
            }
        }
    }

    #[tokio::test]
    async fn continuous_inputs_do_not_starve_path_generation() {
        let mut harness=Harness::start_with_rate(Arc::new(body_path),Duration::from_micros(100)).await;
        harness.wait_ready().await;
        let goal=goal_for(&harness.feed.lock().unwrap().map,1,[1.55,1.55,2.5]);
        let started=Instant::now();harness.goal(&goal).await;
        tokio::time::timeout(Duration::from_millis(500),harness.path_for(&goal)).await
            .expect("连续输入不能阻止路径生成");
        eprintln!("持续输入到路径交付 {:?}",started.elapsed());
        harness.cancel(&goal).await;harness.terminal(TerminalOutcome::Cancelled).await;
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn fresh_geometry_delivers_quick_path_without_waiting_for_old_search() {
        use std::sync::atomic::{AtomicBool,AtomicUsize,Ordering as Order};
        let entered=Arc::new(AtomicBool::new(false));let retired=Arc::new(AtomicBool::new(false));
        let calls=Arc::new(AtomicUsize::new(0));
        let (a,b,c)=(Arc::clone(&entered),Arc::clone(&retired),Arc::clone(&calls));
        let computer:SurfacePathComputer=Arc::new(move |map,pose,goal,config| {
            if c.fetch_add(1,Order::SeqCst)==0 {
                a.store(true,Order::SeqCst);
                while !config.planning_cancelled.as_ref().unwrap().load(Order::Relaxed) {
                    std::thread::sleep(Duration::from_millis(1));
                }
                b.store(true,Order::SeqCst);
                return Err(module_error("test old search","已被新图路线退休"));
            }
            body_path(map,pose,goal,config)
        });
        let mut harness=Harness::start_with_computer(computer).await;
        harness.wait_ready().await;
        let goal=goal_for(&harness.feed.lock().unwrap().map,1,[3.2,1.55,2.5]);
        harness.goal(&goal).await;
        tokio::time::timeout(Duration::from_secs(2),async {
            while !entered.load(Order::SeqCst) {tokio::time::sleep(Duration::from_millis(5)).await;}
        }).await.unwrap();
        let started=Instant::now();
        {
            let mut feed=harness.feed.lock().unwrap();
            for value in [3.55_f32,0.55,2.35] {feed.map.cloud.data.extend_from_slice(&value.to_le_bytes());}
            feed.map.cloud.width+=1;feed.map.cloud.row_step+=12;
        }
        let path=tokio::time::timeout(Duration::from_secs(2),harness.path_for(&goal)).await
            .expect("新图可执行路线不得等待旧搜索");
        eprintln!("新图跨过慢搜索交付路径：{:?}",started.elapsed());
        assert!(!path.path.poses.is_empty());
        tokio::time::timeout(Duration::from_secs(1),async {
            while !retired.load(Order::SeqCst) {tokio::time::sleep(Duration::from_millis(5)).await;}
        }).await.expect("旧搜索必须收到明确取消");
        harness.wait_controlling().await;
        assert_eq!(harness.scheduler.status().path_revision,path.path_revision as u64);
        harness.cancel(&goal).await;harness.terminal(TerminalOutcome::Cancelled).await;
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn failed_global_search_is_rate_limited_and_never_delivers_a_prefix() {
        use std::sync::atomic::{AtomicUsize, Ordering};
        let calls = Arc::new(AtomicUsize::new(0));
        let counter = Arc::clone(&calls);
        let computer: SurfacePathComputer = Arc::new(move |_, _, _, _| {
            counter.fetch_add(1, Ordering::SeqCst);
            Err(module_error("test global search", "目标没有完整路线"))
        });
        let mut harness = Harness::start_with_computer(computer).await;
        harness.wait_ready().await;
        let goal = goal_for(&harness.feed.lock().unwrap().map, 1, [8.0, 1.55, 2.5]);
        harness.goal(&goal).await;
        assert!(tokio::time::timeout(Duration::from_millis(1300), harness.paths.next()).await.is_err(),
            "完整路线缺失时不能发布近端短段");
        assert_eq!(calls.load(Ordering::SeqCst), 1, "同图无路不能持续占用处理器重试");
        harness.feed.lock().unwrap().position[0] += 0.1;
        tokio::time::timeout(Duration::from_secs(2), async {
            while calls.load(Ordering::SeqCst) < 2 { tokio::time::sleep(Duration::from_millis(20)).await; }
        }).await.expect("实际位姿改变后仍保留重试能力");
        harness.cancel(&goal).await;
        harness.terminal(TerminalOutcome::Cancelled).await;
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn blocked_planning_keeps_inputs_fresh_and_cancellation_retires_late_result() {
        use std::sync::atomic::{AtomicBool,Ordering as AtomicOrdering};
        let entered=Arc::new(AtomicBool::new(false));
        let release=Arc::new(AtomicBool::new(false));
        let observed_cancel=Arc::new(AtomicBool::new(false));
        let cancelled=Arc::clone(&observed_cancel);
        let started=Arc::clone(&entered);let resume=Arc::clone(&release);
        let computer:SurfacePathComputer=Arc::new(move |map,pose,goal,config| {
            if !started.swap(true,AtomicOrdering::SeqCst) {
                let deadline=std::time::Instant::now()+Duration::from_secs(2);
                while !resume.load(AtomicOrdering::SeqCst) && std::time::Instant::now()<deadline {
                    if config.planning_cancelled.as_ref().is_some_and(|flag|flag.load(AtomicOrdering::Relaxed)) {
                        cancelled.store(true,AtomicOrdering::SeqCst);
                        break;
                    }
                    std::thread::sleep(Duration::from_millis(5));
                }
            }
            body_path(map,pose,goal,config)
        });
        let mut harness=Harness::start_with_computer(computer).await;
        harness.wait_ready().await;
        let first=goal_for(&harness.feed.lock().unwrap().map,1,[1.55,1.55,2.5]);
        harness.goal(&first).await;
        tokio::time::timeout(Duration::from_secs(2),async {
            while !entered.load(AtomicOrdering::SeqCst) { tokio::time::sleep(Duration::from_millis(5)).await; }
        }).await.expect("路径计算未启动");
        tokio::time::sleep(Duration::from_millis(650)).await;
        assert!(harness.scheduler.status().planner_inputs_fresh,"长计算不能令持续输入的地图超时");
        // 同图不再重复启动快速搜索；全局仍在计算时取消，不能先等待它结束。
        assert!(harness.paths.next().now_or_never().is_none());
        harness.cancel(&first).await;
        tokio::time::timeout(Duration::from_millis(300),harness.terminal(TerminalOutcome::Cancelled))
            .await.expect("取消被长计算阻塞");
        tokio::time::timeout(Duration::from_millis(300),async {
            while !observed_cancel.load(AtomicOrdering::SeqCst) {tokio::time::sleep(Duration::from_millis(5)).await;}
        }).await.expect("后台计算未接收取消信号");
        release.store(true,AtomicOrdering::SeqCst);
        assert!(tokio::time::timeout(Duration::from_millis(150),harness.paths.next()).await.is_err(),
            "被取消目标的迟到路径不能发布");
        let second=goal_for(&harness.feed.lock().unwrap().map,2,[1.55,1.55,2.5]);
        harness.goal(&second).await;
        harness.path_for(&second).await;
        harness.wait_controlling().await;
        harness.cancel(&second).await;
        harness.terminal(TerminalOutcome::Cancelled).await;
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn real_lcm_surface_chain_preserves_height_replans_and_retires_completed_goals() {
        let mut harness = Harness::start().await;
        let mut states = typed_stream(
            &harness.transport,
            &harness.planner_node.inputs[4],
            "nuwax_msgs.NavigationState",
        )
        .await
        .unwrap();
        harness.wait_ready().await;
        let first = goal_for(&harness.feed.lock().unwrap().map, 1, [1.55, 1.55, 2.5]);
        harness.goal(&first).await;
        let path = harness.path_for(&first).await;
        assert!(path
            .path
            .poses
            .iter()
            .all(|pose| (pose.pose.position[2] - 2.5).abs() < 0.11));
        assert_eq!(path.path.poses.last().unwrap().pose, first.goal.pose);
        harness.goal(&first).await;
        harness.wait_controlling().await;
        // 重复目标和新鲜度更新均不产生新路线，避免反复安装网格接续段。
        assert!(tokio::time::timeout(Duration::from_millis(250), harness.paths.next()).await.is_err());
        assert_eq!(harness.scheduler.status().path_revision, u64::try_from(path.path_revision).unwrap());
        // 新路线安装后迟到的旧版等待帧不得重新发布相同路线。
        let mut old_wait = tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let frame=states.next().await.unwrap().unwrap();
                let state=schema::decode_navigation_state_body(&frame.payload).unwrap();
                if state_matches_goal(&state,&first) && state.state==2 {break state;}
            }
        }).await.unwrap();
        old_wait.state=1;
        old_wait.message="当前路径请求重规划".into();
        publish_body(&harness.transport,&harness.planner_node.inputs[4],
            "nuwax_msgs.NavigationState",schema::encode_navigation_state_body(&old_wait).unwrap()).await.unwrap();
        let replacement=harness.path_for(&first).await;
        assert!(replacement.path_revision>path.path_revision);
        harness.wait_controlling().await;
        old_wait.path_revision=path.path_revision;
        old_wait.message="迟到的旧路径等待".into();
        publish_body(&harness.transport,&harness.planner_node.inputs[4],
            "nuwax_msgs.NavigationState",schema::encode_navigation_state_body(&old_wait).unwrap()).await.unwrap();
        assert!(tokio::time::timeout(Duration::from_millis(250),harness.paths.next()).await.is_err(),
            "旧版等待不得打断最新路径");
        harness.feed.lock().unwrap().integrate = true;
        harness.terminal(TerminalOutcome::Succeeded).await;
        // 调度器内部先完成停车，远端规划器只能依赖随后实际发布的带身份终态。
        let published_terminal = tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let frame = states.next().await.unwrap().unwrap();
                let state = schema::decode_navigation_state_body(&frame.payload).unwrap();
                if state_matches_goal(&state, &first) && state.state == 3 {
                    assert!(!state.zero_ack_id.is_empty());
                    assert!(state.zero_applied_monotonic_ns > 0);
                    break state;
                }
            }
        })
        .await
        .expect("真实执行拥有方必须发布原目标的成功及停车证明");
        let terminal_observed_ns =
            published_terminal.stamp_sec * 1_000_000_000 + i64::from(published_terminal.stamp_nsec);
        assert!(harness.scheduler.status().last_zero_ack_id.is_some());
        while harness.paths.next().now_or_never().is_some() {}
        harness.goal(&first).await;
        if let Ok(Some(frame)) =
            tokio::time::timeout(Duration::from_millis(250), harness.paths.next()).await
        {
            let path = schema::decode_path_envelope_body(&frame.unwrap().payload).unwrap();
            panic!(
                "终态后收到路径：sealed={} terminal_observed={} delta_ns={} revision={}",
                path.source_timestamp_ns,
                terminal_observed_ns,
                path.source_timestamp_ns - terminal_observed_ns,
                path.path_revision
            );
        }
        let next = goal_for(&harness.feed.lock().unwrap().map, 2, [3.2, 1.55, 2.5]);
        harness.feed.lock().unwrap().integrate = false;
        harness.goal(&next).await;
        harness.path_for(&next).await;
        harness.cancel(&next).await;
        harness.terminal(TerminalOutcome::Cancelled).await;
        assert!(harness
            .calls
            .lock()
            .unwrap()
            .iter()
            .any(|call| call.starts_with("zero:")));
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn pending_surface_goal_remains_cancellable_after_map_build_failure() {
        for awaiting_build in [true, false] {
        let mut harness = Harness::start().await;
        harness.wait_ready().await;
        let mut future_map = harness.freeze_map().await;
        future_map.committed_map.revision += 100;
        future_map.cloud_revision += 100;
        let goal = goal_for(&future_map, 1, [1.55, 1.55, 2.5]);
        harness.goal(&goal).await;
        tokio::time::sleep(Duration::from_millis(80)).await;
        assert!(harness.calls.lock().unwrap().is_empty());
        // 精细地图补取失败不能删除已经受理、尚未取得运动租约的目标身份。
        harness.hub.publish(PreparedSurfaceMap {
            authority: map_authority(&future_map),
            source_timestamp_ns: future_map.source_timestamp_ns,
            received_at: Instant::now(),
            awaiting_build,
            snapshot: Err("模拟精细地图补取失败".into()),
        });
        tokio::time::sleep(Duration::from_millis(80)).await;
        {
            let mut feed = harness.feed.lock().unwrap();
            feed.map = future_map;
            feed.map_enabled = true;
        }
        harness.wait_ready().await;
        if awaiting_build {
            harness.wait_controlling().await;
        } else {
            tokio::time::sleep(Duration::from_millis(120)).await;
            assert!(harness.calls.lock().unwrap().is_empty(), "已撤销目标不能随有效地图恢复执行");
        }
        harness.cancel(&goal).await;
        harness.terminal(TerminalOutcome::Cancelled).await;
        assert!(harness.calls.lock().unwrap().iter().any(|call| call.starts_with("zero:")));
        harness.feed.lock().unwrap().map_enabled = true;
        harness.wait_ready().await;
        let next = goal_for(&harness.feed.lock().unwrap().map, 2, [1.55, 1.55, 2.5]);
        harness.goal(&next).await;
        harness.path_for(&next).await;
        harness.cancel(&next).await;
        harness.terminal(TerminalOutcome::Cancelled).await;
        harness.shutdown().await;
        }
    }

    #[tokio::test]
    async fn distant_route_changes_wait_for_local_replan_from_actual_pose() {
        let starts = Arc::new(Mutex::new(Vec::new()));
        let captured = Arc::clone(&starts);
        let computer: SurfacePathComputer = Arc::new(move |map, pose, goal, config| {
            captured.lock().unwrap().push(pose.pose.position.x);
            body_path(map, pose, goal, config)
        });
        let mut harness = Harness::start_with_computer(computer).await;
        harness.wait_ready().await;
        let goal = goal_for(&harness.feed.lock().unwrap().map, 1, [3.2, 1.55, 2.5]);
        harness.goal(&goal).await;
        harness.path_for(&goal).await;
        harness.wait_controlling().await;
        {
            let mut feed = harness.feed.lock().unwrap();
            // 完整路线远端被真实障碍占据，当前局部前方仍可持续前进。
            for value in [2.55_f32, 1.55, 2.35] { feed.map.cloud.data.extend_from_slice(&value.to_le_bytes()); }
            feed.map.cloud.width += 1;
            feed.map.cloud.row_step += 12;
        }
        assert!(tokio::time::timeout(Duration::from_millis(450), harness.paths.next()).await.is_err());
        assert_eq!(starts.lock().unwrap().len(), 1, "新图不能触发全程重复检查和搜索");
        harness.feed.lock().unwrap().position[0] = 1.85;
        tokio::time::timeout(Duration::from_secs(2), async {
            while starts.lock().unwrap().len() < 2 { tokio::time::sleep(Duration::from_millis(20)).await; }
        }).await.expect("局部接续失效应触发完整重规划");
        assert!(starts.lock().unwrap()[1] > 1.8, "重算必须从实际位姿开始");
        harness.cancel(&goal).await;
        harness.terminal(TerminalOutcome::Cancelled).await;
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn new_geometry_off_the_route_preserves_active_surface_path() {
        let mut harness = Harness::start().await;
        harness.wait_ready().await;
        let goal = goal_for(&harness.feed.lock().unwrap().map, 1, [1.55, 1.55, 2.5]);
        harness.goal(&goal).await;
        let path = harness.path_for(&goal).await;
        harness.wait_controlling().await;
        {
            let mut feed = harness.feed.lock().unwrap();
            // 路线外新增真实占据，使几何缓存变化，但不阻断原路径。
            for value in [3.55_f32, 0.55, 2.35] {
                feed.map.cloud.data.extend_from_slice(&value.to_le_bytes());
            }
            feed.map.cloud.width += 1;
            feed.map.cloud.row_step += 12;
        }
        assert!(tokio::time::timeout(Duration::from_millis(500), harness.paths.next()).await.is_err());
        let state = harness.scheduler.status();
        assert!(state.planner_inputs_fresh);
        assert_eq!(state.path_revision, u64::try_from(path.path_revision).unwrap());
        assert!(harness.hub.latest.lock().unwrap().as_ref().unwrap().snapshot.as_ref().unwrap()
            .locate_support([3.55, 0.55, 2.05]).is_err(), "路线外新增占据必须确实进入执行地图");
        harness.cancel(&goal).await;
        harness.terminal(TerminalOutcome::Cancelled).await;
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn real_lcm_map_mutation_fences_motion_and_old_replays_do_not_reactivate() {
        let mut harness = Harness::start().await;
        harness.wait_ready().await;
        let goal = goal_for(&harness.feed.lock().unwrap().map, 1, [3.2, 1.55, 2.5]);
        harness.goal(&goal).await;
        harness.path_for(&goal).await;
        harness.wait_controlling().await;
        let latest = harness.freeze_map().await;
        let mut retired = latest.clone();
        retired.producer_runtime_generation -= 1;
        harness.map(&retired).await;
        let mut old = latest.clone();
        old.cloud_revision -= 1;
        old.cloud.header.seq -= 1;
        harness.map(&old).await;
        tokio::time::sleep(Duration::from_millis(30)).await;
        assert!(
            harness.scheduler.status().last_terminal.is_none(),
            "旧producer/云修订只拒绝，不能污染当前地图"
        );
        let mut mutation = latest.clone();
        mutation.cloud.data[0] ^= 1;
        harness.map(&mutation).await;
        harness.terminal(TerminalOutcome::SafetyFailed).await;
        assert!(harness.scheduler.status().message.contains("mutation"));
        assert_eq!(
            harness.scheduler.status().last_applied_twist,
            Some(Twist2::ZERO)
        );
        harness.feed.lock().unwrap().map_enabled = true;
        harness.wait_ready().await;
        while harness.paths.next().now_or_never().is_some() {}
        harness.goal(&goal).await;
        assert!(
            tokio::time::timeout(Duration::from_millis(250), harness.paths.next())
                .await
                .is_err()
        );
        assert_eq!(
            harness
                .calls
                .lock()
                .unwrap()
                .iter()
                .filter(|call| call.starts_with("acquire:"))
                .count(),
            1
        );
        let next = goal_for(&harness.feed.lock().unwrap().map, 2, [3.0, 1.55, 2.5]);
        harness.goal(&next).await;
        harness.path_for(&next).await;
        harness.cancel(&next).await;
        harness.terminal(TerminalOutcome::Cancelled).await;
        harness.shutdown().await;
    }
    #[tokio::test]
    async fn real_lcm_future_map_goal_clear_and_owner_shutdown_remain_isolated() {
        use nuwax_nav_runtime::{SchedulerPhase, TerminalOutcome};
        let mut first = Harness::start().await;
        first.wait_ready().await;
        let mut second = Harness::start().await;
        second.wait_ready().await;
        let other_goal = goal_for(&second.feed.lock().unwrap().map, 1, [3.2, 1.55, 2.5]);
        second.goal(&other_goal).await;
        second.path_for(&other_goal).await;
        second.wait_controlling().await;

        // 新域目标可先于体积较大的地图消息抵达，必须等待对应地图而不是丢弃。
        let old = first.freeze_map().await;
        let mut next_map = old.clone();
        next_map.committed_map.session_id = "next-map".into();
        next_map.map_frame_generation += 1;
        let pending = goal_for(&next_map, 1, [3.1, 1.55, 2.5]);
        first.goal(&pending).await;
        tokio::time::sleep(Duration::from_millis(40)).await;
        assert!(
            first.calls.lock().unwrap().is_empty(),
            "未来域目标不能提前借用旧地图取得租约"
        );
        {
            let mut feed = first.feed.lock().unwrap();
            feed.map = next_map;
            feed.map_enabled = true;
        }
        first.path_for(&pending).await;
        first.wait_controlling().await;

        // 同域的新点云删除前方支撑，实际执行先零速等待；恢复后连续重规划。
        let original = first.feed.lock().unwrap().map.cloud.clone();
        {
            let mut feed = first.feed.lock().unwrap();
            let data: Vec<_> = feed
                .map
                .cloud
                .data
                .chunks_exact(12)
                .filter(|point| {
                    let x = f32::from_le_bytes(point[0..4].try_into().unwrap());
                    !(0.9..1.3).contains(&x)
                })
                .flatten()
                .copied()
                .collect();
            feed.map.cloud.width = (data.len() / 12) as i32;
            feed.map.cloud.row_step = data.len() as i32;
            feed.map.cloud.data = data;
        }
        first.wait_phase(SchedulerPhase::AwaitingPath).await;
        assert_eq!(
            first.scheduler.status().last_applied_twist,
            Some(Twist2::ZERO)
        );
        assert!(first.scheduler.status().last_terminal.is_none());
        {
            let mut feed = first.feed.lock().unwrap();
            feed.map.cloud = original;
        }
        first.wait_controlling().await;

        // 虚拟禁区与物理点云分开，额外限制同样必须实际停车并在解除后重规划。
        {
            let mut feed = first.feed.lock().unwrap();
            feed.map.committed_map.revision += 1;
            feed.map.committed_map.payload_sha256 = [0x32; 32];
            feed.map.virtual_obstacles = Some(virtual_mask(&feed.map, true));
        }
        first.wait_phase(SchedulerPhase::AwaitingPath).await;
        assert_eq!(
            first.scheduler.status().last_applied_twist,
            Some(Twist2::ZERO)
        );
        {
            let mut feed = first.feed.lock().unwrap();
            feed.map.committed_map.revision += 1;
            feed.map.committed_map.payload_sha256 = [0x33; 32];
            feed.map.virtual_obstacles = None;
        }
        first.wait_controlling().await;

        let mut clear = first.freeze_map().await;
        clear.cloud_revision += 1;
        clear.commit_sequence += 1;
        clear.cleared = true;
        clear.virtual_obstacles = None;
        let stamp = current_unix_timestamp_ns().unwrap();
        clear.source_timestamp_ns = stamp;
        clear.cloud = schema::PointCloud2 {
            header: schema::Header {
                seq: clear.cloud_revision as i32,
                stamp_sec: (stamp / 1_000_000_000) as i32,
                stamp_nsec: (stamp % 1_000_000_000) as i32,
                frame_id: "map".into(),
            },
            height: 1,
            ..schema::PointCloud2::default()
        };
        first.map(&clear).await;
        first.terminal(TerminalOutcome::SafetyFailed).await;
        assert!(first.scheduler.status().message.contains("cleared"));
        {
            let mut feed = first.feed.lock().unwrap();
            feed.map.cloud_revision = clear.cloud_revision;
            feed.map.commit_sequence = clear.commit_sequence;
            feed.map_enabled = true;
        }
        first.wait_ready().await;
        while first.paths.next().now_or_never().is_some() {}
        first.goal(&pending).await;
        assert!(
            tokio::time::timeout(Duration::from_millis(200), first.paths.next())
                .await
                .is_err(),
            "clear后的旧goal不得复活"
        );
        first.shutdown().await;
        assert!(second.executor.as_mut().unwrap().is_running().unwrap());
        assert!(second.planner.as_mut().unwrap().is_running().unwrap());
        second.wait_controlling().await;
        assert_eq!(
            second
                .calls
                .lock()
                .unwrap()
                .iter()
                .filter(|call| call.starts_with("zero:"))
                .count(),
            0,
            "一个拥有方的回收不能停止另一个拥有方"
        );
        second.cancel(&other_goal).await;
        second.terminal(TerminalOutcome::Cancelled).await;
        second.shutdown().await;
    }
}
