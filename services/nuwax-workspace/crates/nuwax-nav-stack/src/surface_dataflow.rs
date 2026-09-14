//! MLS生产拥有方：共享一次构建的表面，沿既有目标、路径与运动守卫推进。

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
}

impl SurfaceMapHub {
    fn new() -> Self {
        Self {
            latest: Arc::new(Mutex::new(None)),
            events: tokio::sync::broadcast::channel(32).0,
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
    let source_max_age = configured_pose_source_max_age()?;
    let goals = typed_stream(&transport, &node.inputs[0], "nuwax_msgs.GoalEnvelope").await?;
    let poses = typed_stream(&transport, &node.inputs[1], "geometry_msgs.PoseStamped").await?;
    let paths = typed_stream(&transport, &node.inputs[2], "nuwax_msgs.PathEnvelope").await?;
    let cancels = typed_stream(&transport, &node.inputs[4], "nuwax_msgs.CancelNavigation").await?;
    let (scheduler, task) = create_scheduler
        .await
        .map_err(|error| module_error("surface scheduler connect", error))?;
    let status = scheduler.subscribe_status();
    signal_ready(ready)?;
    let loop_future = run_guarded_nav_loop(
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
        Some(source_max_age),
        false,
    );
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
        awaiting_build: false,
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
    let start_quat = [
        start.pose.orientation.x,
        start.pose.orientation.y,
        start.pose.orientation.z,
        start.pose.orientation.w,
    ];
    if surface.body_envelope().is_some() {
        let p=start.pose.position;
        let boundary:Vec<_>=goal.boundary_points.iter().map(|p|[p[0],p[1]]).collect();
        let options=goal.options;
        let route=crate::surface_oriented::plan(surface,([p.x,p.y,p.z],start_quat),(goal.goal.pose.position,goal.goal.pose.orientation_xyzw),options,config.follower.motion_limits.as_ref(),&boundary,30_000).map_err(|e|module_error("surface body planning",e))?;
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

/// 新几何只在既有路线失效时触发重规划；局部执行仍持续验证当前支撑与扫掠。
fn body_path_remains_traversable(
    surface: &SurfaceSnapshot,
    path: &Path3D,
    config: &SurfaceNavigationConfig,
) -> bool {
    if surface.body_envelope().is_some() {
        let poses:Vec<_>=path.poses.iter().map(|p| {let q=p.pose.orientation;([p.pose.position.x,p.pose.position.y,p.pose.position.z],[q.x,q.y,q.z,q.w])}).collect();
        return poses.iter().all(|p|surface.validate_body_pose(p.0,p.1).is_ok())&&poses.windows(2).all(|w|surface.validate_body_motion(w[0],w[1],&[]).is_ok());
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
async fn run_surface_planner_with_transport(
    node: ModuleNode,
    hub: SurfaceMapHub,
    config: SurfaceNavigationConfig,
    transport: LcmTransport,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let source_max_age = configured_pose_source_max_age()?;
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
    let mut pending_map: Option<(schema::NavigationMap3D, Instant, u64)> = None;
    let mut accepted_domain: Option<GuardedMapAuthority> = None;
    let mut map_epoch = 0_u64;
    let mut latest_map: Option<PreparedSurfaceMap> = None;
    let mut pose: Option<(PoseStamped, Instant, tokio::time::Instant)> = None;
    let mut goal: Option<schema::GoalEnvelope> = None;
    let mut newest_goal: Option<schema::GoalEnvelope> = None;
    let mut retired_goals = VecDeque::<[u8; 32]>::new();
    let generation = saturating_i64(runtime_generation());
    let mut path_revision = 0_i64;
    let mut last_path: Option<Path3D> = None;
    let mut path_surface: Option<std::sync::Weak<SurfaceSnapshot>> = None;
    let mut needs_plan = false;
    let mut execution_admitted = false;
    let mut diagnostics = InputDiagnosticLimiter::default();
    let mut ticker = tokio::time::interval(Duration::from_millis(100));
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    signal_ready(ready)?;
    let terminal: NavStackResult<()> = async {
    loop {
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
            biased;
            _ = &mut shutdown => break,
            frame = cancels.next() => {
                let frame = required_frame(frame, "surface planner cancellation")?;
                let cancel = match schema::decode_cancel_navigation_body(&frame.payload) {
                    Ok(cancel) => cancel,
                    Err(error) => { diagnostics.reject("surface cancel", &error.to_string()); continue; }
                };
                if goal.as_ref().is_some_and(|active| validate_cancel_goal_correlation(&cancel, active).is_ok()) {
                    retire_goal(&mut goal, &mut retired_goals);
                    last_path = None; needs_plan = false; execution_admitted = false;
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
                    last_path = None; needs_plan = false; execution_admitted = false;
                } else if matches!(state.state, 1 | 2) && state.sink_ready != 0 {
                    execution_admitted = true;
                    if state.state == 1 && (state.path_revision > 0 || last_path.is_none()) { needs_plan = true; }
                }
            }
            computed = computations.join_next(), if !computations.is_empty() => {
                let (epoch, update) = computed.expect("已有表面任务").map_err(|e| NavStackError::Internal(e.to_string()))?;
                let update = update.map_err(NavStackError::Internal)?;
                if epoch != map_epoch || latest_map.as_ref().is_some_and(|old| old.authority.same_domain(&update.authority)
                    && (old.authority.cloud_revision > update.authority.cloud_revision || (old.authority.cloud_revision == update.authority.cloud_revision && old.snapshot.is_ok()))) { continue; }
                if update.snapshot.is_err() {
                    retire_goal(&mut goal, &mut retired_goals);
                    pose = None; last_path = None; execution_admitted = false;
                }
                hub.publish(update.clone()); latest_map = Some(update);
                needs_plan = goal.is_some() && (needs_plan || last_path.is_none());
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
                        pose = None; last_path = None; needs_plan = false; execution_admitted = false;
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
                    pose = None; last_path = None; execution_admitted = false;
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
                if newest_goal.as_ref().is_some_and(|current| validate_goal_envelope_order(&incoming, current).is_err()) { continue; }
                if let Err(error) = navigation_boundary_from_goal(&incoming) { diagnostics.reject("surface goal boundary",&error.to_string());continue; }
                newest_goal = Some(incoming.clone()); goal = Some(incoming);
                path_revision = 0; last_path = None; path_surface = None; needs_plan = true; execution_admitted = false;
            }
            frame = poses.next() => {
                let Some(input)=drain_guarded_pose_input(frame,&mut poses,&mut diagnostics,Some(source_max_age))? else {continue;};
                let incoming=input.pose;
                let valid_until=match guarded_pose_source_deadline(incoming.stamp.nanos,source_max_age,
                    std::time::SystemTime::now(),tokio::time::Instant::now()) {
                    Ok(deadline)=>deadline,
                    Err(reason)=>{diagnostics.reject("surface pose source",&reason);continue;}
                };
                if pose.as_ref().is_none_or(|(previous,_,_)| incoming.stamp.nanos>previous.stamp.nanos) {
                    pose=Some((incoming,Instant::now(),valid_until));
                }
            }
            _ = ticker.tick() => {
                let (Some(map), Some((pose, pose_received_at, pose_valid_until)), Some(active_goal)) = (latest_map.as_ref(), pose.as_ref(), goal.as_ref()) else { continue; };
                if !execution_admitted || !map.authority.permits_goal(active_goal)
                    || map.received_at.elapsed() > nuwax_nav_runtime::DEFAULT_COSTMAP_TTL
                    || pose_received_at.elapsed() > nuwax_nav_runtime::DEFAULT_POSE_TTL
                    || tokio::time::Instant::now() >= *pose_valid_until { continue; }
                let Ok(surface) = map.snapshot.as_ref() else { continue; };
                if !needs_plan {
                    if let Some(path) = last_path.as_ref() {
                        let unchanged = path_surface.as_ref().and_then(std::sync::Weak::upgrade)
                            .is_some_and(|previous| previous.same_geometry(surface));
                        if unchanged || body_path_remains_traversable(surface, path, &config) {
                            path_surface = Some(Arc::downgrade(surface));
                            continue;
                        }
                    }
                }
                match body_path(surface, pose, active_goal, &config) {
                    Ok(path) => {
                        path_revision = path_revision.checked_add(1).ok_or_else(|| NavStackError::Internal("surface path revision exhausted".into()))?;
                        publish_surface_path(&transport, &node.outputs[0], active_goal, generation, path_revision, &path).await?;
                        last_path = Some(path); path_surface = Some(Arc::downgrade(surface)); needs_plan = false;
                    }
                    Err(error) => { diagnostics.reject("surface planning", &error.to_string()); last_path = None; }
                }
            }
        }
    }
    Ok(())
    }.await;
    pending_map.take();
    // 关闭/错误不遗留阻塞构建，晚到结果不再交付执行。
    while let Some(joined) = computations.join_next().await {
        if let Err(error) = joined {
            if terminal.is_ok() { return Err(NavStackError::Internal(error.to_string())); }
        }
    }
    terminal
}

#[cfg(test)]
mod tests {
    use super::super::guarded_tests::RecordingMotion;
    use super::*;
    use nuwax_nav_runtime::{NavSchedulerHandle, SchedulerPhase, TerminalOutcome};

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
        let mut follower = nuwax_nav_runtime::SurfaceFollower::new(configured.follower).unwrap();
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
                    Box::pin(run_surface_planner_with_transport(
                        planner_spec,
                        planner_hub,
                        config(),
                        planner_transport,
                        shutdown,
                        ready,
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
                            let local = nuwax_nav_runtime::SurfaceFollower::new(config().follower)?;
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
                let mut ticker = tokio::time::interval(Duration::from_millis(25));
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
