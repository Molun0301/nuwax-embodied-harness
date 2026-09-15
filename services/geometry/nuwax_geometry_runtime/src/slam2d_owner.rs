//! 二维算法的生产拥有方：保留子进程、绑定会话、提交栅格与坐标。
use crate::{
    correction_coordinator::RelocalizationModeFence,
    lcm_runtime::{self, GeometryRuntimeHandle, PoseState},
};
use nuwax_slam_2d_runtime::{
    protocol::{Operation, Session, Snapshot, StatusNotice, VERSION},
    supervisor::{Client, ProcessOwner},
};
use std::{path::PathBuf, sync::Arc, time::Duration};

#[derive(Clone)]
pub(crate) struct Settings {
    pub template: Session,
    pub binary: PathBuf,
    pub data_root: PathBuf,
    pub unix_offset_ns: i128,
}
impl Settings {
    pub fn from_env() -> Result<Option<Self>, String> {
        let pipeline = nuwax_nav_stack::ProductionPipeline::from_env()
            .map_err(|error| error.to_string())?;
        if pipeline == nuwax_nav_stack::ProductionPipeline::Lio3D {
            return Ok(None);
        }
        let text = std::env::var("NUWAX_GEOMETRY_SLAM2D_CONFIG")
            .map_err(|_| "二维链缺少明确传感器、坐标、时钟与标定配置")?;
        let mut value: serde_json::Value =
            serde_json::from_str(&text).map_err(|e| e.to_string())?;
        let object = value.as_object_mut().ok_or("二维配置必须为JSON对象")?;
        for (key, value_) in [
            ("session_id", serde_json::json!("preflight")),
            ("map_name", serde_json::json!("preflight")),
            ("session_generation", serde_json::json!(1)),
        ] {
            object.insert(key.to_owned(), value_);
        }
        if object.get("clock_domain").and_then(|v| v.as_str()) == Some("monotonic_ns")
            && object.get("clock_id").and_then(|v| v.as_str()) == Some("local_boot")
        {
            let boot = std::fs::read_to_string("/proc/sys/kernel/random/boot_id")
                .map_err(|e| e.to_string())?;
            object.insert(
                "clock_id".into(),
                serde_json::Value::String(boot.trim().into()),
            );
        }
        let template: Session = serde_json::from_value(value).map_err(|e| e.to_string())?;
        template.validate()?;
        let unix_offset_ns = match template.clock_domain.as_str() {
            "unix_ns" => 0,
            "monotonic_ns" => {
                let mut stamp = libc::timespec {
                    tv_sec: 0,
                    tv_nsec: 0,
                };
                // 仅接受本机启动身份，防止另一设备的单调时间被套用本机偏移。
                let boot = std::fs::read_to_string("/proc/sys/kernel/random/boot_id")
                    .map_err(|e| e.to_string())?;
                if template.clock_id != boot.trim() {
                    return Err("二维单调时钟标识必须为本机boot_id".into());
                }
                if unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut stamp) } != 0 {
                    return Err(std::io::Error::last_os_error().to_string());
                }
                let unix = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map_err(|e| e.to_string())?
                    .as_nanos() as i128;
                unix - (stamp.tv_sec as i128 * 1_000_000_000 + stamp.tv_nsec as i128)
            }
            _ => {
                return Err(
                    "设备时钟需先提供经过验证的时间映射，生产二维链暂不接纳未映射设备时钟".into(),
                )
            }
        };
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../..");
        let binary = std::env::var_os("NUWAX_GEOMETRY_SLAM2D_BINARY")
            .filter(|v| !v.is_empty())
            .map(PathBuf::from)
            .unwrap_or_else(|| {
                root.join("services/nuwax-workspace/target/release/nuwax-slam-2d-worker")
            });
        if !binary.is_file() {
            return Err(format!("二维工作程序不存在：{}", binary.display()));
        }
        let data_root = std::env::var("STATE_DIRECTORY")
            .ok()
            .and_then(|value| value.split(':').next().map(PathBuf::from))
            .filter(|path| path.is_absolute())
            .map(|path| path.join("slam2d"))
            .or_else(|| {
                std::env::var_os("NUWAX_RUNTIME_DATA_DIR")
                    .filter(|v| !v.is_empty())
                    .map(|path| PathBuf::from(path).join("geometry/slam2d"))
            })
            .or_else(|| {
                std::env::var_os("NUWAX_GEOMETRY_SEQUENCE_STATE_PATH").and_then(|path| {
                    PathBuf::from(path)
                        .parent()
                        .map(|parent| parent.join("slam2d"))
                })
            })
            .unwrap_or_else(|| root.join("runtime_data/geometry/slam2d"));
        Ok(Some(Self {
            template,
            binary,
            data_root,
            unix_offset_ns,
        }))
    }
    pub(crate) fn unix_stamp(&self, source: u64) -> Result<u64, String> {
        u64::try_from(source as i128 + self.unix_offset_ns)
            .map_err(|_| "二维采样时间映射越界".into())
    }
}
/// 每次操作捕获实际客户端；进程切换与会话围栏一起提交。
#[derive(Clone)]
pub(crate) struct ClientSlot(Arc<parking_lot::RwLock<Client>>);
impl ClientSlot {
    pub fn current(&self) -> Client {
        self.0.read().clone()
    }
    pub fn identity(&self) -> nuwax_slam_2d_runtime::protocol::Identity {
        self.0.read().identity().clone()
    }
    pub async fn execute(
        &self,
        op: Operation,
    ) -> Result<nuwax_slam_2d_runtime::protocol::Response, String> {
        self.current().execute(op).await
    }
    pub async fn status(&self) -> Result<nuwax_slam_2d_runtime::protocol::Status, String> {
        self.current().status().await
    }
    pub async fn read_snapshot(&self, s: &Snapshot) -> Result<Vec<i8>, String> {
        self.current().read_snapshot(s).await
    }
    pub async fn publish_scan(
        &self,
        g: u64,
        s: &nuwax_codec::lcm::planar::PlanarScan,
    ) -> Result<(), String> {
        self.current().publish_scan(g, s).await
    }
}
#[derive(Clone)]
struct RelocalizationInput {
    client: Client,
    session_generation: u64,
    epoch: u64,
    armed_unix_ns: u64,
    scans: Arc<parking_lot::Mutex<std::collections::VecDeque<(u64, u64, [u8; 32])>>>,
}
struct Owners {
    current: Option<ProcessOwner>,
    staged: Vec<ProcessOwner>,
    retiring: Vec<ProcessOwner>,
}
#[derive(Clone)]
struct LaserOdometry {session_id:String,generation:u64,epoch:u64,stamp:u64,pose:[f64;3]}
#[derive(Clone)]
pub(crate) struct Binding {
    pub client: ClientSlot,
    owners: Arc<parking_lot::Mutex<Owners>>,
    context: Arc<nuwax_lcm::LcmContext>,
    /// 栅格轮询、导出和候选交换互斥；耗时工作不占公共生命周期锁。
    pub operation: Arc<tokio::sync::Mutex<()>>,
    relocation: Arc<parking_lot::RwLock<Option<RelocalizationInput>>>,
    status: Arc<parking_lot::RwLock<Option<nuwax_slam_2d_runtime::protocol::Status>>>,
    latest_laser: Arc<parking_lot::RwLock<Option<LaserOdometry>>>,
    pub settings: Settings,
    pub fence: Arc<RelocalizationModeFence>,
    pub navigation: Arc<crate::planar_navigation::Owner>,
    pub navigation_binding: (nuwax_protocol::LcmTopic, nuwax_protocol::LcmMessageType),
    pub tf_binding: (nuwax_protocol::LcmTopic, nuwax_protocol::LcmMessageType),
}
impl Binding {
    pub fn arm_relocalization_input(
        &self,
        client: Client,
        generation: u64,
        epoch: u64,
        armed_unix_ns: u64,
    ) {
        *self.relocation.write() = Some(RelocalizationInput {
            client,
            session_generation: generation,
            epoch,
            armed_unix_ns,
            scans: Arc::new(parking_lot::Mutex::new(std::collections::VecDeque::new())),
        });
    }
    pub fn clear_relocalization_input(&self, epoch: u64) {
        let mut input = self.relocation.write();
        if input.as_ref().is_some_and(|i| i.epoch == epoch) {
            *input = None;
        }
    }
    pub fn relocation_scan_proof(&self, epoch: u64, sequence: u64) -> Option<(u64, [u8; 32])> {
        let input = self.relocation.read().clone()?;
        if input.epoch != epoch {
            return None;
        }
        let result = input
            .scans
            .lock()
            .iter()
            .find(|(s, _, _)| *s == sequence)
            .map(|(_, t, h)| (*t, *h));
        result
    }
    pub async fn route_relocalization_scan(
        &self,
        scan: &nuwax_codec::lcm::planar::PlanarScan,
    ) -> Result<(), String> {
        let input = self
            .relocation
            .read()
            .clone()
            .ok_or("二维重定位候选正在准备")?;
        let epoch = self.fence.snapshot();
        let stamp = self.settings.unix_stamp(scan.timestamp_ns as u64)?;
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_err(|e| e.to_string())?
            .as_nanos() as u64;
        if !epoch.active
            || epoch.epoch != input.epoch
            || stamp < input.armed_unix_ns
            || stamp > now.saturating_add(50_000_000)
            || now.saturating_sub(stamp) > 500_000_000
        {
            return Err("二维重定位扫描不是本次请求后的新鲜来源".into());
        }
        let bytes = nuwax_codec::lcm::planar::encode(scan).map_err(|e| e.to_string())?;
        input
            .client
            .publish_scan(input.session_generation, scan)
            .await?;
        let mut scans = input.scans.lock();
        if scans.len() >= 16 {
            scans.pop_front();
        }
        scans.push_back((
            scan.source_sequence as u64,
            stamp,
            nuwax_codec::hash::sha256_bytes(&bytes),
        ));
        Ok(())
    }
    pub async fn route_relocalization_odom(
        &self,
        sample: nuwax_slam_2d_runtime::input::TimedPose,
    ) -> Result<(), String> {
        let input = self
            .relocation
            .read()
            .clone()
            .ok_or("二维重定位候选正在准备")?;
        let epoch = self.fence.snapshot();
        if !epoch.active || epoch.epoch != input.epoch {
            return Err("二维重定位里程计围栏失效".into());
        }
        let response = input.client.execute(Operation::Odometry { sample }).await?;
        if let Some(e) = response.error {
            return Err(e);
        }
        Ok(())
    }
    pub async fn control_until_settled(
        &self,
        operation: Operation,
    ) -> Result<nuwax_slam_2d_runtime::protocol::Response, String> {
        let deadline = tokio::time::Instant::now() + Duration::from_secs(120);
        loop {
            match self.client.execute(operation.clone()).await {
                Ok(result) => {
                    *self.status.write() = Some(result.status.clone());
                    if let Some(e) = result.error.clone() {
                        return Err(e);
                    }
                    return Ok(result);
                }
                Err(error) => {
                    // 原序号重试只用于传输未决；确定错误由状态回执返回。
                    if tokio::time::Instant::now() >= deadline {
                        return Err(error);
                    }
                    tokio::time::sleep(Duration::from_millis(20)).await;
                }
            }
        }
    }
    pub async fn prepare_candidate(
        &self,
        source: std::fs::File,
        size: u64,
        sha: [u8; 32],
        session: Session,
        localization: bool,
        seed: [f64; 3],
    ) -> Result<Client, String> {
        let owner = ProcessOwner::start(
            &self.settings.binary,
            self.context.clone(),
            self.client.identity().runtime_generation,
            &self.settings.data_root,
        )
        .await?;
        let client = owner.client();
        self.owners.lock().staged.push(owner);
        let result = async {
            let artifact = client.stage_artifact(source, size, sha).await?;
            let response = client
                .execute(Operation::LoadGraph {
                    session,
                    artifact,
                    localization,
                    seed,
                })
                .await?;
            if let Some(e) = response.error {
                return Err(e);
            }
            Ok(client.clone())
        }
        .await;
        if result.is_err() {
            self.discard_candidate(client.identity().owner_nonce.as_str())
                .await?;
        }
        result
    }
    pub async fn discard_candidate(&self, nonce: &str) -> Result<(), String> {
        let candidate = {
            let mut owners = self.owners.lock();
            owners
                .staged
                .iter()
                .position(|o| o.client().identity().owner_nonce == nonce)
                .map(|i| owners.staged.remove(i))
        };
        if let Some(mut owner) = candidate {
            let result = owner.shutdown().await;
            if owner.process_id().is_some() {
                self.owners.lock().retiring.push(owner);
            }
            result
        } else {
            Ok(())
        }
    }
    /// 调用者持有P→L且已推进会话；只交换实际资源，不等待根监督循环。
    pub fn promote(&self, client: &Client) -> Result<(), String> {
        let mut owners = self.owners.lock();
        let i = owners
            .staged
            .iter()
            .position(|o| o.client().identity() == client.identity())
            .ok_or("二维候选拥有方丢失")?;
        let owner = owners.staged.remove(i);
        if let Some(old) = owners.current.replace(owner) {
            owners.retiring.push(old);
        }
        *self.client.0.write() = client.clone();
        *self.status.write() = None;
        Ok(())
    }
    pub(crate) async fn retire(&self) -> Result<(), String> {
        let mut failures = Vec::new();
        loop {
            let owner = self.owners.lock().retiring.pop();
            let Some(mut owner) = owner else { break };
            if let Err(e) = owner.shutdown().await {
                failures.push(e);
            }
            if owner.process_id().is_some() {
                self.owners.lock().retiring.push(owner);
                break;
            }
        }
        if failures.is_empty() {
            Ok(())
        } else {
            Err(failures.join("；"))
        }
    }
    async fn close(&self) -> Result<(), String> {
        {
            let mut owners = self.owners.lock();
            if let Some(owner) = owners.current.take() {
                owners.retiring.push(owner);
            }
            let staged = std::mem::take(&mut owners.staged);
            owners.retiring.extend(staged);
        }
        self.retire().await
    }
    pub async fn execute(
        &self,
        operation: Operation,
    ) -> Result<nuwax_slam_2d_runtime::protocol::Response, String> {
        let response = self.client.execute(operation).await?;
        *self.status.write() = Some(response.status.clone());
        if let Some(error) = response.error.clone() {
            return Err(error);
        }
        Ok(response)
    }

    pub(crate) fn sensor_edge(&self) -> Option<lcm_runtime::TransformEntry> {
        let session = &self.settings.template;
        if session.frame_id.trim().trim_start_matches('/')
            == session.body_frame_id.trim().trim_start_matches('/')
        {
            return None;
        }
        let offset = session.core_config["base_from_laser"]
            .as_array()
            .expect("已验证二维外参");
        let yaw = offset[2].as_f64().expect("已验证二维偏航");
        Some(lcm_runtime::TransformEntry {
            parent: session.body_frame_id.clone(),
            child: session.frame_id.clone(),
            tx: offset[0].as_f64().unwrap(),
            ty: offset[1].as_f64().unwrap(),
            rz: (yaw * 0.5).sin(),
            rw: (yaw * 0.5).cos(),
            ..Default::default()
        })
    }
    pub(crate) fn install_sensor_edge(&self, state: &mut lcm_runtime::SharedState) {
        if let Some(edge) = self.sensor_edge() {
            lcm_runtime::apply_sensor_extrinsic_locked(state, &edge);
        }
    }

    pub(crate) fn diagnostics(
        &self,
        session: &crate::mapping_adapter::MappingSessionSnapshot,
    ) -> Option<serde_json::Value> {
        let status = self.status.read();
        let status = status.as_ref()?;
        let current = status.session.as_ref()?;
        if current.session_id != session.session_id
            || current.session_generation != session.session_generation
        {
            return None;
        }
        Some(serde_json::json!({
            "scan_input_mode":status.scan_input_mode,"input_beams":status.input_beams,
            "processed_beams":status.processed_beams,"valid_beams":status.valid_beams,
            "processing_ms":status.processing_ms,
            "received_scans":status.received_scans,"dropped_scans":status.dropped_scans,
            "rejected_scans":status.rejected_scans,"worker_active":status.active,
            "last_estimate":status.last_estimate,"last_error":status.last_error,"motion_compensation":status.motion_compensation,
        }))
    }

    pub fn session(&self, id: &str, name: &str, generation: u64) -> Session {
        let mut session = self.settings.template.clone();
        session.session_id = id.into();
        session.map_name = name.into();
        session.session_generation = generation;
        session
    }
}

pub(crate) async fn start(
    settings: Settings,
    transport: nuwax_transport::LcmTransport,
    generation: u64,
    fence: Arc<RelocalizationModeFence>,
    tf_binding: (nuwax_protocol::LcmTopic, nuwax_protocol::LcmMessageType),
    navigation_binding: (nuwax_protocol::LcmTopic, nuwax_protocol::LcmMessageType),
) -> Result<
    (
        crate::lio_owner::LioOwnerHandle,
        Binding,
        tokio::sync::oneshot::Sender<GeometryRuntimeHandle>,
    ),
    String,
> {
    let context = transport
        .shared_lcm_context()
        .ok_or("二维运行时要求真实LCM上下文")?;
    let owner = ProcessOwner::start(
        &settings.binary,
        context.clone(),
        generation,
        &settings.data_root,
    )
    .await?;
    let binding = Binding {
        client: ClientSlot(Arc::new(parking_lot::RwLock::new(owner.client()))),
        owners: Arc::new(parking_lot::Mutex::new(Owners {
            current: Some(owner),
            staged: Vec::new(),
            retiring: Vec::new(),
        })),
        context,
        operation: Arc::new(tokio::sync::Mutex::new(())),
        relocation: Arc::new(parking_lot::RwLock::new(None)),
        status: Arc::new(parking_lot::RwLock::new(None)),
        latest_laser: Arc::new(parking_lot::RwLock::new(None)),
        settings,
        fence,
        tf_binding,
        navigation_binding,
        navigation: Arc::new(crate::planar_navigation::Owner::default()),
    };
    let supervisor_binding = binding.clone();
    let (tx, rx) = tokio::sync::oneshot::channel();
    let handle =
        crate::lio_owner::LioOwnerHandle::supervised_planar(move |mut shutdown| async move {
            let bridge = tokio::select! {
                result=rx=>result.ok(),
                _=shutdown.changed()=>None,
            };
            let result = if let Some(bridge) = bridge {
                supervise(&supervisor_binding, &bridge, &mut shutdown).await
            } else {
                Ok(())
            };
            let closed = supervisor_binding.close().await;
            match (result, closed) {
                (Ok(()), result) => result,
                (Err(error), Ok(())) => Err(error),
                (Err(error), Err(close)) => Err(format!("{error}；二维进程回收同时失败：{close}")),
            }
        });
    Ok((handle, binding, tx))
}
async fn supervise(
    binding: &Binding,
    bridge: &GeometryRuntimeHandle,
    shutdown: &mut tokio::sync::watch::Receiver<bool>,
) -> Result<(), String> {
    let mut notices = binding
        .context
        .subscribe("^2/[^/]+/state#nuwax_msgs\\.PlanarStatus$")
        .await
        .map_err(|e| e.to_string())?;
    let mut pending: Option<tokio::sync::oneshot::Receiver<Result<(), String>>> = None;
    let result=async {
        let mut notice_identity=binding.client.current().identity().clone();
        let mut notice_sequence=0_u64;
        let mut tick=tokio::time::interval(Duration::from_millis(100));
        tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        let mut failures=0;
        let mut refresh_due = true;
        loop {
            let mut incoming=None;
            tokio::select! {biased;
                _=shutdown.changed()=>return Ok(()),
                message=notices.receiver.recv()=>match message {
                    Ok(message)=>{
                        if let Ok(notice)=serde_json::from_slice::<StatusNotice>(&message.payload) {
                            let current=binding.client.current().identity().clone();
                            if current!=notice_identity {notice_identity=current.clone();notice_sequence=0;}
                            if notice.version==VERSION && notice.identity==current && notice.sequence>notice_sequence {
                                notice_sequence=notice.sequence;incoming=Some(notice.status);refresh_due=true;
                            }
                        }
                    },
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(_))=>{},
                    Err(e)=>return Err(e.to_string()),
                },
                result=async {match &mut pending {Some(receiver)=>Some(receiver.await),None=>std::future::pending().await}}=>{
                    pending=None;
                    match result.unwrap().map_err(|_|"二维保留任务未交付终态".to_string())? {
                        Ok(())=>failures=0,
                        Err(e)=>{failures+=1;if failures>=3{return Err(e);}}
                    }
                },
                _=tick.tick()=>{refresh_due=true;},
            }
            if *shutdown.borrow(){return Ok(());}
            binding.retire().await?;
            binding.owners.lock().current.as_mut().ok_or("二维当前进程丢失")?.check_alive()?;
            let adapter=bridge.mapping_adapter.as_ref().ok_or("二维适配器丢失")?;
            let (session,revision,ready)={let a=adapter.lock();(a.snapshot(),a.planar_revision,a.planar_ready)};
            if let Some(status)=incoming {
                let epoch=binding.fence.snapshot();
                if status.active && !status.paused && !status.stopping && status.last_error.is_none() && !epoch.active {
                    if let (Some(current),Some(estimate))=(&status.session,&status.last_estimate) {
                        if current.session_id==session.session_id && current.session_generation==session.session_generation
                            && matches!(estimate.disposition.as_str(),"accepted"|"initialized"|"motion_skipped") {
                            if current.odometry_source==nuwax_slam_2d_runtime::protocol::OdometrySource::Laser {
                                update_pose(bridge,binding,current,estimate,epoch.epoch,status.localization).await?;
                            }
                        }
                    }
                }
                *binding.status.write()=Some(status);
            }
            if !session.active || !ready || pending.is_some() || !refresh_due {continue;}
            let epoch=binding.fence.snapshot();if epoch.active{continue;}
            if epoch.epoch!=adapter.lock().planar_epoch {
                if let Ok(_operation)=binding.operation.try_lock(){let mut a=adapter.lock();a.planar_ready=false;a.planar_error=Some("二维原生图跨越重定位代际，必须重新建立会话".into());}
                continue;
            }
            refresh_due=false;
            let owner=bridge.clone();let binding=binding.clone();let current=session.clone();
            match bridge.register_planar_operation(async move {poll_and_commit(&owner,&binding,&current,revision,epoch.epoch).await}) {
                Ok(receiver)=>pending=Some(receiver),
                Err(_) if bridge.task_control().is_some_and(|control|!control.admission_is_open())=>{},
                Err(error)=>return Err(error),
            }
        }
    }.await;
    // 关闭仍等待已保留的地图任务，再由根拥有方回收算法进程。
    if let Some(receiver) = pending.take() {
        let _ = receiver.await;
    }
    let unsubscribed = notices.unsubscribe().await.map_err(|e| e.to_string());
    result.and(unsubscribed)
}

async fn poll_and_commit(
    bridge: &GeometryRuntimeHandle,
    binding: &Binding,
    session: &crate::mapping_adapter::MappingSessionSnapshot,
    revision: u64,
    epoch: u64,
) -> Result<(), String> {
    let _operation = binding.operation.lock().await;
    let status = binding.client.status().await?;
    *binding.status.write() = Some(status.clone());
    if status.session.as_ref().is_none_or(|current| {
        current.session_id != session.session_id
            || current.session_generation != session.session_generation
    }) {
        record_error(
            bridge,
            &session,
            Some("二维算法会话身份与Geometry不一致".into()),
        );
        return Ok(());
    }
    if !status.active {
        record_error(bridge, &session, Some("二维算法会话已停止".into()));
        return Ok(());
    }
    if let Some(error) = status.last_error {
        record_error(bridge, &session, Some(error));
        return Ok(());
    }
    let Some(estimate) = status.last_estimate else {
        return Ok(());
    };
    if estimate.disposition == "match_rejected" {
        record_error(
            bridge,
            &session,
            Some("二维扫描匹配退化，等待有效观测".into()),
        );
        return Ok(());
    }
    record_error(bridge, &session, None);
    if estimate.graph_revision == revision {
        if matches!(
            estimate.disposition.as_str(),
            "accepted" | "initialized" | "motion_skipped"
        ) {
            if let Some(session) = status.session {
                update_pose(
                    bridge,
                    binding,
                    &session,
                    &estimate,
                    epoch,
                    status.localization,
                )
                .await?;
                crate::planar_navigation::observe(bridge, binding, &session, &estimate, epoch)
                    .await?;
            }
        }
        return Ok(());
    }
    if estimate.graph_revision < revision {
        return Ok(());
    }
    let response = binding
        .execute(Operation::Snapshot {
            session_id: session.session_id.clone(),
            session_generation: session.session_generation,
        })
        .await?;
    let Some(snapshot) = response.snapshot else {
        return Ok(());
    };
    let result = async {
        let data = binding.client.read_snapshot(&snapshot).await?;
        commit(bridge, binding, &snapshot, data, epoch).await
    }
    .await;
    binding
        .execute(Operation::ReleaseSnapshot {
            snapshot_id: snapshot.snapshot_id,
        })
        .await?;
    if let Err(error) = result {
        record_error(bridge, &session, Some(error));
    } else {
        crate::planar_navigation::observe(
            bridge,
            binding,
            &snapshot.session,
            &snapshot.estimate,
            epoch,
        )
        .await?;
    }
    Ok(())
}

async fn commit(
    bridge: &GeometryRuntimeHandle,
    binding: &Binding,
    snapshot: &Snapshot,
    data: Vec<i8>,
    epoch: u64,
) -> Result<(), String> {
    let estimate = &snapshot.estimate;
    if estimate.clock_domain != binding.settings.template.clock_domain
        || estimate.clock_id != binding.settings.template.clock_id
    {
        return Err("二维结果时钟身份冲突".into());
    }
    let timestamp = binding.settings.unix_stamp(estimate.source_timestamp_ns)?;
    let effective = nalgebra::Isometry3::from_parts(
        nalgebra::Translation3::new(estimate.map_from_odom[0], estimate.map_from_odom[1], 0.0),
        nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, estimate.map_from_odom[2]),
    );
    let preparation = bridge
        .mapping_adapter
        .as_ref()
        .ok_or("二维适配器不可用")?
        .lock()
        .prepare_planar_map(snapshot.clone(), data, timestamp);
    let prepared = tokio::task::spawn_blocking(move || preparation.compute())
        .await
        .map_err(|e| e.to_string())??;
    let _projection = bridge.lock_explore_projection_owned().await;
    let _lifecycle = bridge.mapping_lifecycle_lock.lock().await;
    if binding.fence.snapshot().epoch != epoch {
        return Err("二维候选跨越重定位代际".into());
    }
    let mut state = bridge.state.write().await;
    let odom = planar_commit_odometry(
        &binding.navigation,
        &snapshot.session,
        estimate,
        binding.settings.unix_offset_ns,
        state.pose.timestamp_ns,
        binding.latest_laser.read().as_ref(),
        epoch,
    )?;
    let observation = lcm_runtime::canonical_mapping_observation_from_isometry(
        &snapshot.session.map_frame_id,
        &snapshot.session.odom_frame_id,
        snapshot.session.session_generation,
        &effective,
        &odom,
        &snapshot.session.body_frame_id,
    )?;

    let adapter = bridge.mapping_adapter.as_ref().ok_or("二维适配器不可用")?;
    let (map, event) = adapter
        .lock()
        .commit_planar_snapshot(prepared, state.map.revision)?;
    state.map = map;
    lcm_runtime::apply_canonical_mapping_observation_locked(&mut state, &observation);
    binding.install_sensor_edge(&mut state);
    drop(state);
    let _ = bridge.map_revision_tx.send(event);
    publish_tf(bridge, binding, observation.tf_payload, odom.timestamp_ns).await?;
    Ok(())
}

/// 新图提交时保留较新的已接纳原始里程计，避免地图计算把公共位姿倒退到旧扫描。
fn planar_commit_odometry(
    navigation: &crate::planar_navigation::Owner,
    session: &Session,
    estimate: &nuwax_slam_2d_runtime::protocol::Estimate,
    unix_offset_ns: i128,
    latest_unix_ns: u64,
    latest_laser: Option<&LaserOdometry>,
    epoch: u64,
) -> Result<PoseState, String> {
    let mut timestamp = u64::try_from(i128::from(estimate.source_timestamp_ns) + unix_offset_ns)
        .map_err(|_| "二维提交采样时间越界")?;
    let mut pose = estimate.odom_from_base;
    if session.odometry_source == nuwax_slam_2d_runtime::protocol::OdometrySource::Device
        && latest_unix_ns > timestamp
    {
        let native = u64::try_from(i128::from(latest_unix_ns) - unix_offset_ns)
            .map_err(|_| "二维设备时间越界")?;
        pose = navigation
            .odometry_at(
                session.session_generation,
                native,
                &session.clock_domain,
                &session.clock_id,
            )?
            .ok_or("新图提交缺少已发布位姿的原始设备采样")?;
        timestamp = latest_unix_ns;
    }
    if session.odometry_source == nuwax_slam_2d_runtime::protocol::OdometrySource::Laser {
        if let Some(latest) = latest_laser.filter(|p| {
            p.session_id == session.session_id
                && p.generation == session.session_generation
                && p.epoch == epoch
                && p.stamp > timestamp
        }) {
            timestamp = latest.stamp;
            pose = latest.pose;
        }
        if timestamp < latest_unix_ns {
            return Err("新图提交缺少已发布位姿的原始激光采样".into());
        }
    }
    Ok(PoseState {
        x: pose[0],
        y: pose[1],
        qz: (pose[2] * 0.5).sin(),
        qw: (pose[2] * 0.5).cos(),
        frame_id: session.odom_frame_id.clone(),
        timestamp_ns: timestamp,
        ..Default::default()
    })
}

/// 设备里程计只沿当前已提交有效校正更新位置，不提前使用待提交回环。
/// 同会话的新鲜匹配证明输入有效；原生图可领先规范提交，但不能倒退或自相矛盾。
fn device_pose_has_current_basis(
    status:&nuwax_slam_2d_runtime::protocol::Status, session_id:&str, generation:u64,
    revision:u64, sample:&nuwax_slam_2d_runtime::input::TimedPose,
)->bool {
    let (Some(session),Some(estimate))=(&status.session,&status.last_estimate) else{return false;};
    status.active && !status.paused && !status.stopping && status.last_error.is_none()
        && session.session_id==session_id && session.session_generation==generation
        && session.odometry_source==nuwax_slam_2d_runtime::protocol::OdometrySource::Device
        && session.odom_frame_id==sample.odom_frame_id && session.body_frame_id==sample.body_frame_id
        && session.clock_domain==sample.clock_domain && session.clock_id==sample.clock_id
        && estimate.graph_revision==status.graph_revision && status.graph_revision>=revision
        && estimate.clock_domain==sample.clock_domain && estimate.clock_id==sample.clock_id
        && estimate.source_timestamp_ns>0 && sample.timestamp_ns>=estimate.source_timestamp_ns
        && sample.timestamp_ns-estimate.source_timestamp_ns<=500_000_000
        && matches!(estimate.disposition.as_str(),"accepted"|"initialized"|"motion_skipped")
}

/// 调用方已持原生命周期锁，并取得原生里程计接纳回执；此处不推进地图修订。
pub(crate) async fn publish_device_odometry(
    bridge:&GeometryRuntimeHandle,binding:&Binding,
    session:&crate::mapping_adapter::MappingSessionSnapshot,
    sample:&nuwax_slam_2d_runtime::input::TimedPose,
    status:&nuwax_slam_2d_runtime::protocol::Status,
)->Result<bool,String> {
    let fence=binding.fence.snapshot();
    if fence.active{return Ok(false);}
    let adapter=bridge.mapping_adapter.as_ref().ok_or("二维适配器不存在")?;
    let transform={
        let a=adapter.lock();let current=a.snapshot();
        if !current.active || !a.planar_ready || a.planar_epoch!=fence.epoch
            || current.session_id!=session.session_id || current.session_generation!=session.session_generation
            || !device_pose_has_current_basis(status,&session.session_id,session.session_generation,a.planar_revision,sample) {return Ok(false);}
        let Some(transform)=a.map_odom_transform_snapshot() else{return Ok(false);};
        transform
    };
    let native_session=status.session.as_ref().expect("已核对原生会话");
    if native_session.map_name!=session.map_name{return Ok(false);}
    let timestamp=binding.settings.unix_stamp(sample.timestamp_ns)?;
    let now=std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_err(|e|e.to_string())?.as_nanos()as u64;
    if timestamp>now.saturating_add(50_000_000) || now.saturating_sub(timestamp)>200_000_000 {return Ok(false);}
    let odom=PoseState{x:sample.pose[0],y:sample.pose[1],qz:(sample.pose[2]*0.5).sin(),qw:(sample.pose[2]*0.5).cos(),
        frame_id:native_session.odom_frame_id.clone(),timestamp_ns:timestamp,..Default::default()};
    let observation=lcm_runtime::canonical_mapping_observation_from_isometry(
        &native_session.map_frame_id,&native_session.odom_frame_id,transform.generation,&transform.effective_isometry(),&odom,&native_session.body_frame_id)?;
    let mut state=bridge.state.write().await;
    if timestamp<=state.pose.timestamp_ns{return Ok(false);}
    lcm_runtime::apply_canonical_mapping_observation_locked(&mut state,&observation);
    binding.install_sensor_edge(&mut state);drop(state);
    publish_tf(bridge,binding,observation.tf_payload,timestamp).await?;
    Ok(true)
}

async fn update_pose(
    bridge: &GeometryRuntimeHandle,
    binding: &Binding,
    session: &Session,
    estimate: &nuwax_slam_2d_runtime::protocol::Estimate,
    epoch: u64,
    localization: bool,
) -> Result<(), String> {
    let timestamp = binding.settings.unix_stamp(estimate.source_timestamp_ns)?;
    let _lifecycle = bridge.mapping_lifecycle_lock.lock().await;
    let fence = binding.fence.snapshot();
    if fence.active || fence.epoch != epoch {
        return Ok(());
    }
    let adapter = bridge.mapping_adapter.as_ref().ok_or("二维适配器不存在")?;
    let transform = {
        let mut a = adapter.lock();
        let current = a.snapshot();
        if !a.planar_ready
            || !current.active
            || current.session_id != session.session_id
            || current.session_generation != session.session_generation
            || a.planar_revision > estimate.graph_revision
        {
            return Ok(());
        }
        if localization {
            if a.planar_revision != estimate.graph_revision {
                return Ok(());
            }
            a.update_planar_localization(estimate)?
        } else {
            let Some(transform) = a.map_odom_transform_snapshot() else {
                return Ok(());
            };
            transform
        }
    };
    let odom = PoseState {
        x: estimate.odom_from_base[0],
        y: estimate.odom_from_base[1],
        qz: (estimate.odom_from_base[2] * 0.5).sin(),
        qw: (estimate.odom_from_base[2] * 0.5).cos(),
        frame_id: session.odom_frame_id.clone(),
        timestamp_ns: timestamp,
        ..Default::default()
    };
    let observation = lcm_runtime::canonical_mapping_observation_from_isometry(
        &session.map_frame_id,
        &session.odom_frame_id,
        transform.generation,
        &transform.effective_isometry(),
        &odom,
        &session.body_frame_id,
    )?;
    let mut state = bridge.state.write().await;
    if timestamp <= state.pose.timestamp_ns {
        return Ok(());
    }
    if session.odometry_source == nuwax_slam_2d_runtime::protocol::OdometrySource::Laser {
        // 与实际位姿发布共用生命周期和单调水位，旧控制状态不能回写缓存。
        *binding.latest_laser.write() = Some(LaserOdometry {
            session_id: session.session_id.clone(),
            generation: session.session_generation,
            epoch,
            stamp: timestamp,
            pose: estimate.odom_from_base,
        });
    }
    lcm_runtime::apply_canonical_mapping_observation_locked(&mut state, &observation);
    binding.install_sensor_edge(&mut state);
    drop(state);
    publish_tf(bridge, binding, observation.tf_payload, timestamp).await
}

fn record_error(
    bridge: &GeometryRuntimeHandle,
    session: &crate::mapping_adapter::MappingSessionSnapshot,
    error: Option<String>,
) {
    if let Some(adapter) = &bridge.mapping_adapter {
        let mut a = adapter.lock();
        let current = a.snapshot();
        if current.session_id == session.session_id
            && current.session_generation == session.session_generation
        {
            a.planar_error = error;
        }
    }
}

pub(crate) async fn publish_tf(
    bridge: &GeometryRuntimeHandle,
    binding: &Binding,
    payload: Vec<u8>,
    timestamp: u64,
) -> Result<(), String> {
    let payload = if let Some(edge) = binding.sensor_edge() {
        use nuwax_codec::lcm::schema;
        let mut message = schema::decode_tf_message_body(&payload).map_err(|e| e.to_string())?;
        message.transforms.push(schema::TransformStamped {
            header: schema::Header {
                frame_id: edge.parent,
                ..Default::default()
            },
            child_frame_id: edge.child,
            transform: schema::Transform {
                translation: [edge.tx, edge.ty, edge.tz],
                rotation_xyzw: [edge.rx, edge.ry, edge.rz, edge.rw],
            },
        });
        schema::encode_tf_message_body(&message).map_err(|e| e.to_string())?
    } else {
        payload
    };
    let frame = crate::typed_codec::build_typed_frame(
        binding.tf_binding.0.clone(),
        binding.tf_binding.1.clone(),
        payload,
        timestamp,
    );
    tokio::time::timeout(Duration::from_secs(2), bridge.transport.publish(frame))
        .await
        .map_err(|_| "二维规范坐标发布超时".to_string())?
        .map_err(|e| e.to_string())
}


#[cfg(test)]
mod device_pose_tests {
    use super::*;
    use nuwax_slam_2d_runtime::{input::TimedPose,protocol::{Status,Estimate,OdometrySource}};
    #[test]
    fn device_pose_requires_fresh_same_graph_scanning_basis() {
        let session=Session{session_id:"mapping-session".into(),map_name:"当前层".into(),session_generation:3,sensor_id:"scan".into(),
            frame_id:"laser".into(),map_frame_id:"map".into(),odom_frame_id:"odom".into(),body_frame_id:"base".into(),
            clock_domain:"unix_ns".into(),clock_id:String::new(),calibration_ref:"测试标定".into(),odometry_source:OdometrySource::Device,core_config:serde_json::json!({})};
        let estimate=Estimate{source_timestamp_ns:1_000_000_000,clock_domain:"unix_ns".into(),clock_id:String::new(),source_sequence:10,
            graph_revision:2,optimizations:0,anchor_odom_from_base:[0.0;3],odom_from_base:[0.0;3],map_from_base:[0.0;3],map_from_odom:[0.0;3],
            covariance:[0.0;9],matching_response:1.0,local_response:1.0,disposition:"motion_skipped".into(),node_count:1,edge_count:0};
        let mut status=Status{active:true,session:Some(session),last_estimate:Some(estimate),graph_revision:2,..Default::default()};
        let sample=TimedPose{timestamp_ns:1_050_000_000,clock_domain:"unix_ns".into(),clock_id:String::new(),odom_frame_id:"odom".into(),body_frame_id:"base".into(),pose:[1.0,2.0,0.2]};
        assert!(device_pose_has_current_basis(&status,"mapping-session",3,2,&sample));
        assert!(!device_pose_has_current_basis(&status,"old-session",3,2,&sample));
        assert!(!device_pose_has_current_basis(&status,"mapping-session",4,2,&sample));
        assert!(!device_pose_has_current_basis(&status,"mapping-session",3,3,&sample));
        // 候选图推进不暂停旧规范图上的设备位姿；新变换只在图提交时应用。
        status.graph_revision=3;
        status.last_estimate.as_mut().unwrap().graph_revision=3;
        assert!(device_pose_has_current_basis(&status,"mapping-session",3,2,&sample));
        status.last_estimate.as_mut().unwrap().graph_revision=4;
        assert!(!device_pose_has_current_basis(&status,"mapping-session",3,2,&sample));
        status.last_estimate.as_mut().unwrap().graph_revision=3;
        let navigation=crate::planar_navigation::Owner::default();
        navigation.capture_odometry(3,sample.clone()).unwrap();
        let session=status.session.as_ref().unwrap();let estimate=status.last_estimate.as_ref().unwrap();
        let odom=planar_commit_odometry(&navigation,session,estimate,0,sample.timestamp_ns,None,0).unwrap();
        assert_eq!(odom.timestamp_ns,sample.timestamp_ns);assert_eq!([odom.x,odom.y],sample.pose[..2]);
        let effective=nalgebra::Isometry3::from_parts(nalgebra::Translation3::new(3.0,-2.0,0.0),
            nalgebra::UnitQuaternion::from_euler_angles(0.0,0.0,std::f64::consts::FRAC_PI_2));
        let observation=lcm_runtime::canonical_mapping_observation_from_isometry("map","odom",3,&effective,&odom,"base").unwrap();
        let mut state=lcm_runtime::SharedState::default();
        lcm_runtime::apply_canonical_mapping_observation_locked(&mut state,&observation);
        assert_eq!(state.pose.timestamp_ns,sample.timestamp_ns);
        assert!((state.pose.x-1.0).abs()<1e-9 && (state.pose.y+1.0).abs()<1e-9);
        assert!(planar_commit_odometry(&navigation,session,estimate,0,sample.timestamp_ns+1,None,0).is_err());
        let mut laser_session=session.clone();laser_session.odometry_source=OdometrySource::Laser;
        let mut laser=LaserOdometry{session_id:laser_session.session_id.clone(),generation:3,epoch:7,
            stamp:sample.timestamp_ns,pose:sample.pose};
        let kept=planar_commit_odometry(&navigation,&laser_session,estimate,0,sample.timestamp_ns,Some(&laser),7).unwrap();
        assert_eq!(kept.timestamp_ns,sample.timestamp_ns);assert_eq!(kept.x,sample.pose[0]);
        assert!(planar_commit_odometry(&navigation,&laser_session,estimate,0,sample.timestamp_ns,Some(&laser),8).is_err());
        laser.generation=4;
        assert!(planar_commit_odometry(&navigation,&laser_session,estimate,0,sample.timestamp_ns,Some(&laser),7).is_err());
        let mut wrong=session.clone();wrong.session_generation=4;
        assert!(planar_commit_odometry(&navigation,&wrong,estimate,0,sample.timestamp_ns,None,0).is_err());
        let mut stale=sample.clone();stale.timestamp_ns=1_600_000_000;
        assert!(!device_pose_has_current_basis(&status,"mapping-session",3,2,&stale));
        stale.timestamp_ns=900_000_000;
        assert!(!device_pose_has_current_basis(&status,"mapping-session",3,2,&stale));
        status.last_estimate.as_mut().unwrap().disposition="match_rejected".into();
        assert!(!device_pose_has_current_basis(&status,"mapping-session",3,2,&sample));
        status.last_estimate=None;
        assert!(!device_pose_has_current_basis(&status,"mapping-session",3,2,&sample),"激活后的静态资产不能提供新鲜位姿依据");
    }
}
