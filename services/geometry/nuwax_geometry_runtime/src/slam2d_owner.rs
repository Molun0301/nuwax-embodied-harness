//! 二维工作进程由一个拥有方保留；控制与快照直接交给会话。
use crate::{proto, session::FrameRouter};
use nuwax_slam_2d_runtime::{
    protocol::{Operation, Response, Session, Snapshot},
    supervisor::{Client, ProcessOwner},
};
use std::{
    io::{Read, Seek, SeekFrom},
    path::PathBuf,
    sync::Arc,
    time::Duration,
};
use tokio::sync::{watch, Mutex};
#[derive(Clone)]
pub(crate) struct Settings {
    pub template: Session,
    pub binary: PathBuf,
    pub data_root: PathBuf,
    pub unix_offset_ns: i128,
}
impl Settings {
    pub fn from_env() -> Result<Option<Self>, String> {
        let pipeline =
            nuwax_nav_stack::ProductionPipeline::from_env().map_err(|error| error.to_string())?;
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

pub(crate) struct Owner {
    settings: Settings,
    client: Client,
    worker: Arc<Mutex<Option<ProcessOwner>>>,
    router: FrameRouter,
    session: Arc<parking_lot::RwLock<Option<Session>>>,
    state: watch::Receiver<nuwax_slam_2d_runtime::protocol::Status>,
    shutdown: watch::Sender<bool>,
    task: Mutex<Option<tokio::task::JoinHandle<()>>>,
}
impl Owner {
    pub(crate) async fn start(
        settings: Settings,
        transport: nuwax_transport::LcmTransport,
        router: FrameRouter,
    ) -> Result<Arc<Self>, String> {
        let context = transport
            .shared_lcm_context()
            .ok_or("二维工作进程需要真实通信上下文")?;
        // 1只用于尚未重写的工作进程内部协议，公共输入不携带代际。
        let process =
            ProcessOwner::start(&settings.binary, context, 1, &settings.data_root).await?;
        let client = process.client();
        let worker = Arc::new(Mutex::new(Some(process)));
        let (shutdown, mut stop) = watch::channel(false);
        let (states, state) = watch::channel(Default::default());
        let session = Arc::new(parking_lot::RwLock::new(None::<Session>));
        let owner = Arc::new(Self {
            settings: settings.clone(),
            client: client.clone(),
            worker: worker.clone(),
            router: router.clone(),
            session: session.clone(),
            state,
            shutdown,
            task: Mutex::new(None),
        });
        let task = tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_millis(100));
            tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
            let mut last_revision = None;
            let result: Result<(), String> = async {
                loop {
                    tokio::select! {biased;
                        _=stop.wait_for(|value|*value)=>return Ok(()),
                        _=tick.tick()=>{},
                    }
                    worker
                        .lock()
                        .await
                        .as_mut()
                        .ok_or("二维拥有方已关闭")?
                        .check_alive()?;
                    let status = client.status().await?;
                    if let Some(error) = &status.last_error {
                        return Err(error.clone());
                    }
                    states.send_replace(status.clone());
                    let current = session.read().clone();
                    let Some(current) = current else { continue };
                    let Some(estimate) = &status.last_estimate else {
                        continue;
                    };
                    let id = current
                        .session_id
                        .parse::<u64>()
                        .map_err(|e| e.to_string())?;
                    if status
                        .session
                        .as_ref()
                        .is_none_or(|value| value.session_id != current.session_id)
                    {
                        continue;
                    }
                    if !status.localization {
                        router
                            .publish_map_from_odom(
                                id,
                                pose(
                                    estimate.map_from_odom,
                                    &current.map_frame_id,
                                    settings.unix_stamp(estimate.source_timestamp_ns)?,
                                ),
                            )
                            .await;
                    }
                    let position = estimate.odom_from_base;
                    let frame = &current.odom_frame_id;
                    router.publish_pose(
                        id,
                        pose(
                            position,
                            frame,
                            settings.unix_stamp(estimate.source_timestamp_ns)?,
                        ),
                    );
                    if !status.localization
                        && Some((id, estimate.graph_revision)) != last_revision
                        && estimate.graph_revision > 0
                    {
                        let response = execute(
                            &client,
                            Operation::Snapshot {
                                session_id: current.session_id.clone(),
                                session_generation: current.session_generation,
                            },
                        )
                        .await?;
                        if let Some(snapshot) = response.snapshot {
                            let data = client.read_snapshot(&snapshot).await;
                            let released = execute(
                                &client,
                                Operation::ReleaseSnapshot {
                                    snapshot_id: snapshot.snapshot_id,
                                },
                            )
                            .await;
                            let data = data?;
                            released?;
                            router.publish_planar(id, grid(&settings, &snapshot, data)?);
                            last_revision = Some((id, estimate.graph_revision));
                        }
                    }
                }
            }
            .await;
            if let Err(error) = result {
                router.fault(error);
            }
        });
        *owner.task.lock().await = Some(task);
        Ok(owner)
    }
    pub(crate) async fn start_session(
        &self,
        id: u64,
        name: &str,
        checkpoint: Option<Vec<u8>>,
        localization: bool,
        seed: Option<proto::Pose>,
    ) -> Result<(), String> {
        self.start_session_inner(id, name, checkpoint, localization, seed)
            .await
            .map(|_| ())
    }
    async fn start_session_inner(
        &self,
        id: u64,
        name: &str,
        checkpoint: Option<Vec<u8>>,
        localization: bool,
        seed: Option<proto::Pose>,
    ) -> Result<Response, String> {
        self.stop_session().await?;
        let mut session = self.settings.template.clone();
        session.session_id = id.to_string();
        session.map_name = name.into();
        session.session_generation = id;
        let operation = if let Some(data) = checkpoint {
            let mut source = memory_file(&data)?;
            let (metadata, _) = nuwax_slam_2d_runtime::archive::read_structure(&mut source)?;
            source.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
            let artifact = self
                .client
                .stage_artifact(source, data.len() as u64, metadata.identity())
                .await?;
            Operation::LoadGraph {
                session: session.clone(),
                artifact,
                localization,
                seed: seed
                    .as_ref()
                    .map(planar_pose)
                    .transpose()?
                    .unwrap_or([0.0; 3]),
            }
        } else {
            Operation::Start {
                session: session.clone(),
            }
        };
        let response = execute(&self.client, operation).await?;
        *self.session.write() = Some(session);
        Ok(response)
    }
    pub(crate) async fn stop_session(&self) -> Result<(), String> {
        let session = self.session.write().take();
        if let Some(session) = session {
            execute(
                &self.client,
                Operation::Stop {
                    session_id: session.session_id,
                    session_generation: session.session_generation,
                },
            )
            .await?;
        }
        Ok(())
    }
    pub(crate) async fn publish_scan(&self, payload: &[u8]) -> Result<(), String> {
        let scan = decode_input(payload)?;
        let Some(session) = self.session.read().clone() else {
            return Ok(());
        };
        if let Err(error) = self
            .client
            .publish_scan(session.session_generation, &scan)
            .await
        {
            self.router.fault(error.clone());
            return Err(error);
        }
        Ok(())
    }
    pub(crate) async fn publish_odom(
        &self,
        request: proto::PublishOdomRequest,
    ) -> Result<(), String> {
        let value = request.pose.ok_or("二维里程计缺少位姿")?;
        if value.timestamp_ns == 0 {
            return Err("二维里程计需要真实采样时间".into());
        }
        let sample = nuwax_slam_2d_runtime::input::TimedPose {
            timestamp_ns: value.timestamp_ns,
            clock_domain: request.clock_domain,
            clock_id: request.clock_id,
            odom_frame_id: value.frame_id.clone(),
            body_frame_id: self.settings.template.body_frame_id.clone(),
            pose: planar_pose(&value)?,
        };
        if self.session.read().is_none() {
            return Ok(());
        }
        execute(&self.client, Operation::Odometry { sample }).await?;
        Ok(())
    }
    pub(crate) async fn checkpoint(&self) -> Result<Vec<u8>, String> {
        let session = self.session.read().clone().ok_or("二维会话尚未开始")?;
        let response = execute(
            &self.client,
            Operation::ExportGraph {
                session_id: session.session_id,
                session_generation: session.session_generation,
                unix_offset_ns: i64::try_from(self.settings.unix_offset_ns)
                    .map_err(|e| e.to_string())?,
            },
        )
        .await?;
        let artifact = response.artifact.ok_or("二维工作进程未返回图")?;
        let result: Result<Vec<u8>, String> = async {
            let mut file = self.client.open_artifact(&artifact).await?;
            let mut data = Vec::new();
            file.read_to_end(&mut data).map_err(|e| e.to_string())?;
            Ok(data)
        }
        .await;
        let released = execute(&self.client, Operation::ReleaseArtifact { artifact }).await;
        let data = result?;
        released?;
        Ok(data)
    }
    pub(crate) async fn relocalize(
        &self,
        seed: Option<proto::Pose>,
    ) -> Result<proto::RelocalizationResult, String> {
        let session = self.session.read().clone().ok_or("二维会话尚未开始")?;
        let checkpoint = self.checkpoint().await?;
        let response = self
            .start_session_inner(
                session
                    .session_id
                    .parse::<u64>()
                    .map_err(|e| e.to_string())?,
                &session.map_name,
                Some(checkpoint),
                true,
                seed,
            )
            .await?;
        let mut state = self.state.clone();
        let armed_unix_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_err(|error| error.to_string())?
            .as_nanos() as u64;
        let deadline = tokio::time::Instant::now() + Duration::from_secs(2);
        let mut accepted = 0;
        let mut previous = response
            .status
            .last_estimate
            .as_ref()
            .map(|estimate| estimate.source_sequence);
        let mut frames = 0;
        loop {
            match tokio::time::timeout_at(deadline, state.changed()).await {
                Err(_) => {
                    return Ok(proto::RelocalizationResult {
                        frames_used: frames,
                        message: "二维重定位超过2秒".into(),
                        ..Default::default()
                    })
                }
                Ok(result) => result.map_err(|e| e.to_string())?,
            }
            let estimate = state.borrow().last_estimate.clone();
            let Some(estimate) = estimate else { continue };
            if previous == Some(estimate.source_sequence) {
                continue;
            }
            previous = Some(estimate.source_sequence);
            frames += 1;
            let stamp = self.settings.unix_stamp(estimate.source_timestamp_ns)?;
            let now = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_err(|e| e.to_string())?
                .as_nanos() as u64;
            if estimate.disposition == "accepted"
                && estimate.matching_response >= 0.3
                && stamp >= armed_unix_ns
                && now.saturating_sub(stamp) <= 500_000_000
            {
                accepted += 1;
                if accepted >= 3 {
                    return Ok(proto::RelocalizationResult {
                        succeeded: true,
                        pose: Some(pose(estimate.map_from_odom, &session.map_frame_id, stamp)),
                        frames_used: frames,
                        message: "重定位成功".into(),
                        ..Default::default()
                    });
                }
            } else if estimate.disposition == "match_rejected" {
                accepted = 0;
            }
            if frames >= 8 {
                return Ok(proto::RelocalizationResult {
                    frames_used: frames,
                    message: "二维重定位8帧内未匹配成功".into(),
                    ..Default::default()
                });
            }
        }
    }
    pub(crate) async fn shutdown(&self) -> Result<(), String> {
        self.shutdown.send_replace(true);
        if let Some(task) = self.task.lock().await.take() {
            task.await.map_err(|e| e.to_string())?;
        }
        if let Some(mut worker) = self.worker.lock().await.take() {
            worker.shutdown().await?;
        }
        Ok(())
    }
}
async fn execute(client: &Client, operation: Operation) -> Result<Response, String> {
    let response = client.execute(operation).await?;
    if let Some(error) = &response.error {
        return Err(error.clone());
    }
    Ok(response)
}
fn planar_pose(value: &proto::Pose) -> Result<[f64; 3], String> {
    let iso = crate::lio_ingress::pose_isometry(value)?;
    let rotation = value.orientation.as_ref().ok_or("二维位姿缺少四元数")?;
    if rotation.x.abs() > 1.0e-5 || rotation.y.abs() > 1.0e-5 {
        return Err("二维位姿需要平面姿态".into());
    }
    Ok([
        iso.translation.x,
        iso.translation.y,
        iso.rotation.euler_angles().2,
    ])
}
fn pose(value: [f64; 3], frame: &str, stamp: u64) -> proto::Pose {
    proto::Pose {
        position: Some(proto::Vec3 {
            x: value[0],
            y: value[1],
            z: 0.0,
        }),
        orientation: Some(proto::Quat {
            x: 0.0,
            y: 0.0,
            z: (value[2] * 0.5).sin(),
            w: (value[2] * 0.5).cos(),
        }),
        frame_id: frame.into(),
        timestamp_ns: stamp,
    }
}
fn grid(
    settings: &Settings,
    snapshot: &Snapshot,
    data: Vec<i8>,
) -> Result<proto::OccupancyGrid, String> {
    Ok(proto::OccupancyGrid {
        map_name: snapshot.session.map_name.clone(),
        meta: Some(proto::OccupancyGridMeta {
            width: snapshot.width,
            height: snapshot.height,
            resolution: snapshot.resolution as f32,
            origin_x: snapshot.origin[0],
            origin_y: snapshot.origin[1],
            frame_id: snapshot.session.map_frame_id.clone(),
            timestamp_ns: settings.unix_stamp(snapshot.estimate.source_timestamp_ns)?,
        }),
        data: data.into_iter().map(|value| value as u8).collect(),
    })
}
fn memory_file(data: &[u8]) -> Result<std::fs::File, String> {
    use std::io::Write;
    use std::os::fd::FromRawFd;
    let fd = unsafe { libc::memfd_create(c"geometry-planar-load".as_ptr(), libc::MFD_CLOEXEC) };
    if fd < 0 {
        return Err(std::io::Error::last_os_error().to_string());
    }
    let mut file = unsafe { std::fs::File::from_raw_fd(fd) };
    file.write_all(data).map_err(|e| e.to_string())?;
    file.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
    Ok(file)
}
/// 公共v3扫描去掉运行代际；进入旧算法工作进程时转换其必需的内部格式。
fn decode_input(data: &[u8]) -> Result<nuwax_codec::lcm::planar::PlanarScan, String> {
    const FINGERPRINT: [u8; 8] = 0xdec4_0103_4b18_dbb5u64.to_be_bytes();
    if data.len() < 12
        || data.len() > nuwax_codec::lcm::planar::MAX_BYTES
        || data[..8] != FINGERPRINT
    {
        return Err("二维扫描格式或长度非法".into());
    }
    let mut internal = Vec::with_capacity(data.len() + 8);
    internal.extend_from_slice(&nuwax_codec::lcm::planar::FINGERPRINT);
    internal.extend_from_slice(&data[8..12]);
    internal.extend_from_slice(&1_i64.to_be_bytes());
    internal.extend_from_slice(&data[12..]);
    nuwax_codec::lcm::planar::decode(&internal).map_err(|e| e.to_string())
}

#[cfg(test)]
#[path = "slam2d_owner_tests.rs"]
mod tests;
