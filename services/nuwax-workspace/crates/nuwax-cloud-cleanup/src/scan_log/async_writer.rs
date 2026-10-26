//! 根拥有方管理单个写盘线程；扫描回调只进行有界非阻塞交接。
use super::*;
use std::sync::{
    atomic::{AtomicBool, AtomicU64, Ordering},
    mpsc::{self, SyncSender, TrySendError},
    Arc, Mutex,
};
use std::thread::{self, JoinHandle};

#[derive(Clone, Copy, Debug)]
pub struct SavePolicy {
    pub translation_m: f64,
    pub rotation_deg: f64,
    pub interval_s: f64,
}
impl Default for SavePolicy {
    fn default() -> Self {
        Self {
            translation_m: 0.1,
            rotation_deg: 3.0,
            interval_s: 1.0,
        }
    }
}
impl SavePolicy {
    pub fn from_env() -> Result<Self, String> {
        let number = |name: &str, default: f64, max: f64| {
            let text = std::env::var(name).map_err(|e| match e {
                std::env::VarError::NotPresent => String::new(),
                _ => format!("{name} 必须是有效文本"),
            });
            let text = match text {
                Ok(t) => t,
                Err(t) if t.is_empty() => default.to_string(),
                Err(e) => return Err(e),
            };
            let n = if text.trim().is_empty() {
                default
            } else {
                text.trim()
                    .parse::<f64>()
                    .map_err(|_| format!("{name} 必须是数值"))?
            };
            if n.is_finite() && (0.0..=max).contains(&n) {
                Ok(n)
            } else {
                Err(format!("{name} 必须在0～{max}之间"))
            }
        };
        Ok(Self {
            translation_m: number("NUWAX_RAW_SCAN_TRANSLATION_M", 0.1, 100.0)?,
            rotation_deg: number("NUWAX_RAW_SCAN_ROTATION_DEG", 3.0, 180.0)?,
            interval_s: number("NUWAX_RAW_SCAN_INTERVAL_S", 1.0, 3600.0)?,
        })
    }
    pub fn selects(&self, previous: Option<&ScanMetadata>, current: &ScanMetadata) -> bool {
        let Some(p) = previous else {
            return true;
        };
        let a = nalgebra::Matrix4::from_column_slice(&p.odom_from_body);
        let b = nalgebra::Matrix4::from_column_slice(&current.odom_from_body);
        let distance = (a.fixed_view::<3, 1>(0, 3) - b.fixed_view::<3, 1>(0, 3)).norm();
        let relative = a.fixed_view::<3, 3>(0, 0).transpose() * b.fixed_view::<3, 3>(0, 0);
        let angle = ((relative.trace() - 1.0) * 0.5)
            .clamp(-1.0, 1.0)
            .acos()
            .to_degrees();
        distance >= self.translation_m
            || angle >= self.rotation_deg
            || current
                .source_timestamp_ns
                .saturating_sub(p.source_timestamp_ns) as f64
                >= self.interval_s * 1e9
    }
}
fn queue_capacity() -> Result<usize, String> {
    let v = match std::env::var("NUWAX_RAW_SCAN_QUEUE_FRAMES") {
        Ok(v) => v,
        Err(std::env::VarError::NotPresent) => "16".into(),
        Err(_) => return Err("NUWAX_RAW_SCAN_QUEUE_FRAMES 必须是有效文本".into()),
    };
    let v = if v.trim().is_empty() { "16" } else { v.trim() };
    v.bytes()
        .all(|b| b.is_ascii_digit())
        .then(|| v.parse::<usize>().ok())
        .flatten()
        .filter(|n| (1..=256).contains(n))
        .ok_or_else(|| "NUWAX_RAW_SCAN_QUEUE_FRAMES 必须是1～256的整数".into())
}
#[derive(Default)]
struct Status {
    dropped: AtomicU64,
    persisted_drops: AtomicU64,
    failed: AtomicBool,
}
struct Context {
    root: PathBuf,
    session: Session,
    status: Arc<Status>,
}
enum Command {
    #[cfg(test)]
    Pause(mpsc::Sender<()>, mpsc::Receiver<()>),
    Observe(Arc<Context>, ScanMetadata, Option<Vec<[f32; 3]>>),
    Poses(PathBuf, FinalPoses),
    Stop,
}
static WRITER: Mutex<Option<SyncSender<Command>>> = Mutex::new(None);
pub struct StorageWorker {
    sender: SyncSender<Command>,
    thread: Option<JoinHandle<()>>,
}
impl StorageWorker {
    pub fn validate_config() -> Result<(), String> {
        storage_budget_bytes()?;
        SavePolicy::from_env()?;
        queue_capacity()?;
        Ok(())
    }
    pub fn start() -> Result<Self, String> {
        Self::validate_config()?;
        Self::start_with_capacity(queue_capacity()?)
    }
    fn start_with_capacity(capacity: usize) -> Result<Self, String> {
        let mut slot = WRITER.lock().map_err(error)?;
        if slot.is_some() {
            return Err("原始扫描后台拥有方已启动".into());
        }
        let (sender, receiver) = mpsc::sync_channel(capacity);
        let thread = thread::Builder::new()
            .name("raw-scan-writer".into())
            .spawn(move || {
                let mut active: Option<(Arc<Context>, ScanLog)> = None;
                loop {
                    let command = receiver.recv_timeout(std::time::Duration::from_millis(100));
                    if let Some((ctx, log)) = &active {
                        persist_drops(ctx, &log.path);
                    }
                    match command {
                        #[cfg(test)]
                        Ok(Command::Pause(entered, release)) => {
                            entered.send(()).unwrap();
                            release.recv().unwrap();
                        }

                        Ok(Command::Observe(ctx, metadata, points)) => {
                            if ctx.status.failed.load(Ordering::Acquire) {
                                continue;
                            }
                            let result = (|| {
                                if active.as_ref().is_none_or(|(current, _)| {
                                    current.root != ctx.root || current.session != ctx.session
                                }) {
                                    if let Some((_, log)) = active.take() {
                                        log.sync()?;
                                    }
                                    let log = ScanLog::open(&ctx.root, ctx.session.clone())?;
                                    active = Some((ctx.clone(), log));
                                }
                                let (_, log) = active.as_mut().unwrap();
                                if let Some(points) = points {
                                    log.append(&metadata, points.into_iter(), None)?;
                                }
                                log.progress(
                                    metadata.source_timestamp_ns,
                                    ctx.status.dropped.load(Ordering::Acquire),
                                )?;
                                persist_drops(&ctx, &log.path);
                                Ok::<_, String>(())
                            })();
                            if let Err(e) = result {
                                ctx.status.failed.store(true, Ordering::Release);
                                let path = session_path(&ctx.root, &ctx.session)
                                    .with_extension("incomplete");
                                let _ = std::fs::write(path, &e);
                                eprintln!("原始扫描记录中断，在线建图继续：{e}");
                            }
                        }
                        Ok(Command::Poses(root, poses)) => {
                            if let Err(e) = save_final_poses(&root, &poses) {
                                let _ = std::fs::write(
                                    session_path(&root, &poses.session)
                                        .with_extension("incomplete"),
                                    &e,
                                );
                                eprintln!("最终位姿记录失败：{e}");
                            }
                        }
                        Ok(Command::Stop) | Err(mpsc::RecvTimeoutError::Disconnected) => break,
                        Err(mpsc::RecvTimeoutError::Timeout) => {}
                    }
                }
                if let Some((ctx, log)) = active {
                    persist_drops(&ctx, &log.path);
                    if let Err(e) = log.sync() {
                        let _ = std::fs::write(log.path.with_extension("incomplete"), &e);
                        eprintln!("原始扫描关闭同步失败：{e}");
                    }
                }
            })
            .map_err(error)?;
        *slot = Some(sender.clone());
        Ok(Self {
            sender,
            thread: Some(thread),
        })
    }
}
impl Drop for StorageWorker {
    fn drop(&mut self) {
        // 先撤销新交接，再让已排队任务结束；根拥有方在在线资源退出后回收线程。
        if let Ok(mut slot) = WRITER.lock() {
            *slot = None;
        }
        let _ = self.sender.send(Command::Stop);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}
fn persist_drops(ctx: &Context, path: &Path) {
    let n = ctx.status.dropped.load(Ordering::Acquire);
    if n > ctx.status.persisted_drops.load(Ordering::Acquire) {
        if std::fs::write(
            path.with_extension("incomplete"),
            format!("后台队列丢弃{n}帧，离线判定不完整"),
        )
        .is_ok()
        {
            ctx.status.persisted_drops.store(n, Ordering::Release);
        }
    }
}
pub struct Recorder {
    sender: SyncSender<Command>,
    ctx: Arc<Context>,
    policy: SavePolicy,
    previous: Option<ScanMetadata>,
}
impl Recorder {
    pub fn new(root: PathBuf, session: Session) -> Result<Self, String> {
        let sender = WRITER
            .lock()
            .map_err(error)?
            .as_ref()
            .ok_or("原始扫描后台拥有方未启动")?
            .clone();
        Ok(Self {
            sender,
            ctx: Arc::new(Context {
                root,
                session,
                status: Arc::new(Status::default()),
            }),
            policy: SavePolicy::from_env()?,
            previous: None,
        })
    }
    pub fn observe(
        &mut self,
        metadata: ScanMetadata,
        points: impl ExactSizeIterator<Item = [f32; 3]>,
    ) -> Result<bool, String> {
        if self.ctx.status.failed.load(Ordering::Acquire) {
            return Err("原始扫描写盘已中断".into());
        }
        let selected = self.policy.selects(self.previous.as_ref(), &metadata);
        let payload = selected.then(|| points.collect());
        match self.sender.try_send(Command::Observe(
            self.ctx.clone(),
            metadata.clone(),
            payload,
        )) {
            Ok(()) => {
                if selected {
                    self.previous = Some(metadata);
                }
                Ok(selected)
            }
            Err(TrySendError::Full(_)) => {
                self.ctx.status.dropped.fetch_add(1, Ordering::AcqRel);
                Ok(false)
            }
            Err(TrySendError::Disconnected(_)) => {
                self.ctx.status.failed.store(true, Ordering::Release);
                Err("原始扫描后台拥有方已退出".into())
            }
        }
    }
    pub fn dropped_frames(&self) -> u64 {
        self.ctx.status.dropped.load(Ordering::Acquire)
    }
}
pub fn enqueue_final_poses(root: PathBuf, poses: FinalPoses) -> Result<(), String> {
    let slot = WRITER.lock().map_err(error)?;
    let sender = slot.as_ref().ok_or("原始扫描后台拥有方未启动")?;
    sender
        .try_send(Command::Poses(root, poses))
        .map_err(|e| format!("最终位姿后台交接失败：{e}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    fn metadata(stamp: u64, x: f64, angle: f64) -> ScanMetadata {
        let pose = nalgebra::Isometry3::new(
            nalgebra::Vector3::new(x, 0., 0.),
            nalgebra::Vector3::new(0., 0., angle.to_radians()),
        );
        ScanMetadata {
            source_timestamp_ns: stamp,
            source_sequence: stamp,
            sensor: "lidar".into(),
            source_generation: 1,
            odom_from_body: pose.to_homogeneous().as_slice().try_into().unwrap(),
            map_from_odom: nalgebra::Matrix4::identity().as_slice().try_into().unwrap(),
            sensor_origin_body: Some([0.28945, 0., -0.046825]),
            observation_minimum_range_m: None,
            point_count: 1,
        }
    }
    #[test]
    fn policy_uses_full_pose_or_source_interval_from_last_selected_frame() {
        let p = SavePolicy::default();
        let start = metadata(1, 0., 0.);
        assert!(p.selects(None, &start));
        assert!(!p.selects(Some(&start), &metadata(500_000_001, 0.09, 2.9)));
        assert!(p.selects(Some(&start), &metadata(500_000_001, 0.101, 0.)));
        assert!(p.selects(Some(&start), &metadata(500_000_001, 0., 3.01)));
        assert!(p.selects(Some(&start), &metadata(1_000_000_001, 0., 0.)));
    }
    #[test]
    fn full_queue_drops_new_frame_without_waiting_and_persists_incomplete() {
        let root = std::env::temp_dir().join(format!("neh-async-test-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        let worker = StorageWorker::start_with_capacity(1).unwrap();
        let (entered_tx, entered_rx) = mpsc::channel();
        let (release_tx, release_rx) = mpsc::channel();
        worker
            .sender
            .send(Command::Pause(entered_tx, release_rx))
            .unwrap();
        entered_rx.recv().unwrap();
        let session = Session {
            session_id: "queue".into(),
            generation: 1,
        };
        let mut recorder = Recorder::new(root.clone(), session.clone()).unwrap();
        assert!(recorder
            .observe(metadata(1, 0., 0.), [[1., 0., 0.]].into_iter())
            .unwrap());
        assert!(!recorder
            .observe(metadata(2, 1., 0.), [[2., 0., 0.]].into_iter())
            .unwrap());
        assert_eq!(recorder.dropped_frames(), 1);
        release_tx.send(()).unwrap();
        drop(worker);
        let path = session_path(&root, &session);
        assert!(path.with_extension("incomplete").exists());
        let mut reader = ScanReader::open(&path).unwrap();
        assert_eq!(reader.next_scan().unwrap().unwrap().points, [[1., 0., 0.]]);
        assert!(reader.next_scan().unwrap().is_none());
        assert_eq!(reader.dropped_frames, 1);
        std::fs::remove_dir_all(&root).unwrap();
        let worker = StorageWorker::start_with_capacity(16).unwrap();
        let session = Session {
            session_id: "sampled".into(),
            generation: 2,
        };
        let mut recorder = Recorder::new(root.clone(), session.clone()).unwrap();
        assert!(recorder
            .observe(metadata(10, 0., 0.), [[1., 0., 0.]].into_iter())
            .unwrap());
        assert!(!recorder
            .observe(metadata(20, 0., 0.), [[2., 0., 0.]].into_iter())
            .unwrap());
        enqueue_final_poses(
            root.clone(),
            FinalPoses {
                session: session.clone(),
                cloud_revision: 3,
                frame_generation: Some(2),
                poses: vec![(10, metadata(10, 1., 0.).odom_from_body)],
            },
        )
        .unwrap();
        drop(worker);
        let path = session_path(&root, &session);
        assert!(!path.with_extension("incomplete").exists());
        let mut reader = ScanReader::open(&path).unwrap();
        assert_eq!(
            reader
                .next_scan()
                .unwrap()
                .unwrap()
                .metadata
                .source_timestamp_ns,
            10
        );
        assert!(reader.next_scan().unwrap().is_none());
        assert_eq!(reader.observed_until_ns, 20);
        assert!(root.read_dir().unwrap().any(|v| v
            .unwrap()
            .path()
            .extension()
            .is_some_and(|e| e == "json")));
        std::fs::remove_dir_all(root).unwrap();
    }
}
