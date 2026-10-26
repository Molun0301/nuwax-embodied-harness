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
    written: Mutex<Option<(u64, u64)>>,
}
struct Context {
    root: PathBuf,
    path: PathBuf,
    session: Session,
    status: Arc<Status>,
}
/// 只持有会话身份和固定大小写入水位，不持有扫描点或后台拥有方。
#[derive(Clone)]
pub struct ReplaySource {
    ctx: Arc<Context>,
}
impl ReplaySource {
    pub fn session(&self) -> &Session {
        &self.ctx.session
    }
    pub fn path(&self) -> &Path {
        &self.ctx.path
    }
    /// 规范提交锁内只检查即时失败和丢帧状态，不访问文件系统。
    pub fn check_complete(&self) -> Result<(), String> {
        if self.ctx.status.failed.load(Ordering::Acquire) {
            return Err("原始扫描写盘已中断，不能用于完整重放".into());
        }
        if self.ctx.status.dropped.load(Ordering::Acquire) != 0 {
            return Err("原始扫描后台队列已经丢帧，不能用于完整重放".into());
        }
        Ok(())
    }
    /// 已提交地图所需的辅助记录交接失败时锁存失效；取消私有重建不调用此接口。
    pub fn mark_incomplete(&self) {
        self.ctx.status.failed.store(true, Ordering::Release);
    }
    pub fn snapshot(&self) -> Result<Option<ReplayPrefix>, String> {
        let complete = || -> Result<(), String> {
            self.check_complete()?;
            if self
                .ctx
                .path
                .with_extension("incomplete")
                .try_exists()
                .map_err(error)?
            {
                return Err("原始扫描会话已有不完整标记，不能用于完整重放".into());
            }
            Ok(())
        };
        complete()?;
        let written = *self.ctx.status.written.lock().map_err(error)?;
        // 复制水位之后再核对失败与丢帧，避免接受复制期间已失效的完整性。
        complete()?;
        Ok(
            written.map(|(valid_bytes, observed_until_ns)| ReplayPrefix {
                path: self.ctx.path.clone(),
                session: self.ctx.session.clone(),
                valid_bytes,
                observed_until_ns,
            }),
        )
    }
}
enum Command {
    #[cfg(test)]
    Pause(mpsc::Sender<()>, mpsc::Receiver<()>),
    Observe(Arc<Context>, ScanMetadata, Option<Vec<[f32; 3]>>),
    Poses(PathBuf, FinalPoses),
    SpooledPoses(PathBuf, FinalPoses, PoseSpool),
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
                                persist_drops(&ctx, &ctx.path);
                                continue;
                            }
                            let result = (|| {
                                if active.as_ref().is_none_or(|(current, _)| {
                                    current.root != ctx.root || current.session != ctx.session
                                }) {
                                    if let Some((previous, log)) = active.take() {
                                        if let Err(e) = log.sync() {
                                            previous.status.failed.store(true, Ordering::Release);
                                            let _ = std::fs::write(
                                                log.path.with_extension("incomplete"),
                                                &e,
                                            );
                                            return Err(e);
                                        }
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
                                // File 直接写入已经可被读取；关闭/切换时仍沿用既有同步，
                                // 不给在线路径增加逐帧 fsync（持久化同步）。
                                let valid_bytes = log.file.stream_position().map_err(error)?;
                                *ctx.status.written.lock().map_err(error)? =
                                    Some((valid_bytes, metadata.source_timestamp_ns));
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
                                fail_pose_save(&active, &root, &poses.session, &e);
                                eprintln!("最终位姿记录失败：{e}");
                            }
                        }
                        Ok(Command::SpooledPoses(root, metadata, spool)) => {
                            if let Err(e) =
                                pose_spool::save_spooled_final_poses(&root, &metadata, spool)
                            {
                                fail_pose_save(&active, &root, &metadata.session, &e);
                                eprintln!("流式最终位姿记录失败：{e}");
                            }
                        }
                        Ok(Command::Stop) | Err(mpsc::RecvTimeoutError::Disconnected) => break,
                        Err(mpsc::RecvTimeoutError::Timeout) => {}
                    }
                }
                if let Some((ctx, log)) = active {
                    persist_drops(&ctx, &log.path);
                    if let Err(e) = log.sync() {
                        ctx.status.failed.store(true, Ordering::Release);
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
        if persist_incomplete_if_absent(path, &format!("后台队列丢弃{n}帧，离线判定不完整"))
        {
            ctx.status.persisted_drops.store(n, Ordering::Release);
        }
    }
    if ctx.status.failed.load(Ordering::Acquire) {
        persist_incomplete_if_absent(path, "最终位姿或辅助记录交接失败，当前会话记录不完整");
    }
}
fn persist_incomplete_if_absent(path: &Path, text: &str) -> bool {
    let Some(root) = path.parent() else {
        return false;
    };
    if std::fs::create_dir_all(root).is_err() {
        return false;
    }
    match OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path.with_extension("incomplete"))
    {
        Ok(mut file) => file.write_all(text.as_bytes()).is_ok(),
        // 已有记录可能包含更具体的磁盘错误，定时收尾不覆盖它。
        Err(e) => e.kind() == std::io::ErrorKind::AlreadyExists,
    }
}
fn fail_pose_save(
    active: &Option<(Arc<Context>, ScanLog)>,
    root: &Path,
    session: &Session,
    text: &str,
) {
    if let Some((ctx, _)) = active {
        if ctx.root == root && &ctx.session == session {
            ctx.status.failed.store(true, Ordering::Release);
        }
    }
    persist_incomplete_if_absent(&session_path(root, session), text);
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
                path: session_path(&root, &session),
                root,
                session,
                status: Arc::new(Status::default()),
            }),
            policy: SavePolicy::from_env()?,
            previous: None,
        })
    }
    pub fn replay_source(&self) -> ReplaySource {
        ReplaySource {
            ctx: self.ctx.clone(),
        }
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
pub fn enqueue_spooled_final_poses(
    root: PathBuf,
    metadata: FinalPoses,
    spool: PoseSpool,
) -> Result<(), String> {
    if !metadata.poses.is_empty() || !spool.belongs_to(&root) {
        return Err("流式最终位姿必须使用同目录暂存及空位姿元数据".into());
    }
    let slot = WRITER.lock().map_err(error)?;
    let sender = slot.as_ref().ok_or("原始扫描后台拥有方未启动")?;
    sender
        .try_send(Command::SpooledPoses(root, metadata, spool))
        .map_err(|e| format!("流式最终位姿后台交接失败：{e}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    static TEST_WORKER: Mutex<()> = Mutex::new(());
    fn pause(worker: &StorageWorker) -> mpsc::Sender<()> {
        let (entered_tx, entered_rx) = mpsc::channel();
        let (release_tx, release_rx) = mpsc::channel();
        worker
            .sender
            .send(Command::Pause(entered_tx, release_rx))
            .unwrap();
        entered_rx.recv().unwrap();
        release_tx
    }
    fn folder(name: &str) -> PathBuf {
        let root =
            std::env::temp_dir().join(format!("neh-async-prefix-{}-{name}", std::process::id(),));
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).unwrap();
        root
    }
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
        let _guard = TEST_WORKER.lock().unwrap();
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
        let source = recorder.replay_source();
        assert!(recorder
            .observe(metadata(1, 0., 0.), [[1., 0., 0.]].into_iter())
            .unwrap());
        assert!(!recorder
            .observe(metadata(2, 1., 0.), [[2., 0., 0.]].into_iter())
            .unwrap());
        assert_eq!(recorder.dropped_frames(), 1);
        assert!(source.snapshot().unwrap_err().contains("丢帧"));
        release_tx.send(()).unwrap();
        drop(worker);
        assert!(source.snapshot().is_err());
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
    #[test]
    fn replay_prefix_waits_for_writer_and_extends_without_replaying_history() {
        let _guard = TEST_WORKER.lock().unwrap();
        let root = folder("incremental");
        let worker = StorageWorker::start_with_capacity(4).unwrap();
        let mut recorder = Recorder::new(
            root.clone(),
            Session {
                session_id: "prefix".into(),
                generation: 1,
            },
        )
        .unwrap();
        let source = recorder.replay_source();
        let release = pause(&worker);
        recorder
            .observe(metadata(10, 0., 0.), [[1., 0., 0.]].into_iter())
            .unwrap();
        assert!(source.snapshot().unwrap().is_none(), "队列中的帧尚不可重放");
        release.send(()).unwrap();
        let release = pause(&worker);
        let first = source.snapshot().unwrap().unwrap();
        assert_eq!(first.observed_until_ns(), 10);
        let mut reader = ScanReader::open_prefix(&first).unwrap();
        assert_eq!(reader.next_scan().unwrap().unwrap().points, [[1., 0., 0.]]);
        assert!(reader.next_scan().unwrap().is_none());
        assert!(recorder
            .observe(metadata(20, 1., 0.), [[2., 0., 0.]].into_iter())
            .unwrap());
        assert!(!recorder
            .observe(metadata(30, 1., 0.), [[3., 0., 0.]].into_iter())
            .unwrap());
        assert_eq!(
            source.snapshot().unwrap().unwrap(),
            first,
            "水位不能越过暂停的写线程"
        );
        release.send(()).unwrap();
        let release = pause(&worker);
        let next = source.snapshot().unwrap().unwrap();
        assert!(next.valid_bytes() > first.valid_bytes());
        assert_eq!(next.observed_until_ns(), 30, "未抽中帧也要发布源时间进度");
        assert!(
            reader.next_scan().unwrap().is_none(),
            "旧快照仍然固定在旧边界"
        );
        reader.extend_prefix(&next).unwrap();
        assert_eq!(reader.next_scan().unwrap().unwrap().points, [[2., 0., 0.]]);
        assert!(reader.next_scan().unwrap().is_none());
        assert_eq!(reader.observed_until_ns, 30);
        assert!(reader.extend_prefix(&first).unwrap_err().contains("回退"));
        release.send(()).unwrap();
        drop(worker);
        assert_eq!(source.snapshot().unwrap().unwrap(), next);
        std::fs::remove_dir_all(root).unwrap();
    }
    #[test]
    fn replay_source_rejects_existing_marker_budget_exhaustion_and_writer_failure() {
        let _guard = TEST_WORKER.lock().unwrap();
        let root = folder("failures");
        let worker = StorageWorker::start_with_capacity(4).unwrap();
        let mut recorder = Recorder::new(
            root.clone(),
            Session {
                session_id: "budget".into(),
                generation: 1,
            },
        )
        .unwrap();
        let source = recorder.replay_source();
        recorder
            .observe(metadata(10, 0., 0.), [[1., 0., 0.]].into_iter())
            .unwrap();
        let release = pause(&worker);
        assert!(source.snapshot().unwrap().is_some());
        let blocker = root.join("budget-blocker");
        File::create(&blocker)
            .unwrap()
            .set_len(storage_budget_bytes().unwrap())
            .unwrap();
        recorder
            .observe(metadata(20, 1., 0.), [[2., 0., 0.]].into_iter())
            .unwrap();
        release.send(()).unwrap();
        let release = pause(&worker);
        assert!(source.snapshot().unwrap_err().contains("中断"));
        assert!(source.path().with_extension("incomplete").exists());
        std::fs::remove_file(blocker).unwrap();
        release.send(()).unwrap();

        let mut invalid = Recorder::new(
            root.clone(),
            Session {
                session_id: "invalid".into(),
                generation: 1,
            },
        )
        .unwrap();
        let source = invalid.replay_source();
        invalid
            .observe(metadata(10, 0., 0.), [[f32::NAN, 0., 0.]].into_iter())
            .unwrap();
        let release = pause(&worker);
        assert!(source.snapshot().unwrap_err().contains("中断"));
        release.send(()).unwrap();

        let untouched = Recorder::new(
            root.clone(),
            Session {
                session_id: "existing-marker".into(),
                generation: 1,
            },
        )
        .unwrap()
        .replay_source();
        std::fs::write(untouched.path().with_extension("incomplete"), "已有中断").unwrap();
        assert!(untouched.snapshot().unwrap_err().contains("已有不完整"));
        drop(worker);
        std::fs::remove_dir_all(root).unwrap();
    }
    #[test]
    fn spooled_pose_handoff_cleans_rejected_queue_and_worker_shutdown_resources() {
        let _guard = TEST_WORKER.lock().unwrap();
        let root = folder("spooled-handoff");
        let session = Session {
            session_id: "spooled-handoff".into(),
            generation: 1,
        };
        let poses = || FinalPoses {
            session: session.clone(),
            cloud_revision: 3,
            frame_generation: Some(1),
            poses: vec![],
        };
        let worker = StorageWorker::start_with_capacity(1).unwrap();
        let release = pause(&worker);
        let mut recorder = Recorder::new(root.clone(), session.clone()).unwrap();
        recorder
            .observe(metadata(10, 0., 0.), [[1., 0., 0.]].into_iter())
            .unwrap();
        let rejected = PoseSpool::new(root.clone()).unwrap();
        let rejected_path = rejected.path().to_path_buf();
        assert!(enqueue_spooled_final_poses(root.clone(), poses(), rejected)
            .unwrap_err()
            .contains("后台交接失败"));
        assert!(!rejected_path.exists());
        release.send(()).unwrap();
        let release = pause(&worker);
        let mut accepted = PoseSpool::new(root.clone()).unwrap();
        accepted
            .append(10, metadata(10, 1., 0.).odom_from_body)
            .unwrap();
        let accepted_path = accepted.path().to_path_buf();
        enqueue_spooled_final_poses(root.clone(), poses(), accepted).unwrap();
        assert!(accepted_path.exists(), "后台未处理前由队列拥有暂存");
        release.send(()).unwrap();
        drop(worker);
        assert!(!accepted_path.exists());
        let json = root
            .read_dir()
            .unwrap()
            .map(|entry| entry.unwrap().path())
            .find(|path| {
                path.extension()
                    .is_some_and(|extension| extension == "json")
            })
            .unwrap();
        let actual: FinalPoses = serde_json::from_slice(&std::fs::read(json).unwrap()).unwrap();
        assert_eq!(
            actual.poses,
            vec![(10, metadata(10, 1., 0.).odom_from_body)]
        );
        let stopped = PoseSpool::new(root.clone()).unwrap();
        let stopped_path = stopped.path().to_path_buf();
        assert!(enqueue_spooled_final_poses(root.clone(), poses(), stopped)
            .unwrap_err()
            .contains("未启动"));
        assert!(!stopped_path.exists());
        std::fs::remove_dir_all(root).unwrap();
    }
    #[test]
    fn second_pose_handoff_failure_invalidates_session_even_when_old_poses_remain() {
        let _guard = TEST_WORKER.lock().unwrap();
        let root = folder("second-pose-handoff");
        let session = Session {
            session_id: "second-pose-handoff".into(),
            generation: 1,
        };
        let poses = |revision| FinalPoses {
            session: session.clone(),
            cloud_revision: revision,
            frame_generation: Some(1),
            poses: vec![],
        };
        let worker = StorageWorker::start_with_capacity(1).unwrap();
        let mut recorder = Recorder::new(root.clone(), session.clone()).unwrap();
        let source = recorder.replay_source();
        recorder
            .observe(metadata(10, 0., 0.), [[1., 0., 0.]].into_iter())
            .unwrap();
        let release = pause(&worker);
        let mut first = PoseSpool::new(root.clone()).unwrap();
        first
            .append(10, metadata(10, 1., 0.).odom_from_body)
            .unwrap();
        enqueue_spooled_final_poses(root.clone(), poses(1), first).unwrap();
        release.send(()).unwrap();
        let release = pause(&worker);
        let stem = source.path().file_stem().unwrap().to_string_lossy();
        let old_poses = root.join(format!("{stem}-poses-1.json"));
        assert!(old_poses.exists());
        assert!(source.check_complete().is_ok());
        let cancelled = PoseSpool::new(root.clone()).unwrap();
        drop(cancelled);
        assert!(
            source.check_complete().is_ok(),
            "取消私有重建不使原记录失效"
        );

        recorder
            .observe(metadata(20, 1., 0.), [[2., 0., 0.]].into_iter())
            .unwrap();
        let mut second = PoseSpool::new(root.clone()).unwrap();
        second
            .append(20, metadata(20, 2., 0.).odom_from_body)
            .unwrap();
        let second_path = second.path().to_path_buf();
        assert!(enqueue_spooled_final_poses(root.clone(), poses(2), second).is_err());
        assert!(!second_path.exists());
        source.mark_incomplete();
        assert!(source.check_complete().unwrap_err().contains("中断"));
        let marker = source.path().with_extension("incomplete");
        assert!(!marker.exists(), "原子锁存不在调用线程访问文件系统");
        release.send(()).unwrap();
        let release = pause(&worker);
        assert!(marker.exists());
        assert!(std::fs::read_to_string(&marker)
            .unwrap()
            .contains("最终位姿"));
        std::fs::write(&marker, "应保留的具体磁盘错误").unwrap();
        release.send(()).unwrap();
        drop(worker);
        assert_eq!(
            std::fs::read_to_string(&marker).unwrap(),
            "应保留的具体磁盘错误"
        );
        let old: FinalPoses = serde_json::from_slice(&std::fs::read(old_poses).unwrap()).unwrap();
        assert_eq!(old.cloud_revision, 1);
        assert_eq!(old.frame_generation, Some(1));
        assert!(source.snapshot().is_err());
        assert!(ScanLog::open(&root, session)
            .err()
            .unwrap()
            .contains("此前已经中断"));
        std::fs::remove_dir_all(root).unwrap();
    }
    #[test]
    fn background_pose_save_failure_marks_only_the_matching_active_session() {
        let _guard = TEST_WORKER.lock().unwrap();
        let root = folder("pose-save-failure");
        let first_session = Session {
            session_id: "first".into(),
            generation: 1,
        };
        let second_session = Session {
            session_id: "second".into(),
            generation: 2,
        };
        let poses = |session: &Session, revision| FinalPoses {
            session: session.clone(),
            cloud_revision: revision,
            frame_generation: Some(1),
            poses: vec![],
        };
        let worker = StorageWorker::start_with_capacity(4).unwrap();
        let mut first = Recorder::new(root.clone(), first_session.clone()).unwrap();
        let first_source = first.replay_source();
        first
            .observe(metadata(10, 0., 0.), [[1., 0., 0.]].into_iter())
            .unwrap();
        let release = pause(&worker);
        let blocker = root.join("budget-blocker");
        File::create(&blocker)
            .unwrap()
            .set_len(storage_budget_bytes().unwrap())
            .unwrap();
        enqueue_final_poses(root.clone(), poses(&first_session, 1)).unwrap();
        release.send(()).unwrap();
        let release = pause(&worker);
        assert!(
            first_source.check_complete().is_err(),
            "旧式位姿保存错误也要即时撤销完整性"
        );
        let first_marker = first_source.path().with_extension("incomplete");
        assert!(std::fs::read_to_string(&first_marker)
            .unwrap()
            .contains("预算"));
        std::fs::remove_file(&blocker).unwrap();
        release.send(()).unwrap();

        let mut second = Recorder::new(root.clone(), second_session.clone()).unwrap();
        let second_source = second.replay_source();
        second
            .observe(metadata(10, 0., 0.), [[2., 0., 0.]].into_iter())
            .unwrap();
        let release = pause(&worker);
        assert!(second_source.check_complete().is_ok());
        let mut stale_spool = PoseSpool::new(root.clone()).unwrap();
        stale_spool
            .append(10, metadata(10, 1., 0.).odom_from_body)
            .unwrap();
        let stale_path = stale_spool.path().to_path_buf();
        let mut current_spool = PoseSpool::new(root.clone()).unwrap();
        current_spool
            .append(10, metadata(10, 2., 0.).odom_from_body)
            .unwrap();
        let current_path = current_spool.path().to_path_buf();
        std::fs::write(&first_marker, "旧会话具体磁盘错误").unwrap();
        File::create(&blocker)
            .unwrap()
            .set_len(storage_budget_bytes().unwrap())
            .unwrap();
        enqueue_final_poses(root.clone(), poses(&first_session, 2)).unwrap();
        enqueue_spooled_final_poses(root.clone(), poses(&first_session, 3), stale_spool).unwrap();
        release.send(()).unwrap();
        let release = pause(&worker);
        assert!(
            second_source.check_complete().is_ok(),
            "旧会话两种保存错误均不能污染新会话"
        );
        assert!(!stale_path.exists());
        assert_eq!(
            std::fs::read_to_string(&first_marker).unwrap(),
            "旧会话具体磁盘错误"
        );
        enqueue_spooled_final_poses(root.clone(), poses(&second_session, 1), current_spool)
            .unwrap();
        release.send(()).unwrap();
        let release = pause(&worker);
        assert!(
            second_source.check_complete().is_err(),
            "流式位姿保存错误要即时撤销对应完整性"
        );
        assert!(!current_path.exists());
        let second_marker = second_source.path().with_extension("incomplete");
        assert!(std::fs::read_to_string(&second_marker)
            .unwrap()
            .contains("预算"));
        release.send(()).unwrap();
        drop(worker);
        assert_eq!(
            std::fs::read_to_string(&first_marker).unwrap(),
            "旧会话具体磁盘错误"
        );
        assert!(std::fs::read_to_string(&second_marker)
            .unwrap()
            .contains("预算"));
        std::fs::remove_dir_all(root).unwrap();
    }
}
