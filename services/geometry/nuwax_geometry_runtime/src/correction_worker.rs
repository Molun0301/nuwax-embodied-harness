//! 会话校正拥有方：最新输入槽、后台重放和有界尾部交接。
use super::*;
use crate::corrector::CorrectionInput;
use crate::pointcloud::{ReplayFrames, ReplayPosition, ScanPrefix};
use std::sync::atomic::{AtomicBool, Ordering};

pub(super) enum Reply {
    Adopted,
    Replay(Vec<ScanPrefix>),
    CatchUp(Box<Rebuilt>, Vec<ScanPrefix>),
}
pub(super) enum Update {
    Correction(PgoUpdate, mpsc::Sender<Reply>),
    Prefix(mpsc::Sender<Vec<ScanPrefix>>),
    Rebuilt(Box<Rebuilt>, mpsc::Sender<Reply>),
    Stopped(String),
}
struct Inbox {
    input: parking_lot::Mutex<Option<CorrectionInput>>,
    ready: parking_lot::Condvar,
    stop: AtomicBool,
    #[cfg(test)]
    probe: Arc<ReplayProbe>,
}
#[cfg(test)]
#[derive(Default)]
struct ReplayProbe {
    frames: std::sync::atomic::AtomicUsize,
    paused: parking_lot::Mutex<bool>,
    changed: parking_lot::Condvar,
}
pub(super) struct CorrectionWorker {
    inbox: Arc<Inbox>,
    updates: mpsc::Receiver<Update>,
    worker: Option<std::thread::JoinHandle<()>>,
}
impl CorrectionWorker {
    pub fn start(session_id: u64, config: MapperConfig, corrector: Corrector) -> Result<Self, String> {
        let inbox = Arc::new(Inbox {
            input: parking_lot::Mutex::new(None), ready: parking_lot::Condvar::new(),
            stop: AtomicBool::new(false),
            #[cfg(test)]
            probe: Arc::new(ReplayProbe::default()),
        });
        let (tx, updates) = mpsc::channel();
        let source = inbox.clone();
        let worker = std::thread::Builder::new().name(format!("geometry-correct-{session_id}"))
            .spawn(move || {
                if let Err(reason) = run(session_id, config, corrector, &source, &tx) {
                    let _ = tx.send(Update::Stopped(reason));
                }
            }).map_err(|e| e.to_string())?;
        Ok(Self { inbox, updates, worker: Some(worker) })
    }
    pub fn submit(&self, input: CorrectionInput) {
        if self.inbox.stop.load(Ordering::Acquire) { return; }
        *self.inbox.input.lock() = Some(input);
        self.inbox.ready.notify_one();
    }
    pub fn next(&self) -> Option<Update> {
        if self.inbox.stop.load(Ordering::Acquire) { return None; }
        self.updates.try_recv().ok()
    }
    pub fn request_stop(&self) {
        self.inbox.stop.store(true, Ordering::Release);
        self.inbox.input.lock().take();
        while self.updates.try_recv().is_ok() {}
        self.inbox.ready.notify_all();
        #[cfg(test)]
        self.inbox.probe.changed.notify_all();
    }
    pub fn shutdown(&mut self) {
        self.request_stop();
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}
impl Drop for CorrectionWorker {
    fn drop(&mut self) { self.shutdown(); }
}
fn wait<T>(rx: mpsc::Receiver<T>, source: &Inbox) -> Option<T> {
    loop {
        if source.stop.load(Ordering::Acquire) { return None; }
        match rx.recv_timeout(Duration::from_millis(5)) {
            Ok(value) => return Some(value),
            Err(mpsc::RecvTimeoutError::Timeout) => {},
            Err(mpsc::RecvTimeoutError::Disconnected) => return None,
        }
    }
}
fn run(session_id: u64, config: MapperConfig, mut corrector: Corrector,
    source: &Inbox, updates: &mpsc::Sender<Update>) -> Result<(), String> {
    loop {
        let input = {
            let mut pending = source.input.lock();
            while pending.is_none() && !source.stop.load(Ordering::Acquire) {
                source.ready.wait(&mut pending);
            }
            if source.stop.load(Ordering::Acquire) { return Ok(()); }
            pending.take().unwrap()
        };
        let Some(update) = corrector.process_frame(&input)? else { continue };
        let (tx, rx) = mpsc::channel();
        if updates.send(Update::Correction(update.clone(), tx)).is_err() { return Ok(()); }
        let Some(reply) = wait(rx, source) else { return Ok(()) };
        let Reply::Replay(mut prefixes) = reply else { continue };
        let mut rebuilt = Rebuilt::new(session_id, config.clone(), update)?;
        #[cfg(test)]
        { rebuilt.probe = Some(source.probe.clone()); }
        loop {
            if !rebuilt.replay(&prefixes, &source.stop)? { return Ok(()); }
            let (tx, rx) = mpsc::channel();
            if updates.send(Update::Prefix(tx)).is_err() { return Ok(()); }
            let Some(latest) = wait(rx, source) else { return Ok(()) };
            if rebuilt.position.remaining(&latest) > 5 {
                prefixes = latest;
                continue;
            }
            let (tx, rx) = mpsc::channel();
            if updates.send(Update::Rebuilt(Box::new(rebuilt), tx)).is_err() { return Ok(()); }
            match wait(rx, source) {
                Some(Reply::Adopted) | None => break,
                Some(Reply::CatchUp(value, latest)) => { rebuilt = *value; prefixes = latest; },
                Some(Reply::Replay(_)) => unreachable!("交回新地图答复采用或继续追赶"),
            }
        }
    }
}
pub(super) struct Rebuilt {
    pub cloud: CloudMap,
    pub kernel: MappingKernel,
    pub grid: Option<PersistentOccupancyGrid>,
    pub stamp: u64,
    pub update: PgoUpdate,
    pub position: ReplayPosition,
    config: MapperConfig,
    sequence: u64,
    #[cfg(test)]
    probe: Option<Arc<ReplayProbe>>,
}
impl Rebuilt {
    pub fn new(session_id: u64, config: MapperConfig, update: PgoUpdate) -> Result<Self, String> {
        let mut cloud = CloudMap::new(session_id, None, None)?;
        cloud.disable_recording();
        Ok(Self { cloud, kernel: config.backend.into_kernel(config.compute.window_capacity),
            grid: None, stamp: 0, update, position: ReplayPosition::default(), config, sequence: 0,
            #[cfg(test)] probe: None,
        })
    }
    pub fn replay(&mut self, prefixes: &[ScanPrefix], stop: &AtomicBool) -> Result<bool, String> {
        let mut frames = ReplayFrames::new(self.cloud.session_id(), prefixes, &self.position)?;
        loop {
            if stop.load(Ordering::Acquire) { return Ok(false); }
            let Some(frame) = frames.next() else { break };
            let frame = frame?;
            self.stamp = frame.odom_pose.timestamp_ns;
            self.sequence += 1;
            let correction = self.update.correction_at(self.stamp);
            let body = correction * pose_isometry(&frame.odom_pose);
            let points: Vec<_> = frame.points_body.iter().map(|p| body * Point3::from(*p)).collect();
            let origin = body * Point3::from(frame.ray_origin_body);
            let (dynamic, _, _) = self.cloud.accumulate(self.sequence, &frame, &points,
                [origin.x, origin.y, origin.z], correction)?;
            let input = MappingBackendInput {
                source_timestamp_ns: self.stamp, received_at_ns: self.stamp,
                registered_frame_id: self.config.map_frame_id.clone(),
                sensor_origin_map: [origin.x, origin.y, origin.z], dynamic_frame: Some(dynamic),
                registered_points_map: points,
            };
            if let Some(candidate) = mapping_backend::compute_candidate_in_place_with_interval(
                &mut self.kernel, &input, self.config.compute, 0)? {
                PersistentOccupancyGrid::merge_or_create(&mut self.grid, &candidate.latest_frame_occupancy)?;
            }
            #[cfg(test)]
            if let Some(probe) = &self.probe {
                probe.frames.fetch_add(1, Ordering::Release);
                let mut paused = probe.paused.lock();
                probe.changed.notify_all();
                while *paused && !stop.load(Ordering::Acquire) { probe.changed.wait(&mut paused); }
            }
        }
        self.position = frames.position()?;
        Ok(true)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn stopping_discards_a_completed_correction_and_rejects_later_input() {
        let (corrector, confirmed) = Corrector::scripted_for_test(vec![Ok(Some(
            PgoUpdate::constant_for_test(1_000_000_000, Isometry3::translation(0.2, 0., 0.))
        ))]);
        let mut worker = CorrectionWorker::start(957, MapperConfig::default(), corrector).unwrap();
        let frame = tests_frame(1_000_000_000);
        worker.submit(CorrectionInput::new(&frame, &[], None, Isometry3::identity()));
        let started = Instant::now();
        while confirmed.lock().unwrap().is_none() {
            assert!(started.elapsed() < Duration::from_secs(2));
            std::thread::yield_now();
        }
        worker.request_stop();
        worker.submit(CorrectionInput::new(&frame, &[], None, Isometry3::identity()));
        worker.shutdown();
        assert!(worker.next().is_none(), "停止后已经算出的校正也不能再采用");
        assert!(worker.inbox.input.lock().is_none());
    }
    #[test]
    fn replay_shutdown_finishes_within_one_frame_and_pending_input_keeps_only_latest() {
        let frame = tests_frame(1_000_000_000);
        let mut cloud = CloudMap::new(frame.session_id, None, None).unwrap();
        for i in 1..=10 {
            let frame = tests_frame(i * 1_000_000_000);
            let points: Vec<_> = frame.points_body.iter().map(|p| Point3::from(*p)).collect();
            cloud.accumulate(i, &frame, &points, frame.ray_origin_body, Isometry3::identity()).unwrap();
        }
        let (corrector, _) = Corrector::scripted_for_test(vec![Ok(Some(PgoUpdate::constant_for_test(
            1_000_000_000, Isometry3::translation(0.2, 0., 0.))))]);
        let mut worker = CorrectionWorker::start(frame.session_id, MapperConfig::default(), corrector).unwrap();
        *worker.inbox.probe.paused.lock() = true;
        worker.submit(CorrectionInput::new(&frame, &[], None, Isometry3::identity()));
        let Update::Correction(_, reply) = worker.updates.recv_timeout(Duration::from_secs(2)).unwrap() else { panic!() };
        reply.send(Reply::Replay(cloud.replay_prefix().unwrap())).ok().unwrap();
        let probe = worker.inbox.probe.clone();
        {
            let mut paused = probe.paused.lock();
            while probe.frames.load(Ordering::Acquire) == 0 { probe.changed.wait(&mut paused); }
        }
        for stamp in [2_000_000_000, 3_000_000_000] {
            worker.submit(CorrectionInput::new(&tests_frame(stamp), &[], None, Isometry3::identity()));
        }
        assert_eq!(worker.inbox.input.lock().as_ref().unwrap().frame.odom_pose.timestamp_ns, 3_000_000_000);
        worker.shutdown();
        assert_eq!(probe.frames.load(Ordering::Acquire), 1, "停止后不能再重放下一帧");
        assert!(worker.worker.is_none());
    }
    fn tests_frame(stamp: u64) -> MappingFrame { super::super::tests::benchmark_frame(stamp) }
}
