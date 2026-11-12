//! 探索对象及曲面计算独立运行，会话只接收结果并决定是否派发。
use super::*;
use crate::explorer::{apply_forbidden, SurfaceInputBuilder, ExplorerTick};
use std::sync::mpsc as channel;

enum Work {
    Feedback(NavigationStatus),
    Stop,
    Fail(String),
    Name(String),
    Progress(Reply<ExplorationProgress>),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn live_surface_uses_source_age_and_keeps_expired_surface_waiting() {
        const CHILD: &str = "NUWAX_TEST_SURFACE_SOURCE_AGE_CHILD";
        if std::env::var_os(CHILD).is_none() {
            let name = format!("{}::live_surface_uses_source_age_and_keeps_expired_surface_waiting",
                module_path!().split_once("::").unwrap().1);
            let result = std::process::Command::new(std::env::current_exe().unwrap())
                .args(["--exact", &name, "--test-threads=1", "--nocapture"])
                .env(CHILD, "1").output().unwrap();
            assert!(result.status.success(), "{}\n{}", String::from_utf8_lossy(&result.stdout), String::from_utf8_lossy(&result.stderr));
            return;
        }
        let runtime = tokio::runtime::Runtime::new().unwrap();
        runtime.block_on(async {
            let file = std::env::temp_dir().join(format!("e4b-surface-source-{}", std::process::id()));
            std::env::set_var("NUWAX_SOURCE_TIME_FILE", &file);
            let stamp = now_ns() - 3_600_000_000_000;
            let write_clock = |source| {
                let mut time = libc::timespec { tv_sec: 0, tv_nsec: 0 };
                assert_eq!(unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut time) }, 0);
                let monotonic = time.tv_sec as u64 * 1_000_000_000 + time.tv_nsec as u64;
                let mut bytes = b"NEHTIME1".to_vec();
                for value in [1_u64, 1, source, now_ns(), monotonic] { bytes.extend_from_slice(&value.to_le_bytes()); }
                std::fs::write(&file, bytes).unwrap();
            };
            write_clock(stamp + 100_000_000);
            let mapper = Mapper::start(983, crate::mapper::MapperConfig::default()).unwrap();
            mapper.feed(crate::mapper::tests::room_frame(983, stamp));
            mapper.snapshot().await.unwrap();
            let n = nuwax_config::test_support::section("navigation");
            let r = nuwax_config::test_support::section("robot");
            let d = nuwax_config::sections::Derived { min_executable_linear_mps:0.0,min_executable_angular_rps:0.0,
                nav_profile: "pct-scan-3d".into(), actuator_max_linear_mps: 0.0,
                actuator_max_yaw_rps: 0.0, actuator_motion_model: String::new(),
                robot_geometry_json: r#"{"footprint_xy_m":[[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],"z_min_m":-0.2,"z_max_m":0.2,"source":"曲面来源时钟回归"}"#.into(),
            };
            let explorer = Explorer::from_config("测试".into(), "map".into(), true, &n, &r, &d).unwrap();
            let mut worker = Worker { explorer, surface_builder: None, surface_cache: None, masked_surface_cache: None };
            let (_, pose) = watch::channel(None);
            worker.tick(&mapper, &pose).await.unwrap();
            assert!(worker.surface_cache.is_some(), "来源年龄100毫秒的曲面不能因墙钟差一小时丢弃");
            write_clock(stamp + 3_000_000_000);
            assert!(crate::explorer::fresh(stamp, 2_000_000_000).unwrap_err().contains("超过2000毫秒"));
            mapper.shutdown().await.unwrap();
            std::fs::remove_file(file).unwrap();
        });
    }
}

pub(super) struct ExplorerWorker {
    sender: channel::Sender<Work>,
    updates: mpsc::UnboundedReceiver<Result<ExplorerTick, String>>,
}
impl ExplorerWorker {
    pub(super) fn start(
        explorer: Explorer,
        mapper: Arc<Mapper>,
        pose: watch::Receiver<Option<Pose>>,
    ) -> Self {
        let (sender, receiver) = channel::channel();
        let (updates_tx, updates) = mpsc::unbounded_channel();
        let runtime = tokio::runtime::Handle::current();
        tokio::task::spawn_blocking(move || {
            let mut worker = Worker {
                explorer,
                surface_builder: None,
                surface_cache: None,
                masked_surface_cache: None,
            };
            let mut active = true;
            let mut next_tick = std::time::Instant::now();
            loop {
                let work = if active {
                    match receiver.recv_timeout(
                        next_tick.saturating_duration_since(std::time::Instant::now()),
                    ) {
                        Ok(work) => Some(work),
                        Err(channel::RecvTimeoutError::Timeout) => None,
                        Err(channel::RecvTimeoutError::Disconnected) => break,
                    }
                } else {
                    match receiver.recv() {
                        Ok(work) => Some(work),
                        Err(_) => break,
                    }
                };
                match work {
                    Some(Work::Feedback(status)) => {
                        if let Err(reason) = worker.explorer.navigation_feedback(&status) {
                            let _ = updates_tx.send(worker.explorer.failed(&reason));
                            active = false;
                        }
                    }
                    Some(Work::Stop) => {
                        let _ = worker.explorer.stop();
                        active = false;
                    }
                    Some(Work::Fail(reason)) => {
                        let _ = updates_tx.send(worker.explorer.failed(&reason));
                        active = false;
                    }
                    Some(Work::Name(name)) => worker.explorer.set_map_name(name),
                    Some(Work::Progress(reply)) => {
                        let _ = reply.send(worker.explorer.progress());
                    }
                    None => {
                        let result = runtime.block_on(worker.tick(&mapper, &pose));
                        let result = match result {
                            Ok(update) => Ok(update),
                            Err(reason) => worker.explorer.failed(&reason),
                        };
                        active = result.as_ref().is_ok_and(|update| {
                            update.status.phase == ExplorationPhase::Running as i32
                        });
                        let _ = updates_tx.send(result);
                        next_tick = std::time::Instant::now() + Duration::from_millis(200);
                    }
                }
            }
        });
        Self { sender, updates }
    }
    pub(super) async fn next(&mut self) -> Option<Result<ExplorerTick, String>> {
        self.updates.recv().await
    }

    pub(super) fn stop(&self) {
        let _ = self.sender.send(Work::Stop);
    }
    pub(super) fn fail(&self, reason: String) {
        let _ = self.sender.send(Work::Fail(reason));
    }
    pub(super) fn feedback(&self, status: NavigationStatus) {
        let _ = self.sender.send(Work::Feedback(status));
    }
    pub(super) fn set_map_name(&self, name: String) {
        let _ = self.sender.send(Work::Name(name));
    }
    pub(super) fn progress(&self, reply: Reply<ExplorationProgress>) {
        if let Err(channel::SendError(Work::Progress(reply))) =
            self.sender.send(Work::Progress(reply))
        {
            let _ = reply.send(Err(Status::unavailable("探索任务已停止")));
        }
    }
}
struct Worker {
    explorer: Explorer,
    surface_builder: Option<SurfaceInputBuilder>,
    surface_cache: Option<(
        Arc<nuwax_map_chunks::SealedMap>,
        Arc<nuwax_nav_mls_planner::SurfaceSnapshot>,
    )>,
    masked_surface_cache: Option<(
        Option<OccupancyGrid>,
        Arc<nuwax_nav_mls_planner::SurfaceSnapshot>,
    )>,
}
impl Worker {
    async fn tick(
        &mut self,
        mapper: &Mapper,
        pose: &watch::Receiver<Option<Pose>>,
    ) -> Result<ExplorerTick, String> {
        if let Err(reason) = mapper.forbidden_status() {
            return self.explorer.tick(None, None, Err(reason.clone()), Err(reason));
        }
        let sealed = mapper.surface_map().await?;
        let surface = match sealed {
            Some(sealed) => match crate::explorer::fresh(sealed.manifest().source_timestamp_ns, 2_000_000_000) {
                Err(reason) => Err(format!("等待曲面快照：{reason}")),
                Ok(()) => {
                    if self.surface_cache.as_ref().is_none_or(|(current, _)| !Arc::ptr_eq(current, &sealed)) {
                        if self.surface_builder.is_none() {
                            self.surface_builder = Some(SurfaceInputBuilder::new()?);
                        }
                        let built = self.surface_builder.as_mut().unwrap().build(sealed.clone())?;
                        self.surface_cache = Some((sealed, built));
                        self.masked_surface_cache = None;
                    }
                    let mask = mapper.forbidden_mask();
                    if self.masked_surface_cache.as_ref().is_none_or(|(current, _)| *current != mask) {
                        let physical = self.surface_cache.as_ref().unwrap().1.clone();
                        self.masked_surface_cache = Some((mask.clone(), apply_forbidden(physical, mask)?));
                    }
                    Ok(self.masked_surface_cache.as_ref().unwrap().1.clone())
                }
            },
            None => Err("等待曲面快照：尚未生成，来源年龄未知".into()),
        };
        self.explorer.tick(pose.borrow().clone(), mapper.occupancy(true), surface, Ok(()))
    }
}

#[cfg(test)]
#[tokio::test]
async fn forbidden_read_failure_stays_running_and_recovers_after_a_valid_array() {
    let root = std::env::temp_dir().join(format!("e7b-worker-forbidden-{}", uuid::Uuid::new_v4()));
    std::fs::create_dir_all(&root).unwrap();
    let path = root.join("forbidden_zones.json");
    std::fs::write(&path, b"invalid").unwrap();
    let mapper = Mapper::start(922, MapperConfig::default()).unwrap();
    let reason = mapper.set_forbidden_zones(path.clone()).await.unwrap_err();
    let (entered, _) = oneshot::channel();
    let (_release, released) = channel::channel();
    let explorer = crate::explorer::tests::blocking_fixture(entered, released);
    let mut worker = Worker { explorer, surface_builder: None, surface_cache: None, masked_surface_cache: None };
    let (_, pose) = watch::channel(None);
    for _ in 0..2 {
        let waiting = worker.tick(&mapper, &pose).await.unwrap();
        assert_eq!(waiting.status.phase, ExplorationPhase::Running as i32);
        assert!(waiting.status.message.contains(&reason));
        assert!(waiting.goal.is_none() && waiting.cancel_goal.is_none());
    }
    std::fs::write(&path, b"[]").unwrap();
    tokio::time::timeout(Duration::from_secs(2), async {
        while mapper.forbidden_status().is_err() {
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    }).await.unwrap();
    let recovered = worker.tick(&mapper, &pose).await.unwrap();
    assert_eq!(recovered.status.phase, ExplorationPhase::Running as i32);
    assert!(recovered.status.message.contains("等待地图位姿"));
    mapper.shutdown().await.unwrap();
    std::fs::remove_dir_all(root).unwrap();
}
