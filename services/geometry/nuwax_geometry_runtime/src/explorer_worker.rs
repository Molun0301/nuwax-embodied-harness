//! 探索对象及曲面计算独立运行，会话只接收结果并决定是否派发。
use super::*;
use crate::explorer::{apply_forbidden, build_surface_input, ExplorerTick};
use std::sync::mpsc as channel;

enum Work {
    Feedback(NavigationStatus),
    Stop,
    Fail(String),
    Name(String),
    Progress(Reply<ExplorationProgress>),
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
        mapper.forbidden_status()?;
        let sealed = mapper.surface_map().await?;
        let surface = if let Some(sealed) = sealed.filter(|map| {
            now_ns().saturating_sub(map.manifest().source_timestamp_ns) <= 2_000_000_000
        }) {
            if self
                .surface_cache
                .as_ref()
                .is_none_or(|(current, _)| !Arc::ptr_eq(current, &sealed))
            {
                let built = build_surface_input(sealed.clone())?;
                self.surface_cache = Some((sealed, built));
                self.masked_surface_cache = None;
            }
            let mask = mapper.forbidden_mask();
            if self
                .masked_surface_cache
                .as_ref()
                .is_none_or(|(current, _)| *current != mask)
            {
                let physical = self.surface_cache.as_ref().unwrap().1.clone();
                self.masked_surface_cache = Some((mask.clone(), apply_forbidden(physical, mask)?));
            }
            Some(self.masked_surface_cache.as_ref().unwrap().1.clone())
        } else {
            None
        };
        self.explorer
            .tick(pose.borrow().clone(), mapper.occupancy(true), surface)
    }
}
