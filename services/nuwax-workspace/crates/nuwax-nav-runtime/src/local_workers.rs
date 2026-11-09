//! 地图与局部规划各自独占原生可变状态；控制只读取快照与原样条。
use crate::{RegisteredFrame, SurfaceSnapshot};
use nuwax_nav_scan_native::{
    CollisionSnapshot, Config, MapOwner, MotionState, ScanPlanner, Trajectory,
};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;

/// 静态数值夹具只通过函数参数初始化地图，不在生产状态中存测试标记。
pub type MapInitializer = fn(
    &mut MapOwner,
    &[[f64; 3]],
    &[[f64; 3]],
    &[[f64; 2]],
    &[[i32; 2]],
    [f64; 3],
) -> Result<(), String>;
#[derive(Clone)]
pub(crate) struct MapInput {
    pub epoch: u64,
    pub source: Arc<SurfaceSnapshot>,
    pub config: Config,
    pub boundary: Arc<Vec<[f64; 2]>>,
    pub pose: [f64; 3],
    pub yaw: f64,
    pub frame: Option<Arc<RegisteredFrame>>,
    pub active: Option<Trajectory>,
    pub phase: f64,
    pub active_snapshot: Option<CollisionSnapshot>,
}
#[derive(Clone)]
pub(crate) struct PreparedMap {
    pub epoch: u64,
    pub source: Arc<SurfaceSnapshot>,
    pub snapshot: CollisionSnapshot,
    pub config: Config,
    pub changed: bool,
    pub checked_curve: Option<Trajectory>,
}
struct Mailbox<I, O> {
    epoch: u64,
    input: Option<I>,
    output: Option<O>,
    reset: bool,
    closed: bool,
}
impl<I, O> Default for Mailbox<I, O> {
    fn default() -> Self {
        Self {
            epoch: 0,
            input: None,
            output: None,
            reset: false,
            closed: false,
        }
    }
}
struct Worker<I, O> {
    shared: Arc<(Mutex<Mailbox<I, O>>, Condvar)>,
    thread: Option<JoinHandle<()>>,
}
impl<I: Send + 'static, O: Send + 'static> Worker<I, O> {
    fn new(
        name: &str,
        mut compute: impl FnMut(I, bool) -> O + Send + 'static,
    ) -> Result<Self, String> {
        let shared = Arc::new((Mutex::new(Mailbox::<I, O>::default()), Condvar::new()));
        let worker = shared.clone();
        let thread = std::thread::Builder::new()
            .name(name.into())
            .spawn(move || loop {
                let (epoch, input, reset) = {
                    let mut slot = worker.0.lock().unwrap();
                    while slot.input.is_none() && !slot.closed {
                        slot = worker.1.wait(slot).unwrap();
                    }
                    if slot.closed {
                        break;
                    }
                    let reset = std::mem::take(&mut slot.reset);
                    (slot.epoch, slot.input.take().unwrap(), reset)
                };
                let result = compute(input, reset);
                let mut slot = worker.0.lock().unwrap();
                if slot.epoch == epoch && !slot.closed {
                    slot.output = Some(result);
                }
            })
            .map_err(|e| e.to_string())?;
        Ok(Self {
            shared,
            thread: Some(thread),
        })
    }
    fn submit(&self, epoch: u64, input: I) {
        let mut slot = self.shared.0.lock().unwrap();
        slot.epoch = epoch;
        slot.input = Some(input);
        self.shared.1.notify_one();
    }
    fn retire(&self, epoch: u64, reset: bool) {
        let mut slot = self.shared.0.lock().unwrap();
        slot.epoch = epoch;
        slot.input = None;
        slot.output = None;
        slot.reset |= reset;
    }
    fn take(&self) -> Option<O> {
        self.shared.0.lock().unwrap().output.take()
    }
}
impl<I, O> Drop for Worker<I, O> {
    fn drop(&mut self) {
        let mut slot = self.shared.0.lock().unwrap();
        slot.closed = true;
        drop(slot);
        self.shared.1.notify_one();
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

pub(crate) struct MapWorker(Worker<MapInput, Result<PreparedMap, String>>);
impl MapWorker {
    pub fn new(initializer: Option<MapInitializer>) -> Result<Self, String> {
        let mut owner: Option<MapOwner> = None;
        let mut previous: Option<MapInput> = None;
        let mut prepared: Option<PreparedMap> = None;
        Ok(Self(Worker::new(
            "nav-map-maintenance",
            move |input: MapInput, reset| {
                let same_geometry = previous.as_ref().is_some_and(|old| {
                    crate::native_map::same_source_geometry(&old.source, &input.source)
                        && old.source.same_body_query_view(&input.source)
                        && old.config.planning_horizon == input.config.planning_horizon
                        && crate::native_map::collision_ground_padding(&old.config)
                            == crate::native_map::collision_ground_padding(&input.config)
                        && old.boundary == input.boundary
                        && map_geometry(&old.config) == map_geometry(&input.config)
                });
                let same_frame = previous.as_ref().is_some_and(|old| {
                    old.frame.as_ref().map(|f| &f.identity)
                        == input.frame.as_ref().map(|f| &f.identity)
                });
                let same_pose = previous
                    .as_ref()
                    .is_some_and(|old| old.pose == input.pose && old.yaw == input.yaw);
                let same_domain = previous.as_ref().is_none_or(|old| {
                    old.frame.as_ref().map(|f| &f.identity.domain)
                        == input.frame.as_ref().map(|f| &f.identity.domain)
                });
                if reset
                    || !same_domain
                    || previous
                        .as_ref()
                        .is_some_and(|old| map_geometry(&old.config) != map_geometry(&input.config))
                {
                    owner = None;
                    prepared = None;
                }
                let rebuilt = owner.is_none();
                if rebuilt {
                    owner = Some(MapOwner::new(&input.config)?);
                }
                let changed = !same_geometry || !same_frame || !same_pose || prepared.is_none();
                if changed {
                    let c = input.config;
                    let radius = c.planning_horizon
                        + (c.body_length * 0.5 + c.body_center_x.abs())
                            .hypot(c.body_width * 0.5 + c.body_center_y.abs());
                    let native = crate::native_map::from_surface_window(
                        &input.source,
                        Some(([input.pose[0], input.pose[1]], radius + c.resolution)),
                    )?;
                    let (recent_ground, recent_obstacles) =
                        input.source.navigation_recent_samples();
                    let obstacles: Vec<_> = native
                        .obstacles
                        .into_iter()
                        .chain(recent_obstacles)
                        .collect();
                    let ground = crate::native_map::collision_ground(
                        native.static_collision_ground,
                        recent_ground,
                        &c,
                        input.pose,
                        input.yaw,
                    );
                    let map = owner.as_mut().unwrap();
                    if let Some(initialize) = initializer {
                        initialize(
                            map,
                            &obstacles,
                            &ground,
                            &input.boundary,
                            &native.forbidden_columns,
                            input.pose,
                        )?;
                    } else {
                        map.update_surface(
                            &obstacles,
                            &ground,
                            &input.boundary,
                            &native.forbidden_columns,
                        )?;
                    }
                    if rebuilt || !same_frame {
                        if let Some(frame) = &input.frame {
                            map.ingest(
                                &frame.points_world,
                                &frame.ground_point_indices,
                                frame.sensor_origin_world,
                                frame.body_position_world,
                            )?;
                        }
                    }
                    if input.frame.is_none() && initializer.is_none() {
                        previous = Some(input);
                        return Err("等待同帧注册点云".into());
                    }
                    let snapshot = map.snapshot();
                    prepared = Some(PreparedMap {
                        epoch: input.epoch,
                        source: input.source.clone(),
                        snapshot,
                        config: input.config,
                        changed: false,
                        checked_curve: None,
                    });
                }
                let mut output = prepared.as_ref().unwrap().clone();
                output.epoch = input.epoch;
                output.source = input.source.clone();
                output.config = input.config;
                output.changed = input
                    .active_snapshot
                    .as_ref()
                    .zip(input.active.as_ref())
                    .is_some_and(|(old, curve)| {
                        !output.snapshot.same(old)
                            && output.snapshot.affects(old, curve, input.phase)
                    });
                output.checked_curve = input.active.clone();
                previous = Some(input);
                Ok(output)
            },
        )?))
    }
    pub fn submit(&self, input: MapInput) {
        self.0.submit(input.epoch, input)
    }
    pub fn retire(&self, epoch: u64, reset: bool) {
        self.0.retire(epoch, reset)
    }
    pub fn take(&self) -> Option<Result<PreparedMap, String>> {
        self.0.take()
    }
}
fn map_geometry(c: &Config) -> [u64; 12] {
    [
        c.resolution,
        c.body_length,
        c.body_width,
        c.body_height,
        c.body_to_ground,
        c.body_center_x,
        c.body_center_y,
        c.body_z_min,
        c.body_z_max,
        c.max_step_up,
        c.max_step_down,
        c.max_slope,
    ]
    .map(f64::to_bits)
}
#[derive(Clone)]
pub(crate) struct PlanInput {
    pub epoch: u64,
    pub map: PreparedMap,
    pub config: Config,
    pub reference: Arc<Vec<[f64; 3]>>,
    pub state: MotionState,
    pub active: Option<Trajectory>,
    pub phase: f64,
}
pub(crate) struct PlanWorker(Worker<PlanInput, (u64, Result<Trajectory, String>)>);
impl PlanWorker {
    pub fn new() -> Result<Self, String> {
        let mut planner = ScanPlanner::default();
        let mut epoch = None;
        Self::with_computer(move |job| {
            if epoch != Some(job.epoch) {
                planner = ScanPlanner::default();
                epoch = Some(job.epoch);
            }
            planner.plan(
                &job.config,
                &job.map.snapshot,
                &job.reference,
                &job.state,
                job.active.as_ref(),
                job.phase,
            )
        })
    }
    pub fn with_computer(
        mut compute: impl FnMut(&PlanInput) -> Result<Trajectory, String> + Send + 'static,
    ) -> Result<Self, String> {
        Ok(Self(Worker::new(
            "nav-local-planning",
            move |job: PlanInput, _| {
                let result = compute(&job);
                (job.epoch, result)
            },
        )?))
    }
    pub fn submit(&self, input: PlanInput) {
        self.0.submit(input.epoch, input)
    }
    pub fn retire(&self, epoch: u64) {
        self.0.retire(epoch, false)
    }
    pub fn take(&self) -> Option<(u64, Result<Trajectory, String>)> {
        self.0.take()
    }
    #[cfg(test)]
    pub fn ready(&self) -> bool {
        self.0.shared.0.lock().unwrap().output.is_some()
    }
}

#[cfg(test)]
#[path = "local_ground_tests.rs"]
mod ground_tests;

#[cfg(test)]
mod tests {
    use super::*;
    fn publication(worker: &MapWorker, input: MapInput) -> PreparedMap {
        worker.submit(input);
        let until = std::time::Instant::now() + std::time::Duration::from_secs(2);
        loop {
            if let Some(result) = worker.take() {
                return result.unwrap();
            }
            assert!(std::time::Instant::now() < until);
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }
    fn free(snapshot: &CollisionSnapshot, config: &Config) -> bool {
        let mut tracker = nuwax_nav_scan_native::ScanTracker::default();
        tracker
            .track_and_check(
                config,
                snapshot,
                &MotionState {
                    position: [1.55, 1.05, 0.5],
                    measured_velocity: 1,
                    measured_yaw_rate: 1,
                    ..Default::default()
                },
                [0.; 3],
                Some(0.),
            )
            .is_ok()
    }
    #[test]
    fn one_fusion_per_frame_and_immutable_snapshots_survive_rebuild() {
        let (config, source) = crate::native_test_helpers::fixture();
        let worker = MapWorker::new(None).unwrap();
        let mut frame = (*crate::registered_frame::test_frame(1)).clone();
        frame.points_world = Arc::from([[1.55, 1.05, 0.5]]);
        frame.sensor_origin_world = [0.55, 1.05, 0.5];
        frame.body_position_world = frame.sensor_origin_world;
        let mut input = MapInput {
            epoch: 1,
            source,
            config: config.scan,
            boundary: Arc::new(Vec::new()),
            pose: [0.55, 1.05, 0.5],
            yaw: 0.,
            frame: Some(Arc::new(frame.clone())),
            active: None,
            active_snapshot: None,
            phase: 0.,
        };
        let first = publication(&worker, input.clone());
        assert!(free(&first.snapshot, &input.config));
        for _ in 0..3 {
            let repeated = publication(&worker, input.clone());
            assert!(first.snapshot.same(&repeated.snapshot));
            assert!(free(&repeated.snapshot, &input.config));
        }
        // 换目标/路线只退休业务结果，不清除同源概率图，也不把同一帧再命中一次。
        input.epoch += 1;
        worker.retire(input.epoch, false);
        let retired = publication(&worker, input.clone());
        assert!(first.snapshot.same(&retired.snapshot));
        for sequence in 2..=3 {
            frame.identity.sequence = sequence;
            frame.identity.timestamp_ns += 1;
            input.frame = Some(Arc::new(frame.clone()));
            publication(&worker, input.clone());
        }
        let occupied = publication(&worker, input.clone());
        assert!(
            !free(&occupied.snapshot, &input.config),
            "三次真实帧应形成概率占据"
        );
        assert!(
            free(&first.snapshot, &input.config),
            "先前不可变快照不能被后续融合修改"
        );
        input.config.body_width += 0.01;
        let rebuilt = publication(&worker, input.clone());
        assert!(
            free(&rebuilt.snapshot, &input.config),
            "新地图只注入当前帧一次，不继承旧概率或遗漏当前帧"
        );
    }
}
