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
        let mut static_view: Option<([f64; 2], crate::native_map::NativeMap)> = None;
        Ok(Self(Worker::new(
            "nav-map-maintenance",
            move |input: MapInput, reset| {
                let same_static_geometry = previous.as_ref().is_some_and(|old| {
                    crate::native_map::same_source_geometry(&old.source, &input.source)
                        && old.config.planning_horizon == input.config.planning_horizon
                        && map_geometry(&old.config) == map_geometry(&input.config)
                });
                let same_geometry = same_static_geometry && previous.as_ref().is_some_and(|old| {
                    old.source.same_body_query_view(&input.source)
                        && crate::native_map::collision_ground_padding(&old.config)
                            == crate::native_map::collision_ground_padding(&input.config)
                        && old.boundary == input.boundary
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
                    static_view = None;
                }
                let rebuilt = owner.is_none();
                if rebuilt {
                    owner = Some(MapOwner::new(&input.config)?);
                }
                let changed = !same_geometry || !same_frame || !same_pose || prepared.is_none();
                if changed {
                    let c = input.config;
                    let native = static_surface_view(&mut static_view, &input, same_static_geometry)?;
                    let (recent_ground, recent_obstacles) =
                        input.source.navigation_recent_samples();
                    let obstacles: Vec<_> = native
                        .obstacles
                        .iter().copied()
                        .chain(recent_obstacles)
                        .collect();
                    let ground = crate::native_map::collision_ground(
                        native.static_collision_ground.clone(),
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
/// 静态转换仅随来源几何或窗口锚点变化；机身相关处理仍留在原逐拍分支。
fn static_surface_view<'a>(
    cached: &'a mut Option<([f64; 2], crate::native_map::NativeMap)>,
    input: &MapInput,
    same_geometry: bool,
) -> Result<&'a crate::native_map::NativeMap, String> {
    let c = input.config;
    let d = c.planning_horizon * 0.25;
    let moved = cached.as_ref().is_none_or(|(anchor, _)|
        (input.pose[0] - anchor[0]).hypot(input.pose[1] - anchor[1]) > d);
    if !same_geometry || moved {
        let anchor = if moved { [input.pose[0], input.pose[1]] } else { cached.as_ref().unwrap().0 };
        let radius = c.planning_horizon
            + (c.body_length * 0.5 + c.body_center_x.abs())
                .hypot(c.body_width * 0.5 + c.body_center_y.abs())
            + c.resolution + d;
        *cached = Some((anchor, crate::native_map::from_surface_window(&input.source, Some((anchor, radius)))?));
    }
    Ok(&cached.as_ref().unwrap().1)
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
    pub goal_yaw: Option<f64>,
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
                job.goal_yaw,
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
    fn free(snapshot: &CollisionSnapshot, _config: &Config) -> bool {
        snapshot.pose_clearance([1.55, 1.05, 0.5], 0.).occupied == 0
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

#[cfg(test)]
mod static_window_tests {
    use super::*;
    use std::collections::{BTreeMap, BTreeSet};
    use nuwax_map_chunks::{ColumnGroundRecords, HeightfieldSurface, Repository, Writer};

    #[test]
    fn anchored_window_keeps_current_window_ground_obstacles_layers_bands_and_forbidden() {
        let q = 0.1_f32;
        let repository = Repository::temporary().unwrap();
        let mut writer = Writer::new(repository.clone(), q, 8 * 1024 * 1024).unwrap();
        let mut keys = BTreeSet::new();
        for x in -25..25 { for y in -20..20 {
            let z = if x >= 0 { 2 } else { 0 };
            keys.insert([x, y, z]);
            if y == 5 || (x == -8 && (-7..3).contains(&y)) {
                keys.extend((z+1..=z+10).map(|z| [x, y, z]));
            }
        }}
        writer.observe(keys.iter().copied(), 1).unwrap();
        for x in -25..25 { for y in -20..20 {
            let z = if x >= 0 { 2 } else { 0 };
            let column = nuwax_map_chunks::heightfield_observations::HeightfieldColumn::from_counts(
                [x,y], q, BTreeMap::from([(z,[1,0,0,0,0,0,0,0])]),
            ).unwrap().unwrap();
            writer.replace_ground_column([x,y], &ColumnGroundRecords { classified_empty: false,
                heightfields: vec![HeightfieldSurface {column,samples:vec![]}], }).unwrap();
        }}
        let root = writer.snapshot(nuwax_map_chunks::Domain {session:"anchor-window".into()},1,1).unwrap();
        let mut receiver = nuwax_map_chunks::Receiver::new(repository);
        let (observations, _) = nuwax_nav_mls_planner::surface_chunks::observations_with_changes(
            &mut receiver, &root, None,
        ).unwrap();
        let (configured, base) = crate::native_test_helpers::fixture();
        let mut geometry = base.config().clone();geometry.voxel_size=q;
        let points: Vec<_> = keys.iter().map(|key| {
            let p=key.map(|v|((f64::from(v)+0.5)*f64::from(q))as f32);(p[0],p[1],p[2])
        }).collect();
        let mut mask=vec![false;400];mask[8*20+9]=true;mask[10*20+12]=true;
        let source = Arc::new(SurfaceSnapshot::from_points(&points,geometry,q).unwrap()
            .with_observations(Some(observations))
            .with_virtual_obstacles([-2.,-2.],0.2,20,20,&mask).unwrap());
        let mut config=configured.scan;config.resolution=f64::from(q);config.planning_horizon=0.6;
        let mut input=MapInput {epoch:1,source,config,boundary:Arc::new(vec![]),pose:[-1.,0.,0.5],yaw:0.,
            frame:None,active:None,phase:0.,active_snapshot:None};
        let full=crate::native_map::from_surface(&input.source).unwrap();
        assert!(!full.ground.is_empty() && !full.obstacles.is_empty() && !full.cells.is_empty()
            && !full.bands.is_empty() && !full.forbidden_columns.is_empty());
        let mut cached=None;
        for i in 0..=50 {
            input.pose=[-1.+f64::from(i)*0.04, f64::from(i%9)*0.025, 0.5];
            let radius=config.planning_horizon+(config.body_length*0.5+config.body_center_x.abs())
                .hypot(config.body_width*0.5+config.body_center_y.abs())+config.resolution;
            // 原算法每拍以当前位姿转换；参照取原窗口，缓存取外扩锚点窗口。
            let old=crate::native_map::from_surface_window(&input.source,
                Some(([input.pose[0],input.pose[1]],radius))).unwrap();
            let view=static_surface_view(&mut cached,&input,i>0).unwrap();
            let inside=|xy:[f64;2]| (xy[0]-input.pose[0]).abs()<=radius && (xy[1]-input.pose[1]).abs()<=radius;
            let points=|v:&[[f64;3]]| v.iter().filter(|p|inside([p[0],p[1]]))
                .map(|p|p.map(f64::to_bits)).collect::<BTreeSet<_>>();
            let cells=|m:&crate::native_map::NativeMap| m.cells.iter().filter(|c|inside([
                (f64::from(c.x)+0.5)*m.resolution,(f64::from(c.y)+0.5)*m.resolution]))
                .map(|c|(c.x,c.y,c.ground.to_bits(),c.ceiling.to_bits())).collect::<BTreeSet<_>>();
            let bands=|m:&crate::native_map::NativeMap| m.bands.iter().filter(|b|inside([
                (f64::from(b.x)+0.5)*m.resolution,(f64::from(b.y)+0.5)*m.resolution]))
                .map(|b|(b.x,b.y,b.low.to_bits(),b.high.to_bits())).collect::<BTreeSet<_>>();
            let forbidden=|m:&crate::native_map::NativeMap|m.forbidden_columns.iter().copied()
                .filter(|c|inside(c.map(|v|(f64::from(v)+0.5)*m.resolution))).collect::<BTreeSet<_>>();
            for reference in [&old,&full] {
                assert_eq!(points(&view.ground),points(&reference.ground));
                assert_eq!(points(&view.static_collision_ground),points(&reference.static_collision_ground));
                assert_eq!(points(&view.obstacles),points(&reference.obstacles));
                assert_eq!(cells(view),cells(reference));assert_eq!(bands(view),bands(reference));
                assert_eq!(forbidden(view),forbidden(reference));
            }
        }
    }
}
