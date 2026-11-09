//! 原生地图、慢规划与快跟踪的独立接口。
use std::{
    ffi::{c_char, c_void, CStr},
    ptr::NonNull,
    sync::Arc,
};
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub resolution: f64,
    pub body_length: f64,
    pub body_width: f64,
    pub body_height: f64,
    pub body_to_ground: f64,
    pub max_velocity: f64,
    pub max_acceleration: f64,
    pub max_yaw_rate: f64,
    pub planning_horizon: f64,
    pub sample_dt: f64,
    pub body_center_x: f64,
    pub body_center_y: f64,
    pub body_z_min: f64,
    pub body_z_max: f64,
    pub max_step_up: f64,
    pub max_step_down: f64,
    pub max_slope: f64,
    pub max_yaw_acceleration: f64,
    pub max_reverse_velocity: f64,
    pub max_lateral_velocity: f64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct TrajectorySample {
    pub time: f64,
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub acceleration: [f64; 3],
    pub yaw: f64,
    pub yaw_rate: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionBoundarySource {
    Measured,
    PreviousTrajectory,
    InitialZeroAssumption,
    PreviousControlEstimate,
    AuthorDirectionReset,
    SmoothedMeasured,
}

/// 当拍实际状态；测量缺失与真实零速分开。
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct MotionState {
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub planning_velocity: [f64; 3],
    pub yaw: f64,
    pub yaw_rate: f64,
    pub route_progress: f64,
    pub timestamp_ns: i64,
    pub measured_velocity: u32,
    pub measured_yaw_rate: u32,
}
#[repr(C)]
#[derive(Default)]
struct SamplesRaw {
    duration: f64,
    reference_progress: f64,
    reference_duration: f64,
    local_target: [f64; 3],
    command_forward: f64,
    command_lateral: f64,
    command_yaw_rate: f64,
    command_duration: f64,
    samples: *mut TrajectorySample,
    sample_count: usize,
    replanned: u32,
    execution_cycle_complete: u32,
    execution_failed: u32,
    motion_boundary_sources: [u32; 3],
}
/// 原命令检查结论；提前请求与当前命令许可分开。
#[repr(C)]
#[derive(Default, Debug, Clone, Copy)]
struct TrackRaw {
    pub command: [f64; 3],
    pub phase: f64,
    pub remaining: f64,
    pub deviation: f64,
    pub reason: u32,
    pub limited: u32,
    pub installed: u32,
}
#[derive(Default, Debug)]
pub struct TrackResult {
    pub command: [f64; 3],
    pub phase: f64,
    pub remaining: f64,
    pub deviation: f64,
    pub reason: u32,
    pub limited: u32,
    pub installed: bool,
    pub recovery_samples: Vec<TrajectorySample>,
}
/// 原生跟踪失败的原因位及原始文本。
#[derive(Debug)]
pub struct TrackFailure {
    pub reason: u32,
    pub message: String,
}
impl From<TrackFailure> for String {
    fn from(failure: TrackFailure) -> Self {
        failure.message
    }
}
/// 跟踪原因位，与 scan_native.h 的 ScanTrackReason 对应。
pub mod track_reason {
    pub const NO_TRAJECTORY:u32=1<<0;
    pub const DEVIATED:u32=1<<1;
    pub const FINISHED:u32=1<<2;
    pub const NO_CANDIDATE:u32=1<<3;
    pub const REPLAN_EARLY:u32=1<<4;
    pub const RECOVERY_FAILED:u32=1<<5;
    pub const REJECTED_TRAJECTORY:u32=1<<6;
}
impl TrackResult {
    pub fn needs_plan(&self) -> bool {
        self.reason != 0
    }
    pub fn invalid(&self) -> bool {
        self.reason & !(track_reason::REPLAN_EARLY | track_reason::REJECTED_TRAJECTORY) != 0
    }
    pub fn replan_early(&self) -> bool {
        self.reason & track_reason::REPLAN_EARLY != 0
    }
    pub fn rejected_trajectory(&self) -> bool {
        self.reason & track_reason::REJECTED_TRAJECTORY != 0
    }
    pub fn no_candidate(&self) -> bool {
        self.reason & track_reason::NO_CANDIDATE != 0
    }
    pub fn deviated(&self) -> bool {
        self.reason & track_reason::DEVIATED != 0
    }
    pub fn finished(&self) -> bool {
        self.reason & track_reason::FINISHED != 0
    }
}
extern "C" {
    fn nuwax_scan_map_create(config: *const Config) -> *mut c_void;
    fn nuwax_scan_map_destroy(map: *mut c_void);
    fn nuwax_scan_map_surface(
        map: *mut c_void,
        obstacles: *const f64,
        count: usize,
        ground: *const f64,
        ground_count: usize,
        boundary: *const f64,
        boundary_count: usize,
        forbidden: *const i32,
        forbidden_count: usize,
    ) -> i32;
    fn nuwax_scan_map_ingest(
        map: *mut c_void,
        points: *const f64,
        count: usize,
        ground: *const u32,
        ground_count: usize,
        origin: *const f64,
        body: *const f64,
    ) -> i32;
    fn nuwax_scan_map_seed_occupied(
        map: *mut c_void,
        points: *const f64,
        count: usize,
        body: *const f64,
    );
    fn nuwax_scan_map_snapshot(map: *const c_void) -> *mut c_void;
    fn nuwax_scan_snapshot_affects(
        current: *const c_void,
        previous: *const c_void,
        trajectory: *const c_void,
        from: f64,
    ) -> i32;
    fn nuwax_scan_snapshot_destroy(map: *mut c_void);
    fn nuwax_scan_planner_create() -> *mut c_void;
    fn nuwax_scan_planner_destroy(planner: *mut c_void);
    fn nuwax_scan_planner_plan(
        planner: *mut c_void,
        config: *const Config,
        map: *const c_void,
        reference: *const f64,
        count: usize,
        state: *const MotionState,
        active: *const c_void,
        phase: f64,
        error: *mut c_char,
        capacity: usize,
    ) -> *mut c_void;
    fn nuwax_scan_trajectory_destroy(trajectory: *mut c_void);
    fn nuwax_scan_trajectory_samples(
        trajectory: *const c_void,
        from: f64,
        dt: f64,
        out: *mut SamplesRaw,
    );
    fn nuwax_scan_tracker_create() -> *mut c_void;
    fn nuwax_scan_tracker_destroy(tracker: *mut c_void);
    fn nuwax_scan_tracker_clear(tracker: *mut c_void);
    fn nuwax_scan_tracker_install(tracker: *mut c_void, trajectory: *const c_void);
    fn nuwax_scan_tracker_track(
        tracker: *mut c_void,
        config: *const Config,
        map: *const c_void,
        state: *const MotionState,
        previous: *const f64,
        terminal_yaw: f64,
        out: *mut TrackRaw,
        recovery: *mut SamplesRaw,
        error: *mut c_char,
        capacity: usize,
    );
    fn nuwax_scan_free(out: *mut SamplesRaw);
}
fn error_message(error: &[c_char]) -> String {
    unsafe { CStr::from_ptr(error.as_ptr()) }
        .to_string_lossy()
        .into_owned()
}
/// 唯一可变概率地图；必须由地图维护线程独占。
pub struct MapOwner(NonNull<c_void>);
// 原生句柄只通过可变借用访问，可移交到地图线程。
unsafe impl Send for MapOwner {}
impl MapOwner {
    pub fn new(config: &Config) -> Result<Self, String> {
        NonNull::new(unsafe { nuwax_scan_map_create(config) })
            .map(Self)
            .ok_or_else(|| "原生地图初始化失败".into())
    }
    pub fn update_surface(
        &mut self,
        obstacles: &[[f64; 3]],
        ground: &[[f64; 3]],
        boundary: &[[f64; 2]],
        forbidden: &[[i32; 2]],
    ) -> Result<(), String> {
        let result = unsafe {
            nuwax_scan_map_surface(
                self.0.as_ptr(),
                obstacles.as_ptr().cast(),
                obstacles.len(),
                ground.as_ptr().cast(),
                ground.len(),
                boundary.as_ptr().cast(),
                boundary.len(),
                forbidden.as_ptr().cast(),
                forbidden.len(),
            )
        };
        if result == 0 {
            Ok(())
        } else {
            Err("原生实体与业务掩码更新失败".into())
        }
    }
    pub fn ingest(
        &mut self,
        points: &[[f64; 3]],
        ground: &[u32],
        origin: [f64; 3],
        body: [f64; 3],
    ) -> Result<(), String> {
        let result = unsafe {
            nuwax_scan_map_ingest(
                self.0.as_ptr(),
                points.as_ptr().cast(),
                points.len(),
                ground.as_ptr(),
                ground.len(),
                origin.as_ptr(),
                body.as_ptr(),
            )
        };
        if result == 0 {
            Ok(())
        } else {
            Err("同帧注册点云融合失败".into())
        }
    }
    /// 静态数值夹具由调用方函数初始化，不在会话中保留测试模式。
    pub fn seed_occupied(&mut self, points: &[[f64; 3]], body: [f64; 3]) {
        unsafe {
            nuwax_scan_map_seed_occupied(
                self.0.as_ptr(),
                points.as_ptr().cast(),
                points.len(),
                body.as_ptr(),
            )
        }
    }
    pub fn snapshot(&self) -> CollisionSnapshot {
        CollisionSnapshot(Arc::new(SnapshotHandle(
            NonNull::new(unsafe { nuwax_scan_map_snapshot(self.0.as_ptr()) }).unwrap(),
        )))
    }
}
impl Drop for MapOwner {
    fn drop(&mut self) {
        unsafe { nuwax_scan_map_destroy(self.0.as_ptr()) }
    }
}
struct SnapshotHandle(NonNull<c_void>);
// 快照只含不可变查询输入，原生读者分别拥有查询对象。
unsafe impl Send for SnapshotHandle {}
unsafe impl Sync for SnapshotHandle {}
impl Drop for SnapshotHandle {
    fn drop(&mut self) {
        unsafe { nuwax_scan_snapshot_destroy(self.0.as_ptr()) }
    }
}
/// 规划和跟踪共享的不可变碰撞视图。
#[derive(Clone)]
pub struct CollisionSnapshot(Arc<SnapshotHandle>);
impl CollisionSnapshot {
    pub fn same(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }
    pub fn affects(&self, previous: &Self, trajectory: &Trajectory, from: f64) -> bool {
        unsafe {
            nuwax_scan_snapshot_affects(
                self.0 .0.as_ptr(),
                previous.0 .0.as_ptr(),
                trajectory.0 .0.as_ptr(),
                from,
            ) != 0
        }
    }
}
struct TrajectoryHandle(NonNull<c_void>);
// 原样条只读，控制器与规划状态机不写入这个载荷。
unsafe impl Send for TrajectoryHandle {}
unsafe impl Sync for TrajectoryHandle {}
impl Drop for TrajectoryHandle {
    fn drop(&mut self) {
        unsafe { nuwax_scan_trajectory_destroy(self.0.as_ptr()) }
    }
}
/// 原始结点、控制点及对应路线区段；不以计算耗时判过期。
#[derive(Clone)]
pub struct Trajectory(Arc<TrajectoryHandle>);
/// 原样条的只读显示与诊断数据，不携带速度许可。
#[derive(Debug)]
pub struct TrajectoryView {
    pub samples: Vec<TrajectorySample>,
    pub duration: f64,
    pub reference_progress: f64,
    pub reference_duration: f64,
    pub local_target: [f64; 3],
    pub motion_boundary_sources: [MotionBoundarySource; 3],
}
impl Trajectory {
    pub fn same(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }
    pub fn inspect(&self, from: f64, dt: f64) -> TrajectoryView {
        let mut raw = SamplesRaw::default();
        unsafe { nuwax_scan_trajectory_samples(self.0 .0.as_ptr(), from, dt, &mut raw) };
        let samples = if raw.sample_count == 0 {
            Vec::new()
        } else {
            unsafe { std::slice::from_raw_parts(raw.samples, raw.sample_count) }.to_vec()
        };
        let source = |x| match x {
            0 => MotionBoundarySource::Measured,
            1 => MotionBoundarySource::PreviousTrajectory,
            2 => MotionBoundarySource::InitialZeroAssumption,
            3 => MotionBoundarySource::PreviousControlEstimate,
            4 => MotionBoundarySource::AuthorDirectionReset,
            _ => MotionBoundarySource::SmoothedMeasured,
        };
        let view = TrajectoryView {
            samples,
            duration: raw.duration,
            reference_progress: raw.reference_progress,
            reference_duration: raw.reference_duration,
            local_target: raw.local_target,
            motion_boundary_sources: raw.motion_boundary_sources.map(source),
        };
        unsafe { nuwax_scan_free(&mut raw) };
        view
    }
    pub fn samples(&self, from: f64, dt: f64) -> Vec<TrajectorySample> {
        self.inspect(from, dt).samples
    }
}
/// 独立慢规划会话，不持运动端或控制器。
pub struct ScanPlanner(NonNull<c_void>);
unsafe impl Send for ScanPlanner {}
impl Default for ScanPlanner {
    fn default() -> Self {
        Self(NonNull::new(unsafe { nuwax_scan_planner_create() }).unwrap())
    }
}
impl ScanPlanner {
    pub fn plan(
        &mut self,
        config: &Config,
        map: &CollisionSnapshot,
        reference: &[[f64; 3]],
        state: &MotionState,
        active: Option<&Trajectory>,
        phase: f64,
    ) -> Result<Trajectory, String> {
        let mut error = [0 as c_char; 1024];
        let result = unsafe {
            nuwax_scan_planner_plan(
                self.0.as_ptr(),
                config,
                map.0 .0.as_ptr(),
                reference.as_ptr().cast(),
                reference.len(),
                state,
                active.map_or(std::ptr::null(), |t| t.0 .0.as_ptr()),
                phase,
                error.as_mut_ptr(),
                error.len(),
            )
        };
        NonNull::new(result)
            .map(|p| Trajectory(Arc::new(TrajectoryHandle(p))))
            .ok_or_else(|| error_message(&error))
    }
}
impl Drop for ScanPlanner {
    fn drop(&mut self) {
        unsafe { nuwax_scan_planner_destroy(self.0.as_ptr()) }
    }
}
/// 快控制会话；这里只做相位、控制与原命令检查。
pub struct ScanTracker(NonNull<c_void>);
unsafe impl Send for ScanTracker {}
impl Default for ScanTracker {
    fn default() -> Self {
        Self(NonNull::new(unsafe { nuwax_scan_tracker_create() }).unwrap())
    }
}
impl ScanTracker {
    pub fn clear(&mut self) {
        unsafe { nuwax_scan_tracker_clear(self.0.as_ptr()) }
    }
    pub fn install(&mut self, trajectory: &Trajectory) {
        unsafe { nuwax_scan_tracker_install(self.0.as_ptr(), trajectory.0 .0.as_ptr()) }
    }
    pub fn track_and_check(
        &mut self,
        config: &Config,
        map: &CollisionSnapshot,
        state: &MotionState,
        previous: [f64; 3],
        terminal_yaw: Option<f64>,
    ) -> Result<TrackResult, TrackFailure> {
        let mut result = TrackRaw::default();
        let mut recovery = SamplesRaw::default();
        let mut error = [0 as c_char; 1024];
        unsafe {
            nuwax_scan_tracker_track(
                self.0.as_ptr(),
                config,
                map.0 .0.as_ptr(),
                state,
                previous.as_ptr(),
                terminal_yaw.unwrap_or(f64::NAN),
                &mut result,
                &mut recovery,
                error.as_mut_ptr(),
                error.len(),
            )
        };
        let samples = if recovery.sample_count == 0 {
            Vec::new()
        } else {
            unsafe { std::slice::from_raw_parts(recovery.samples, recovery.sample_count) }.to_vec()
        };
        unsafe { nuwax_scan_free(&mut recovery) };
        if error[0] == 0 {
            Ok(TrackResult {
                command: result.command,
                phase: result.phase,
                remaining: result.remaining,
                deviation: result.deviation,
                reason: result.reason,
                limited: result.limited,
                installed: result.installed != 0,
                recovery_samples: samples,
            })
        } else {
            Err(TrackFailure { reason: result.reason, message: error_message(&error) })
        }
    }
}
impl Drop for ScanTracker {
    fn drop(&mut self) {
        unsafe { nuwax_scan_tracker_destroy(self.0.as_ptr()) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    mod fixture {include!("numerical_fixture.rs");}
    use fixture::{Planner,PlanRequest,PlanResult};
    include!("detour_following_tests.rs");
    include!("command_limit_tests.rs");
    include!("surface_execution_tests.rs");
    include!("clearance_tests.rs");
    include!("gait_velocity_tests.rs");
    // These component fixtures provide static occupancy, not live sensor frames.
    // One call remains one real FSM tick; no synthetic catch-up ticks are added.
    fn fixture_planner() -> Planner {
        let mut planner=Planner::new();
        planner.set_static_map_fixture(true);
        planner
    }
    fn plan(request:&PlanRequest<'_>)->Result<PlanResult,String>{
        let mut planner=fixture_planner();let mut req=*request;
        for _ in 0..20{let out=planner.step(&req)?;if !out.samples.is_empty()||req.reference.is_empty(){return Ok(out)}req.odometry_timestamp_ns+=(req.config.sample_dt*1e9) as i64;}
        Err("一秒控制周期内未生成局部曲线".into())
    }
    fn config() -> Config {
        Config {
            resolution: 0.1,
            body_length: 0.7,
            body_width: 0.3,
            body_height: 0.3,
            body_to_ground: 0.35,
            max_velocity: 0.6,
            max_acceleration: 0.8,
            max_yaw_rate: 1.0,
            planning_horizon: 2.0,
            sample_dt: 0.1,
            body_center_x: 0.0,
            body_center_y: 0.0,
            body_z_min: -0.15,
            body_z_max: 0.15,
            max_step_up: 0.2,
            max_step_down: 0.2,
            max_slope: 0.5,
            max_yaw_acceleration: 2.0,
            max_reverse_velocity: 0.6,
            max_lateral_velocity: 0.6,
        }
    }
    fn request<'a>(reference: &'a [[f64; 3]], obstacles: &'a [[f64; 3]]) -> PlanRequest<'a> {
        PlanRequest {
            reference,
            obstacles,
            ground: &[],
            start_position: [0.0, 0.0, 0.35],
            start_velocity: Some([0.0; 3]),
            start_acceleration: Some([0.0; 3]),
            start_yaw: 0.0,
            start_yaw_rate: Some(0.0),
            odometry_timestamp_ns: 0,
            config: config(),
            boundary: &[],
            forbidden_columns: &[],
            goal_yaw: None,
        }
    }
    fn start_reference(req: &mut PlanRequest<'_>) -> (Planner, PlanResult) {
        let mut planner = fixture_planner();
        let result = planner.step(req).unwrap();
        assert!(!result.samples.is_empty(), "生成轨迹后应有样条：{result:?}");
        (planner, result)
    }
    #[test]
    fn entirely_unknown_corridor_is_traversable() {
        let route = [[0.0, 0.0, 0.35], [8.0, 0.0, 0.35]];
        let mut req = request(&route, &[]);
        let (mut planner, result) = start_reference(&mut req);
        assert!(result.local_target[0] > 1.8 && result.local_target[0] < 2.3);
        assert!(result.reference_progress < result.reference_duration);
        let first = result.samples.first().unwrap();
        assert_eq!(first.time, 0.0);
        assert!(first.position[0] >= 0.0 && first.position[0] < result.local_target[0],
            "导出的是控制器当前时点之后的剩余样条：{first:?}");
        req.odometry_timestamp_ns += 100_000_000;
        assert!(planner.step(&req).unwrap().command_forward > 0.0);
    }
    #[test]
    fn measured_wall_is_bypassed_by_original_rebound_optimizer() {
        let route = [[0.0, 0.0, 0.35], [5.0, 0.0, 0.35]];
        let obstacles: Vec<_> = (-5..=5)
            .flat_map(|y| (1..=7).map(move |z| [1.0, y as f64 * 0.1, z as f64 * 0.1]))
            .collect();
        let result = plan(&request(&route, &obstacles)).unwrap();
        // 独立旋转矩形/实体墙分离轴检查，不能以原双圆柱额外绕行幅度作为通过条件。
        let mut yaw = 0.0;
        for point in &result.samples {
            if point.velocity[0].hypot(point.velocity[1]) > 1e-8 {
                yaw = point.velocity[1].atan2(point.velocity[0]);
            }
            let (cs, sn) = (yaw.cos(), yaw.sin());
            let dx = 1.05 - point.position[0];
            let dy = 0.05 - point.position[1];
            let overlaps = dx.abs() < 0.35 * cs.abs() + 0.15 * sn.abs() + 0.05
                && dy.abs() < 0.35 * sn.abs() + 0.15 * cs.abs() + 0.55
                && (dx * cs + dy * sn).abs() < 0.35 + 0.05 * cs.abs() + 0.55 * sn.abs()
                && (-dx * sn + dy * cs).abs() < 0.15 + 0.05 * sn.abs() + 0.55 * cs.abs();
            assert!(!overlaps, "样条机身与实体墙相交: {point:?}");
        }
    }
    #[test]
    fn measured_feedback_does_not_claim_measured_acceleration() {
        let route = [[0.0, 0.0, 0.35], [5.0, 0.0, 0.35]];
        let mut req = request(&route, &[]);
        req.start_velocity = Some([0.25, 0.03, 0.0]);
        req.start_acceleration = Some([0.1, 0.0, 0.0]);
        let mut planner = fixture_planner();
        planner.step(&req).unwrap();
        req.odometry_timestamp_ns += 100_000_000;
        let previous = planner.step(&req).unwrap();
        req.odometry_timestamp_ns += 100_000_000;
        let result = planner.step(&req).unwrap();
        assert!(!result.samples.is_empty());
        assert_eq!(result.motion_boundary_sources[0], MotionBoundarySource::SmoothedMeasured);
        assert_eq!(result.motion_boundary_sources[1], MotionBoundarySource::InitialZeroAssumption);
        // 剩余样条不再从初始时点导出；实际请求仍受上一拍机体系命令的动态约束。
        assert!((result.command_forward - previous.command_forward)
            .hypot(result.command_lateral - previous.command_lateral)
            <= req.config.max_acceleration * req.config.sample_dt + 1e-8);
    }
    #[test]
    fn reference_windows_continue_to_the_final_goal() {
        let route = [[0.0, 0.0, 0.35], [6.0, 0.0, 0.35]];
        let mut req = request(&route, &[]);
        let mut planner = fixture_planner();
        let first = planner.step(&req).unwrap();
        let (arrived, _) = drive(&mut planner, &mut req, [6.0, 0.0]).unwrap();
        assert!(arrived.is_some(), "完整参考应到达最终目标：{:?}", req.start_position);
        let last = planner.step(&req).unwrap();
        assert!(last.reference_progress > first.reference_progress);
        assert!((last.reference_progress - last.reference_duration).abs() < 1e-6);
    }
    #[test]
    #[ignore = "retired candidate q3 contract; user restored independent XY/yaw"]
    fn resting_spline_preserves_actual_initial_heading() {
        let route = [[0.0, 0.0, 0.35], [0.025, -0.025, 0.35], [7.0, -0.025, 0.35]];
        let mut req = request(&route, &[]);
        // 此具名边界单测采样起步极限；原完整闭环仍用原控制周期。
        req.config.sample_dt = 0.001;
        let result = plan(&req).unwrap();
        let first_moving = result.samples.iter()
            .find(|s| s.velocity[0].hypot(s.velocity[1]) > 1e-8).unwrap();
        let heading = first_moving.velocity[1].atan2(first_moving.velocity[0]);
        assert!((heading - req.start_yaw).abs() < 1e-7,
            "zero measured v/a must not erase actual heading: {heading}");
        assert!(result.samples[0].velocity.iter().all(|v| v.abs() < 1e-7));
        assert!(result.samples[0].acceleration.iter().all(|a| a.abs() < 1e-7));
    }
    #[test]
    #[ignore = "retired candidate spatial cursor; user restored original time reference"]
    fn stationary_feedback_does_not_consume_unexecuted_spline() {
        let route = [[0.0, 0.0, 0.35], [7.0, 0.0, 0.35]];
        let mut req = request(&route, &[]);
        let mut planner = fixture_planner();
        // 新时间戳不等于真实移动：执行反馈保持原地、零速，不能把未走样条丢掉。
        for tick in 0..20 {
            req.odometry_timestamp_ns = 1_000_000_000 + tick * 100_000_000;
            let result = planner.step(&req).unwrap();
            let first = result.samples.first().unwrap();
            assert!((first.position[0] - req.start_position[0]).abs() < 1e-8,
                "tick {tick}: unexecuted prefix consumed at {:?}", first.position);
        }
    }
    #[test]
    fn true_body_width_changes_obstacle_collision() {
        let route = [[0.0, 0.0, 0.35], [4.0, 0.0, 0.35]];
        let obstacles = [[0.0, 0.55, 0.35]];
        let mut req = request(&route, &obstacles);
        assert!(plan(&req).is_ok());
        req.config.body_width = 1.4;
        assert!(plan(&req).is_err());
    }
    #[test]
    fn terminal_rotation_uses_real_body_sweep() {
        let mut req = request(&[], &[]);
        req.goal_yaw = Some(1.0);
        let free = plan(&req).unwrap();
        assert_eq!(free.command_forward, 0.0);
        assert!(free.command_yaw_rate > 0.0);
        let obstacles = [[0.35, 0.1, 0.35]];
        req.obstacles = &obstacles;
        let mut tracker=fixture_planner();let mut blocked=false;
        for _ in 0..30{match tracker.step(&req){Ok(out)=>{req.start_yaw+=out.command_yaw_rate*req.config.sample_dt;req.start_yaw_rate=Some(out.command_yaw_rate);},Err(_)=>{blocked=true;break;}}req.odometry_timestamp_ns+=(req.config.sample_dt*1e9) as i64;}
        assert!(blocked,"实体阻挡最终应使当拍响应与制动检查拒绝");
    }
    #[test]
    fn ground_points_do_not_become_body_obstacles() {
        let route = [[0.0, 0.0, 0.35], [4.0, 0.0, 0.35]];
        let ground: Vec<_> = (0..40).map(|x| [x as f64 * 0.1, 0.0, 0.0]).collect();
        let mut req = request(&route, &[]);
        req.ground = &ground;
        let (mut planner, _) = start_reference(&mut req);
        req.odometry_timestamp_ns += 100_000_000;
        assert!(planner.step(&req).unwrap().command_forward > 0.0);
    }
    /// 横跨直线参考的实测墙体。
    fn wall(x: f64) -> Vec<[f64; 3]> {
        (-5..=5)
            .flat_map(|y| (1..=7).map(move |z| [x, y as f64 * 0.1, z as f64 * 0.1]))
            .collect()
    }
    /// 按用户恢复的完整机身XY+角速度积分，保留原场景/预算/到达断言。
    fn drive(
        planner: &mut Planner,
        req: &mut PlanRequest<'_>,
        goal: [f64; 2],
    ) -> Result<(Option<usize>, f64), String> {
        drive_observing(planner, req, goal, |_, _, _, _, _| {})
    }
    fn drive_observing(
        planner: &mut Planner,
        req: &mut PlanRequest<'_>,
        goal: [f64; 2],
        mut observe: impl FnMut([f64; 3], f64, [f64; 2], f64, f64),
    ) -> Result<(Option<usize>, f64), String> {
        let mut lateral: f64 = 0.0;
        let initial_stamp = req.odometry_timestamp_ns;
        let sample_ns = (req.config.sample_dt * 1e9) as i64;
        for tick in 0..600 {
            req.odometry_timestamp_ns = initial_stamp + tick as i64 * sample_ns;
            let result = planner.step(req).map_err(|error| {
                format!(
                    "tick {tick}, pose {:?}, yaw {}, speed {:?}: {error}",
                    req.start_position, req.start_yaw, req.start_velocity
                )
            })?;
            let next = [result.command_forward, result.command_lateral];
            let next_w = result.command_yaw_rate;
            let yaw = req.start_yaw;
            let world = req.start_velocity.unwrap();
            let previous = [yaw.cos()*world[0]+yaw.sin()*world[1], -yaw.sin()*world[0]+yaw.cos()*world[1]];
            let dt = result.command_duration;
            assert!((next[0]-previous[0]).hypot(next[1]-previous[1])
                <= req.config.max_acceleration*dt+1e-8);
            assert!((next_w-req.start_yaw_rate.unwrap()).abs()
                <= req.config.max_yaw_acceleration*dt+1e-8);
            let v=[(previous[0]+next[0])/2.,(previous[1]+next[1])/2.];
            let w=(req.start_yaw_rate.unwrap()+next_w)/2.;
            observe(req.start_position, yaw, v, w, dt);
            let (a,b)=if w.abs()>1e-9 {((w*dt).sin()/w,(1.-(w*dt).cos())/w)}else{(dt,0.)};
            let dx=a*v[0]-b*v[1];let dy=b*v[0]+a*v[1];
            req.start_position[0]+=yaw.cos()*dx-yaw.sin()*dy;
            req.start_position[1]+=yaw.sin()*dx+yaw.cos()*dy;
            req.start_yaw+=w*dt;
            req.start_yaw_rate=Some(next_w);
            req.start_velocity=Some([next[0]*req.start_yaw.cos()-next[1]*req.start_yaw.sin(),next[0]*req.start_yaw.sin()+next[1]*req.start_yaw.cos(),0.]);
            // Model exposes endpoint velocity; no fabricated acceleration observation.
            req.start_acceleration=None;
            lateral = lateral.max(req.start_position[1].abs());
            if (req.start_position[0] - goal[0]).hypot(req.start_position[1] - goal[1]) < 0.15 {
                return Ok((Some(tick), lateral));
            }
        }
        Ok((None, lateral))
    }
    #[test]
    fn actual_differential_commands_reach_goal_around_measured_wall() {
        let route = [[0.0, 0.0, 0.35], [4.0, 0.0, 0.35]];
        let obstacles = wall(1.5);
        let mut req = request(&route, &obstacles);
        let (arrived, _) =
            drive(&mut fixture_planner(), &mut req, [4.0, 0.0]).unwrap_or_else(|error| panic!("{error}"));
        assert!(arrived.is_some(), "final position {:?}", req.start_position);
    }
    #[test]
    fn overhead_ground_layer_is_a_body_obstacle() {
        let ceiling = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.45]];
        let mut req = request(&[], &[]);
        req.goal_yaw = Some(0.5);
        req.ground = &ceiling;
        assert!(plan(&req).is_err());
        req.start_position[2] = 1.35;
        let upper_floor = [[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        req.ground = &upper_floor;
        assert!(plan(&req).is_ok());
    }
    #[test]
    fn body_extent_and_offset_respect_boundary() {
        let boundary = [[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]];
        let mut req = request(&[], &[]);
        req.goal_yaw = Some(0.3);
        req.boundary = &boundary;
        assert!(plan(&req).is_ok());
        req.config.body_center_x = 0.3;
        let result = plan(&req);
        assert!(result.is_err(), "实体右界0.65超过业务边界0.5，实际结果：{result:?}");
    }
    #[test]
    fn forbidden_columns_remain_occupied_at_every_height() {
        let columns = [[0, 0]];
        let mut req = request(&[], &[]);
        req.goal_yaw = Some(0.3);
        req.forbidden_columns = &columns;
        assert!(plan(&req).is_err());
        req.start_position[2] = 12.0;
        assert!(plan(&req).is_err());
    }
    #[test]
    fn terminal_rotation_brakes_using_measured_angular_velocity() {
        let mut req = request(&[], &[]);
        req.goal_yaw = Some(1.0);
        for _ in 0..100 {
            let result = plan(&req).unwrap();
            assert!(
                (result.command_yaw_rate - req.start_yaw_rate.unwrap()).abs()
                    <= req.config.max_yaw_acceleration * req.config.sample_dt + 1e-8
            );
            req.start_yaw += result.command_yaw_rate * result.command_duration;
            req.start_yaw_rate = Some(result.command_yaw_rate);
            if (req.start_yaw - 1.0).abs() < 0.001 && req.start_yaw_rate.unwrap().abs() < 0.01 {
                return;
            }
        }
        panic!("yaw {}, rate {:?}", req.start_yaw, req.start_yaw_rate);
    }
    #[test]
    fn original_time_allocation_preserves_feasible_oblique_thin_door() {
        let resolution = f64::from(0.01_f32);
        let yaw = 5.0_f64.to_radians();
        let start = [
            -0.5,
            33.0 * resolution / 2.0 + (-0.5 - 2.0 * resolution) * yaw.tan(),
            0.4,
        ];
        let end = [
            0.5,
            33.0 * resolution / 2.0 + (0.5 - 2.0 * resolution) * yaw.tan(),
            0.4,
        ];
        let route = [start, end];
        let obstacles: Vec<_> = (0..4)
            .flat_map(|x| {
                (-50..100).filter(|&y| y < 0 || y >= 33).flat_map(move |y| {
                    (0..80).map(move |z| {
                        [
                            (x as f64 + 0.5) * resolution,
                            (y as f64 + 0.5) * resolution,
                            (z as f64 + 0.5) * resolution,
                        ]
                    })
                })
            })
            .collect();
        let mut req = request(&route, &obstacles);
        req.start_position = start;
        req.start_yaw = yaw;
        req.config.resolution = resolution;
        req.config.body_length = 0.6;
        req.config.body_width = 32.0 * resolution;
        req.config.body_to_ground = 0.4;
        req.config.max_velocity = 0.3;
        req.config.max_acceleration = 1.0;
        req.config.max_yaw_rate = 0.8;
        req.config.planning_horizon = 5.0;
        req.config.sample_dt = 0.05;
        let cfg = req.config;
        let radius = (cfg.body_length / 2.).hypot(cfg.body_width / 2.);
        let mut minimum_clearance = f64::INFINITY;
        let (arrived, _) = drive_observing(&mut fixture_planner(), &mut req, [end[0], end[1]],
            |position, heading, velocity, angular, dt| {
                // 独立矩形距离与刚体圆弧积分；采样间隙再扣除实体点的位移上界。
                let speed = velocity[0].hypot(velocity[1]) + angular.abs() * radius;
                let count = (speed * dt / 0.0001).ceil().max(1.) as usize;
                for index in 0..=count {
                    let t = dt * index as f64 / count as f64;
                    let (a, b) = if angular.abs() > 1e-9 {
                        ((angular * t).sin() / angular, (1. - (angular * t).cos()) / angular)
                    } else { (t, 0.) };
                    let dx = a * velocity[0] - b * velocity[1];
                    let dy = b * velocity[0] + a * velocity[1];
                    let at = [position[0] + heading.cos() * dx - heading.sin() * dy,
                        position[1] + heading.sin() * dx + heading.cos() * dy, position[2]];
                    let distance = body_distance_to_door(at, heading + angular * t, &cfg,
                        [0., 4. * resolution], [[-50. * resolution, 0.], [33. * resolution, 100. * resolution]]);
                    let clearance = distance - speed * dt / (2. * count as f64);
                    minimum_clearance = minimum_clearance.min(clearance);
                    assert!(clearance > 0., "原斜薄门实体扫掠不能穿墙：{at:?}，净空下界{clearance}");
                }
            })
            .unwrap_or_else(|error| panic!("原斜薄门恢复失败：{error}"));
        assert!(arrived.is_some(), "原斜薄门600拍未到达：{:?}", req.start_position);
        eprintln!("原斜薄门到达拍={arrived:?}，实体净空下界={minimum_clearance}米");
    }
    #[test]
    fn repeated_sample_reference_is_exact_but_new_motion_still_limits_output() {
        let route = [[0.0, 0.0, 0.35], [5.0, 0.0, 0.35]];
        let mut req = request(&route, &[]);
        req.start_velocity = None;
        req.start_acceleration = None;
        req.start_yaw_rate = None;
        let mut planner = fixture_planner();
        planner.step(&req).unwrap();
        req.odometry_timestamp_ns = 100_000_000;
        req.start_position[0] = -0.5; // Existing position feedback activates the acceleration limit.
        let first = planner.step(&req).unwrap();
        let repeated = planner.step(&req).unwrap();
        assert_eq!(first.command_forward, repeated.command_forward);
        assert_eq!(first.command_lateral, repeated.command_lateral);
        assert_eq!(first.command_yaw_rate, repeated.command_yaw_rate);
        req.start_velocity = Some([0.5, 0.0, 0.0]);
        let changed = planner.step(&req).unwrap();
        assert_eq!(changed.motion_boundary_sources, first.motion_boundary_sources,
            "同拍测速更新不重新生成规划边界");
        assert!((changed.command_forward - first.command_forward).abs()
            <= req.config.max_acceleration * req.config.sample_dt + 1e-8);
        let obstacle = wall(req.start_position[0] + 0.5);
        req.obstacles = &obstacle;
        planner.notify_source_changed();
        let error = planner.step(&req).unwrap_err();
        assert!(error.contains("braking sweep"), "新实测速度的制动证明仍必须生效：{error}");
    }

    #[test]
    fn author_registered_frame_initializes_real_session_and_reset_discards_map() {
        let route=[[0.,0.,0.35],[2.,0.,0.35]];
        let req=request(&route,&[]);
        let mut p=Planner::new();
        assert!(p.step(&req).unwrap_err().contains("registered source frame"));
        p.reset();
        p.ingest_registered_frame(&req.config,&[[0.,0.,-1.]],[0.,0.,1.],[0.,0.,0.35],req.odometry_timestamp_ns).unwrap();
        assert!(p.step(&req).is_ok(),"registered frame must be retained before first planner initialization");
        p.reset();
        assert!(p.step(&req).unwrap_err().contains("registered source frame"),"reset cannot replay the retired frame or select static fallback");
    }

    #[test]
    fn registered_execution_obeys_current_surface_obstacles_and_raised_ground() {
        let route = [[0., 0., 0.35], [2., 0., 0.35]];
        let mut req = request(&route, &[]);
        let mut planner = Planner::new();
        // 雷达该方向无命中；另一实测来源给出桌面时，不得依赖雷达盲区继续执行。
        planner.ingest_registered_frame(&req.config, &[[0., 0., -1.]],
            [0., 0., 1.], req.start_position, 0).unwrap();
        assert!(planner.step(&req).is_ok());
        let table = [[0.05, 0., 0.35]];
        req.obstacles = &table;
        req.odometry_timestamp_ns += 100_000_000;
        let failure = planner.step(&req).expect_err("当前已知深度实体必须阻止原雷达未知区域内的动作");
        assert!(failure.contains("BODY_OVERLAP_RECOVERY_FAILED"), "{failure}");
        planner.reset_execution();
        req.obstacles = &[];
        req.ground = &table;
        req.odometry_timestamp_ns += 100_000_000;
        assert!(planner.step(&req).is_err(), "高处支撑面同样是下层机体不能穿越的实体");
        planner.reset_execution();
        req.ground = &[[0.05, 0., 0.0]];
        req.odometry_timestamp_ns += 100_000_000;
        assert!(planner.step(&req).is_ok(), "原地面低于机体碰撞带，旧融合障碍须随新输入撤销；不重放雷达帧");
        planner.reset();
        assert!(planner.step(&req).unwrap_err().contains("registered source frame"),
            "融合层不能伪造必需的注册雷达来源");
    }

    #[test]
    fn author_rejected_first_frames_do_not_initialize_map_or_replace_valid_frame() {
        let route=[[0.,0.,0.35],[2.,0.,0.35]];
        let req=request(&route,&[]);
        // A local Z range is 2.5m and ray maximum 5m: z=4 at origin z=1
        // is deliberately filtered by the original input rule, not a new gate.
        let bad_frames:Vec<Vec<[f64;3]>>=vec![vec![],vec![[f64::NAN,0.,0.]],vec![[0.,0.,4.]]];
        for points in bad_frames {
            let mut p=Planner::new();
            assert!(p.ingest_registered_frame(&req.config,&points,[0.,0.,1.],[0.,0.,0.35],req.odometry_timestamp_ns).is_err());
            assert!(p.step(&req).unwrap_err().contains("registered source frame"));
            p.ingest_registered_frame(&req.config,&[[0.,0.,-1.]],[0.,0.,1.],[0.,0.,0.35],req.odometry_timestamp_ns).unwrap();
            assert!(p.ingest_registered_frame(&req.config,&points,[0.,0.,1.],[0.,0.,0.35],req.odometry_timestamp_ns+1).is_err());
            assert!(p.step(&req).is_ok(),"rejected input cannot clear the previous accepted map");
        }
    }

    #[test]
    fn author_zero_step_band_excludes_support_voxel_and_keeps_obstacle_above_it() {
        // 未声明上下阶能力时阶高为零。实测地面位于0.1米格内0.02米处，格顶高出真实支撑0.08米；
        // 这一格是支撑面本身，不能使整片地面成为机身障碍。高出支撑格的实体仍须阻挡。
        let route = [[0.0, 0.0, 0.37], [3.0, 0.0, 0.37]];
        let ground: Vec<_> = (-20..50)
            .flat_map(|x| (-60..=60).map(move |y| [(x as f64 + 0.5) * 0.1, (y as f64 + 0.5) * 0.1, 0.02]))
            .collect();
        let mut req = request(&route, &[]);
        req.start_position = route[0];
        req.ground = &ground;
        req.config.max_step_up = 0.0;
        req.config.max_step_down = 0.0;
        let (arrived, _) = drive(&mut fixture_planner(), &mut req, [3.0, 0.0])
            .unwrap_or_else(|error| panic!("{error}"));
        assert!(arrived.is_some(), "平地支撑格被当成障碍，最终位置 {:?}", req.start_position);

        // 横贯局部地图的矮坎落在支撑格上方一格，零越阶能力不能跨越。
        let curb: Vec<_> = (-60..=60).map(|y| [1.55, (y as f64 + 0.5) * 0.1, 0.15]).collect();
        let mut req = request(&route, &curb);
        req.start_position = route[0];
        req.ground = &ground;
        req.config.max_step_up = 0.0;
        req.config.max_step_down = 0.0;
        let outcome = drive(&mut fixture_planner(), &mut req, [3.0, 0.0]);
        assert!(!matches!(outcome, Ok((Some(_), _))), "零越阶机体越过了支撑格上方的矮坎");
        assert!(req.start_position[0] + 0.35 < 1.5, "机身应停在矮坎前：{:?}", req.start_position);
    }

    fn go2_support_request() -> PlanRequest<'static> {
        let mut req=request(&[],&[]);
        req.config.resolution=f64::from(0.05_f32);
        req.config.body_length=0.7363955361934972;
        req.config.body_width=0.41704432480493203;
        req.config.body_height=0.4108918899416131;
        req.config.body_center_x=-0.0026277680952758176;
        req.config.body_center_y=0.003659895132201413;
        req.config.body_z_min=-0.3923118907927897;
        req.config.body_z_max=0.018579999148823356;
        req.config.body_to_ground=0.36385;
        req.config.max_step_up=0.;req.config.max_step_down=0.;
        req.start_position=[0.,0.,0.0123];
        req.goal_yaw=Some(0.);
        req
    }

    fn observe_support(planner:&mut Planner,req:&mut PlanRequest<'_>,points:&[[f64;3]],members:&[u32]) -> Result<PlanResult,String> {
        let mut result=None;
        for _ in 0..3 {
            req.odometry_timestamp_ns+=100_000_000;
            planner.ingest_registered_frame_with_ground(&req.config,points,members,req.start_position,
                req.start_position,req.odometry_timestamp_ns).unwrap();
            result=Some(planner.step(req));
        }
        result.unwrap()
    }

    #[test]
    fn registered_support_members_remove_only_quantized_floor_overlap() {
        // Go2原探索实测：下沿约-0.30155，真实支撑-0.347所在格顶-0.30越入1.55毫米。
        let points=[[0.025,0.025,-0.347]];
        let mut req=go2_support_request();
        let original=observe_support(&mut Planner::new(),&mut req,&points,&[]).unwrap_err();
        assert!(original.contains("Current body pose is occupied"),"{original}");
        let accepted=observe_support(&mut Planner::new(),&mut req,&points,&[0]).unwrap();
        assert_eq!([accepted.command_forward,accepted.command_lateral,accepted.command_yaw_rate],[0.;3]);
    }

    #[test]
    fn registered_support_keeps_above_mixed_and_unknown_occupancy() {
        for (case,(points,members)) in [
            // 上方实体与地面使用可独立看见的射线；实体后方地面不能同时作为自由穿越证据。
            (vec![[0.225,0.125,-0.347],[0.025,0.025,-0.29]],vec![0]),
            (vec![[0.025,0.025,-0.347],[0.026,0.026,-0.315]],vec![0]),
            (vec![[0.025,0.025,-0.300]],vec![0]),
            (vec![[0.025,0.025,-0.347]],vec![]),
        ].into_iter().enumerate() {
            let mut req=go2_support_request();
            let error=observe_support(&mut Planner::new(),&mut req,&points,&members)
                .expect_err(&format!("反例{case}必须保留原占据"));
            assert!(error.contains("Current body pose is occupied"),"{error}");
        }
    }

    #[test]
    fn registered_support_cannot_erase_unreleased_nonmember_history() {
        let mut req=go2_support_request();let mut planner=Planner::new();
        assert!(observe_support(&mut planner,&mut req,&[[0.025,0.025,-0.315]],&[]).is_err());
        assert!(observe_support(&mut planner,&mut req,&[[0.025,0.025,-0.347]],&[0]).is_err(),
            "同格新地面成员不能清掉仍有原概率占据的非成员");
    }

    #[test]
    fn registered_support_retirement_rebuild_and_invalid_members_keep_identity() {
        let mut req=go2_support_request();let mut planner=Planner::new();let points=[[0.025,0.025,-0.347]];
        observe_support(&mut planner,&mut req,&points,&[0]).unwrap();
        planner.reset_execution();
        assert!(planner.step(&req).is_ok(),"取消只退休执行，保留同源地图与地面身份");
        for members in [&[1][..],&[0,0][..]] {
            assert!(planner.ingest_registered_frame_with_ground(&req.config,&points,members,
                req.start_position,req.start_position,req.odometry_timestamp_ns+1).is_err());
            assert!(planner.step(&req).is_ok(),"非法成员不能替换已有合法帧");
        }
        planner.reset_execution();req.config.body_width+=0.01;
        assert!(planner.configure(&req.config).unwrap());
        let error=observe_support(&mut planner,&mut req,&points,&[]).unwrap_err();
        assert!(error.contains("Current body pose is occupied"),"地图重建必须清除旧支撑身份：{error}");
    }

    #[test]
    fn short_reference_preserves_real_detour_and_clear_final_direction() {
        // 原捕获10点短绕行。按0.5米距离抽样只剩首尾，末段从-2.82弧度变成-1.57弧度，
        // 同一终点的真实机体因此撞到下方实体；保留拐角后仍由原求解器和碰撞检查验收。
        let route=[
            [-0.19957526078069113,0.013856169486265677,0.024536172545340695],
            [-0.37500000558793545,-0.02500000037252903,0.01697498807907105],
            [-0.4250000063329935,-0.07500000111758709,0.01697498807907105],
            [-0.37500000558793545,-0.12500000186264515,0.01697498807907105],
            [-0.3250000048428774,-0.1750000026077032,0.01697498807907105],
            [-0.27500000409781933,-0.22500000335276127,0.01697498807907105],
            [-0.22500000335276127,-0.27500000409781933,0.01697498807907105],
            [-0.1750000026077032,-0.3250000048428774,0.01697498807907105],
            [-0.12500000186264515,-0.37500000558793545,0.01697498807907105],
            [-0.2,-0.4,0.01697498807907105],
        ];
        let obstacles=[
            [-0.296437531709671,-0.77738356590271,-0.0840628445148468],
            [-0.3430383503437042,-0.7771164178848267,-0.008730334229767323],
            [-0.31034231185913086,-0.7773557901382446,-0.061080947518348694],
            [-0.323816180229187,-0.7771331071853638,0.014047613367438316],
        ];
        for duplicates in [false,true] {
            let reference:Vec<_>=route.iter().flat_map(|p|std::iter::repeat(*p).take(if duplicates {2}else {1})).collect();
            let mut req=request(&reference,&obstacles);req.config=go2_support_request().config;
            req.config.max_acceleration=0.5;req.config.planning_horizon=5.;req.config.sample_dt=0.05;
            req.start_position=route[0];req.start_yaw=-1.601;req.start_velocity=None;
            req.start_acceleration=None;req.start_yaw_rate=None;
            let mut planner=fixture_planner();let mut produced=false;
            for tick in 0..20 {
                req.odometry_timestamp_ns=1_000_000_000+tick*50_000_000;
                let result=planner.step(&req).unwrap_or_else(|reason|panic!("重复点={duplicates}，原真实短绕行被拒绝：{reason}"));
                if !result.samples.is_empty() && [result.command_forward,result.command_lateral,result.command_yaw_rate]
                    .into_iter().any(|value|value.abs()>1e-10) {produced=true;break;}
            }
            assert!(produced,"重复点={duplicates}，原短绕行虽接纳但未生成局部轨迹及非零命令");
        }
    }

    #[test]
    fn rejected_duplicate_reference_retires_previous_target_and_controller() {
        let route=[[0.,0.,0.35],[2.,0.,0.35]];
        let repeated=[[0.5,0.,0.35];3];
        let mut req=request(&route,&[]);let mut planner=fixture_planner();let mut active=false;
        for tick in 0..10 {
            req.odometry_timestamp_ns=1_000_000_000+tick*100_000_000;
            let output=planner.step(&req).unwrap();
            if !output.samples.is_empty(){active=true;break;}
        }
        assert!(active,"反例必须先建立真实旧目标和控制器轨迹");
        let previous_plans=planner.plan_count();
        req.reference=&repeated;req.odometry_timestamp_ns+=100_000_000;
        let reason=planner.step(&req).expect_err("全相邻重复的新参考必须明确拒绝");
        assert!(reason.contains("two distinct consecutive points"),"{reason}");
        req.reference=&[];req.goal_yaw=Some(req.start_yaw);req.odometry_timestamp_ns+=100_000_000;
        let stopped=planner.step(&req).unwrap();
        assert!(stopped.samples.is_empty());
        assert_eq!([stopped.command_forward,stopped.command_lateral,stopped.command_yaw_rate],[0.;3]);
        assert_eq!(planner.plan_count(),previous_plans,"拒绝后不能重发旧轨迹");
        req.reference=&route;req.goal_yaw=None;req.odometry_timestamp_ns+=100_000_000;
        let installed=planner.step(&req).unwrap();assert!(installed.replanned,"新参考必须由新规划接续，不能复活旧轨迹");
    }

    #[test]
    fn ordered_reference_keeps_dense_door_height_transition_and_endpoint() {
        // 原窄门的102点：最初7毫米过渡后是一米长的实测平面。共线化后的原全局多项式
        // 会落到z=-5.478米；有序PCT参考自身仅在0.395至0.4米，局部目标必须取同一真实折线。
        let fine=f64::from(0.01_f32);let y=33.*fine/2.;let level=0.4-fine/2.;
        let mut route=vec![[-0.5,y,0.4]];
        route.extend((-50..50).map(|x|[(f64::from(x)+0.5)*fine,y,level]));
        route.push([0.5,y,level]);assert_eq!(route.len(),102);
        let mut req=request(&route,&[]);req.start_position=route[0];req.config.body_to_ground=0.4;
        req.config.max_velocity=0.3;req.config.max_acceleration=1.;req.config.planning_horizon=3.5;
        req.config.body_length=0.6;req.config.body_width=32.*fine;req.config.sample_dt=0.05;
        let mut planner=fixture_planner();let mut produced=false;
        for tick in 0..20 {
            req.odometry_timestamp_ns=1_000_000_000+tick*50_000_000;
            let result=planner.step(&req).unwrap();
            if !result.samples.is_empty() {
                assert!((result.local_target[0]-0.5).abs()<1e-8 && (result.local_target[2]-level).abs()<1e-8,
                    "局部目标越出原PCT终点：{:?}",result.local_target);
                assert!(result.reference_progress<=result.reference_duration+1e-8);
                let original=planner.original_samples(req.config.sample_dt);
                let low=original.iter().map(|p|p.position[2]).fold(f64::INFINITY,f64::min);
                let high=original.iter().map(|p|p.position[2]).fold(f64::NEG_INFINITY,f64::max);
                // 同一102点输入在63b43ed0原局部求解器的已有拟合范围；参考修复不改变该样条行为。
                assert!((low-0.39416981200047052).abs()<1e-8 && (high-0.39798499736324322).abs()<1e-8,
                    "原局部样条高度行为发生变化：{low}至{high}");
                assert!(result.command_forward>0.);
                produced=true;break;
            }
        }
        assert!(produced,"原102点高度过渡必须生成局部轨迹，不能停原位反复求解");
    }

    #[test]
    fn author_terminal_yaw_honors_explicit_business_masks() {
        let mut req=request(&[],&[]);req.goal_yaw=Some(0.5);
        let free=fixture_planner().step(&req).unwrap();
        assert!(free.command_yaw_rate>0.0&&free.samples.is_empty());
        let allowed=[[-2.,-2.],[2.,-2.],[2.,2.],[-2.,2.]];
        req.boundary=&allowed;
        let accepted=fixture_planner().step(&req).unwrap();
        assert_eq!(accepted.command_yaw_rate,free.command_yaw_rate);
        let outside=[[2.,2.],[3.,2.],[3.,3.],[2.,3.]];
        req.boundary=&outside;
        assert!(fixture_planner().step(&req).unwrap_err().contains("explicit user boundary"));
        req.boundary=&[];req.forbidden_columns=&[[0,0]];
        assert!(fixture_planner().step(&req).unwrap_err().contains("forbidden mask"));
    }

}
