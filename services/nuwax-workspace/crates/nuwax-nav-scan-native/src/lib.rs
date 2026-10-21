//! 原 SCAN（局部轨迹规划器）的无 ROS（机器人操作系统）接口。
use std::ffi::{c_char, CStr};

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
}

pub struct PlanRequest<'a> {
    /// 实测机身障碍体素中心，正常地面和可走台阶应归入 ground。
    pub obstacles: &'a [[f64; 3]],
    /// 地面实测坐标；缺测处沿用全局机身参考高度。
    pub ground: &'a [[f64; 3]],
    /// PCT（点云地形规划器）输出的世界坐标机体中心参考路径。
    pub reference: &'a [[f64; 3]],
    pub start_position: [f64; 3],
    pub start_velocity: [f64; 3],
    pub start_acceleration: [f64; 3],
    pub start_yaw: f64,
    pub start_yaw_rate: f64,
    /// 里程计采样时间；重复同一采样不得推进原控制器时钟。
    pub odometry_timestamp_ns: i64,
    pub config: Config,
    pub boundary: &'a [[f64; 2]],
    pub forbidden_columns: &'a [[i32; 2]],
    pub goal_yaw: Option<f64>,
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

#[derive(Debug, Clone)]
pub struct PlanResult {
    pub samples: Vec<TrajectorySample>,
    pub duration: f64,
    /// 当前局部窗口目标在原折线参考轨迹中的时间位置。
    pub reference_progress: f64,
    pub reference_duration: f64,
    pub local_target: [f64; 3],
    pub command_forward: f64,
    pub command_yaw_rate: f64,
    pub command_duration: f64,
    pub replanned: bool,
}

#[repr(C)]
struct NativeOutput {
    duration: f64,
    reference_progress: f64,
    reference_duration: f64,
    local_target: [f64; 3],
    command_forward: f64,
    command_yaw_rate: f64,
    command_duration: f64,
    samples: *mut TrajectorySample,
    sample_count: usize,
    replanned: u32,
}
extern "C" {
    fn nuwax_scan_session_create() -> *mut std::ffi::c_void;
    fn nuwax_scan_session_destroy(session: *mut std::ffi::c_void);
    fn nuwax_scan_session_reset(session: *mut std::ffi::c_void);
    fn nuwax_scan_session_set_local_planning(session: *mut std::ffi::c_void, enabled: i32);
    fn nuwax_scan_session_plan_count(session: *const std::ffi::c_void) -> u64;
    fn nuwax_scan_session_plan(
        session: *mut std::ffi::c_void,
        config: *const Config,
        obstacles: *const f64,
        obstacle_count: usize,
        ground: *const f64,
        ground_count: usize,
        reference: *const f64,
        reference_count: usize,
        start_position: *const f64,
        start_velocity: *const f64,
        start_acceleration: *const f64,
        start_yaw: f64,
        start_yaw_rate: f64,
        boundary: *const f64,
        boundary_count: usize,
        forbidden_columns: *const i32,
        forbidden_count: usize,
        goal_yaw: f64,
        odometry_timestamp_ns: i64,
        output: *mut NativeOutput,
        error: *mut c_char,
        error_capacity: usize,
    ) -> i32;
    fn nuwax_scan_free(output: *mut NativeOutput);
}

/// 单次规划供组件诊断；持续执行使用持有原轨迹的 Planner。
pub fn plan(request: &PlanRequest<'_>) -> Result<PlanResult, String> {
    call_native(request, std::ptr::null_mut())
}

/// 原 SCAN 样条与闭环控制器的单线程拥有者。
#[derive(Debug)]
pub struct Planner {
    handle: *mut std::ffi::c_void,
    local_planning: bool,
}
// 句柄只由 &mut self 串行调用，可随原 PlannerWorker 移交；不实现 Sync。
unsafe impl Send for Planner {}
impl Default for Planner {
    fn default() -> Self {
        Self::new()
    }
}
impl Planner {
    pub fn new() -> Self {
        Self {
            handle: std::ptr::null_mut(),
            local_planning: true,
        }
    }
    /// 实验台对照开关，默认开启。关闭后跳过局部 A* 与避障优化，直接跟随全局参考；
    /// 原闭环控制器、实测障碍输入与实际机身检查不变，被挡时返回错误交原链停车重规划。
    pub fn set_local_planning(&mut self, enabled: bool) {
        self.local_planning = enabled;
        if !self.handle.is_null() {
            unsafe { nuwax_scan_session_set_local_planning(self.handle, i32::from(enabled)) }
        }
    }
    pub fn step(&mut self, request: &PlanRequest<'_>) -> Result<PlanResult, String> {
        if self.handle.is_null() {
            self.handle = unsafe { nuwax_scan_session_create() };
            if self.handle.is_null() {
                return Err("无法分配 SCAN 原生规划器".into());
            }
            unsafe { nuwax_scan_session_set_local_planning(self.handle, i32::from(self.local_planning)) }
        }
        call_native(request, self.handle)
    }
    pub fn reset(&mut self) {
        if !self.handle.is_null() {
            unsafe { nuwax_scan_session_reset(self.handle) }
        }
    }
    pub fn plan_count(&self) -> u64 {
        unsafe { nuwax_scan_session_plan_count(self.handle) }
    }
}
impl Drop for Planner {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            unsafe { nuwax_scan_session_destroy(self.handle) }
        }
    }
}

fn call_native(
    request: &PlanRequest<'_>,
    handle: *mut std::ffi::c_void,
) -> Result<PlanResult, String> {
    let mut output = NativeOutput {
        duration: 0.0,
        reference_progress: 0.0,
        reference_duration: 0.0,
        local_target: [0.0; 3],
        command_forward: 0.0,
        command_yaw_rate: 0.0,
        command_duration: 0.0,
        samples: std::ptr::null_mut(),
        sample_count: 0,
        replanned: 0,
    };
    let mut error = [0 as c_char; 512];
    // 输入切片存活至同步 C 接口返回，输出由 C++ 分配并由配套接口释放。
    let status = unsafe {
        nuwax_scan_session_plan(
            handle,
            &request.config,
            request.obstacles.as_ptr().cast(),
            request.obstacles.len(),
            request.ground.as_ptr().cast(),
            request.ground.len(),
            request.reference.as_ptr().cast(),
            request.reference.len(),
            request.start_position.as_ptr(),
            request.start_velocity.as_ptr(),
            request.start_acceleration.as_ptr(),
            request.start_yaw,
            request.start_yaw_rate,
            request.boundary.as_ptr().cast(),
            request.boundary.len(),
            request.forbidden_columns.as_ptr().cast(),
            request.forbidden_columns.len(),
            request.goal_yaw.unwrap_or(f64::NAN),
            request.odometry_timestamp_ns,
            &mut output,
            error.as_mut_ptr(),
            error.len(),
        )
    };
    if status != 0 {
        let message = unsafe { CStr::from_ptr(error.as_ptr()) }
            .to_string_lossy()
            .into_owned();
        unsafe { nuwax_scan_free(&mut output) };
        return Err(message);
    }
    let samples = if output.sample_count == 0 {
        Vec::new()
    } else {
        unsafe { std::slice::from_raw_parts(output.samples, output.sample_count) }.to_vec()
    };
    let result = PlanResult {
        samples,
        duration: output.duration,
        reference_progress: output.reference_progress,
        reference_duration: output.reference_duration,
        local_target: output.local_target,
        command_forward: output.command_forward,
        command_yaw_rate: output.command_yaw_rate,
        command_duration: output.command_duration,
        replanned: output.replanned != 0,
    };
    unsafe { nuwax_scan_free(&mut output) };
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
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
        }
    }
    fn request<'a>(reference: &'a [[f64; 3]], obstacles: &'a [[f64; 3]]) -> PlanRequest<'a> {
        PlanRequest {
            reference,
            obstacles,
            ground: &[],
            start_position: [0.0, 0.0, 0.35],
            start_velocity: [0.0; 3],
            start_acceleration: [0.0; 3],
            start_yaw: 0.0,
            start_yaw_rate: 0.0,
            odometry_timestamp_ns: 0,
            config: config(),
            boundary: &[],
            forbidden_columns: &[],
            goal_yaw: None,
        }
    }
    #[test]
    fn entirely_unknown_corridor_is_traversable() {
        let route = [[0.0, 0.0, 0.35], [8.0, 0.0, 0.35]];
        let result = plan(&request(&route, &[])).unwrap();
        assert!(result.command_forward > 0.0);
        assert!(result.local_target[0] > 1.8 && result.local_target[0] < 2.3);
        assert!(result.reference_progress < result.reference_duration);
        assert!((result.samples.first().unwrap().position[0]).abs() < 1e-8);
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
    fn measured_velocity_and_acceleration_are_preserved() {
        let route = [[0.0, 0.0, 0.35], [5.0, 0.0, 0.35]];
        let mut req = request(&route, &[]);
        req.start_velocity = [0.25, 0.03, 0.0];
        req.start_acceleration = [0.1, 0.0, 0.0];
        let result = plan(&req).unwrap();
        let first = &result.samples[0];
        for i in 0..3 {
            assert!((first.velocity[i] - req.start_velocity[i]).abs() < 1e-7);
            assert!((first.acceleration[i] - req.start_acceleration[i]).abs() < 1e-7);
        }
        assert!(
            (result.command_forward - 0.25).abs()
                <= req.config.max_acceleration * req.config.sample_dt + 1e-8
        );
    }
    #[test]
    fn reference_windows_continue_to_the_final_goal() {
        let route = [[0.0, 0.0, 0.35], [6.0, 0.0, 0.35]];
        let mut req = request(&route, &[]);
        let first = plan(&req).unwrap();
        req.start_position = first.local_target;
        req.start_velocity = [0.3, 0.0, 0.0];
        let second = plan(&req).unwrap();
        assert!(second.reference_progress > first.reference_progress);
        req.start_position = second.local_target;
        let third = plan(&req).unwrap();
        assert!((third.reference_progress - third.reference_duration).abs() < 1e-6);
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
        assert!(plan(&req).is_err());
    }
    #[test]
    fn ground_points_do_not_become_body_obstacles() {
        let route = [[0.0, 0.0, 0.35], [4.0, 0.0, 0.35]];
        let ground: Vec<_> = (0..40).map(|x| [x as f64 * 0.1, 0.0, 0.0]).collect();
        let mut req = request(&route, &[]);
        req.ground = &ground;
        assert!(plan(&req).unwrap().command_forward > 0.0);
    }
    /// 横跨直线参考的实测墙体。
    fn wall(x: f64) -> Vec<[f64; 3]> {
        (-5..=5)
            .flat_map(|y| (1..=7).map(move |z| [x, y as f64 * 0.1, z as f64 * 0.1]))
            .collect()
    }
    /// 按差动命令积分实际运动，返回到达目标时的控制拍及最大|y|偏离。
    fn drive(
        planner: &mut Planner,
        req: &mut PlanRequest<'_>,
        goal: [f64; 2],
    ) -> Result<(Option<usize>, f64), String> {
        let mut lateral: f64 = 0.0;
        for tick in 0..600 {
            req.odometry_timestamp_ns = tick as i64 * 100_000_000;
            let result = planner.step(req).map_err(|error| {
                format!(
                    "tick {tick}, pose {:?}, yaw {}, speed {:?}: {error}",
                    req.start_position, req.start_yaw, req.start_velocity
                )
            })?;
            let next_v = result.command_forward;
            let next_w = result.command_yaw_rate;
            let v = (req.start_velocity[0].hypot(req.start_velocity[1]) + next_v) / 2.0;
            let w = (req.start_yaw_rate + next_w) / 2.0;
            let dt = result.command_duration;
            let yaw = req.start_yaw;
            if w.abs() > 1e-9 {
                req.start_position[0] += v / w * ((yaw + w * dt).sin() - yaw.sin());
                req.start_position[1] += v / w * (yaw.cos() - (yaw + w * dt).cos());
            } else {
                req.start_position[0] += v * dt * yaw.cos();
                req.start_position[1] += v * dt * yaw.sin();
            }
            req.start_yaw += w * dt;
            req.start_yaw_rate = next_w;
            req.start_velocity = [v * req.start_yaw.cos(), v * req.start_yaw.sin(), 0.0];
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
            drive(&mut Planner::new(), &mut req, [4.0, 0.0]).unwrap_or_else(|error| panic!("{error}"));
        assert!(arrived.is_some(), "final position {:?}", req.start_position);
    }
    #[test]
    fn disabled_local_planning_follows_reference_without_counting_local_plans() {
        // 全局参考机身高度与实测起点差2.5厘米，竖向不能压慢水平跟随。
        let route = [[0.0, 0.0, 0.35], [0.08, 0.0, 0.325], [4.0, 0.0, 0.325]];
        let mut req = request(&route, &[]);
        let mut planner = Planner::new();
        planner.set_local_planning(false);
        let (arrived, lateral) =
            drive(&mut planner, &mut req, [4.0, 0.0]).unwrap_or_else(|error| panic!("{error}"));
        // 4米、0.6米/秒、0.8米/秒²的匀加减速约需8秒，即80个0.1秒控制拍。
        assert!(arrived.is_some_and(|tick| tick < 120), "arrived {arrived:?}, final position {:?}", req.start_position);
        assert!(lateral < 0.02, "直接跟随不应离开直线参考：{lateral}");
        assert_eq!(planner.plan_count(), 0, "关闭后不累计局部规划");
    }
    #[test]
    fn disabled_local_planning_keeps_speed_when_goal_enters_the_window() {
        // 实验台的5米视距和1米/秒：终点进入窗口后末速为零，带实测速度重算时不能把轨迹拉成慢行。
        let route = [[0.0, 0.0, 0.35], [7.0, 0.0, 0.35]];
        let mut req = request(&route, &[]);
        req.config.resolution = 0.05;
        req.config.planning_horizon = 5.0;
        req.config.max_velocity = 1.0;
        req.config.max_acceleration = 1.0;
        let mut planner = Planner::new();
        planner.set_local_planning(false);
        let (arrived, _) =
            drive(&mut planner, &mut req, [7.0, 0.0]).unwrap_or_else(|error| panic!("{error}"));
        // 7米、1米/秒、1米/秒²约需9秒，即90个0.1秒控制拍。
        assert!(arrived.is_some_and(|tick| tick < 130), "arrived {arrived:?}, final position {:?}", req.start_position);
    }
    #[test]
    fn disabled_local_planning_stops_before_wall_on_reference() {
        let route = [[0.0, 0.0, 0.35], [4.0, 0.0, 0.35]];
        let obstacles = wall(1.5);
        let mut req = request(&route, &obstacles);
        let mut planner = Planner::new();
        planner.set_local_planning(false);
        let error = drive(&mut planner, &mut req, [4.0, 0.0]).unwrap_err();
        assert!(error.contains("local planning disabled"), "{error}");
        assert!(req.start_position[0] + 0.35 < 1.5, "机身应停在墙前：{:?}", req.start_position);
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
        assert!(plan(&req).is_err());
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
    fn goal_behind_body_starts_with_a_checked_turn() {
        let route = [[0.0, 0.0, 0.35], [-4.0, 0.0, 0.35]];
        let result = plan(&request(&route, &[])).unwrap();
        assert!(result.command_yaw_rate.abs() > 0.0);
        assert_eq!(result.command_forward, 0.0);
    }
    #[test]
    fn terminal_rotation_brakes_using_measured_angular_velocity() {
        let mut req = request(&[], &[]);
        req.goal_yaw = Some(1.0);
        for _ in 0..100 {
            let result = plan(&req).unwrap();
            assert!(
                (result.command_yaw_rate - req.start_yaw_rate).abs()
                    <= req.config.max_yaw_acceleration * req.config.sample_dt + 1e-8
            );
            req.start_yaw += result.command_yaw_rate * result.command_duration;
            req.start_yaw_rate = result.command_yaw_rate;
            if (req.start_yaw - 1.0).abs() < 0.001 && req.start_yaw_rate.abs() < 0.01 {
                return;
            }
        }
        panic!("yaw {}, rate {}", req.start_yaw, req.start_yaw_rate);
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
        let result = plan(&req).unwrap();
        assert!(result.command_forward > 0.0);
        assert!(
            result.command_yaw_rate.abs() < 1e-6,
            "本可行斜直线不能被局部规划扭成转向: {:?}",
            result.command_yaw_rate
        );
        for sample in result.samples {
            let expected =
                33.0 * resolution / 2.0 + (sample.position[0] - 2.0 * resolution) * yaw.tan();
            assert!((sample.position[1] - expected).abs() < 1e-7);
        }
    }
}
