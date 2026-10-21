//! 固定上游PCT源码的无ROS封装，仅从有效实测地面构建全局路线。
use std::cell::RefCell;
use std::ffi::{c_char, c_void, CStr};
use std::ptr::NonNull;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

thread_local! {
    static CANCELLATIONS: RefCell<Vec<Arc<AtomicBool>>> = const { RefCell::new(Vec::new()) };
}
/// 在计算线程绑定现有拥有方的取消信号；嵌套调用和异常展开后恢复原作用域。
pub fn with_planning_cancellation<T>(flag: Arc<AtomicBool>, operation: impl FnOnce() -> T) -> T {
    struct Restore(usize);
    impl Drop for Restore {
        fn drop(&mut self) {
            CANCELLATIONS.with(|scopes| scopes.borrow_mut().truncate(self.0));
        }
    }
    let previous = CANCELLATIONS.with(|scopes| {
        let mut scopes = scopes.borrow_mut();
        let previous = scopes.len();
        scopes.push(flag);
        previous
    });
    let _restore = Restore(previous);
    operation()
}
/// 当前同步计算是否已被所属任务取消，不创建或改变地图状态。
pub fn planning_cancelled() -> bool {
    CANCELLATIONS.with(|scopes| {
        scopes
            .borrow()
            .iter()
            .any(|flag| flag.load(Ordering::Acquire))
    })
}

#[derive(Clone, Debug)]
pub struct Config {
    pub resolution: f64,
    pub point_resolution: f64,
    pub robot_height: f64,
    pub body_to_ground: f64,
    pub max_step: f64,
    pub max_step_down: f64,
    /// 最大连续坡度，单位为弧度；独立台阶由max_step约束。
    pub max_slope: f64,
    /// 无多边形外形时使用；已提供足迹时不使用外包圆作硬膨胀。
    pub footprint_radius: f64,
    pub body_z_min: f64,
    pub body_z_max: f64,
    pub search_margin: f64,
    pub footprint: Vec<[f64; 2]>,
    pub boundary: Vec<[f64; 2]>,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            resolution: 0.15,
            point_resolution: 0.05,
            robot_height: 0.5,
            body_to_ground: 0.4,
            max_step: 0.2,
            max_step_down: 0.2,
            max_slope: 30.0_f64.to_radians(),
            footprint_radius: 0.3,
            body_z_min: -0.2,
            body_z_max: 0.2,
            search_margin: 3.0,
            footprint: Vec::new(),
            boundary: Vec::new(),
        }
    }
}
/// XY格索引按Config.resolution，ground保存原实测高度；无顶面时ceiling为正无穷。
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Cell {
    pub x: i32,
    pub y: i32,
    pub ground: f64,
    pub ceiling: f64,
}
/// 同一XY列允许多个占据区间，端点为米制真实区间。
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Band {
    pub x: i32,
    pub y: i32,
    pub low: f64,
    pub high: f64,
}
#[derive(Clone, Debug)]
pub struct Route {
    /// 与起终点相同的机体中心坐标。
    pub path: Vec<[f64; 3]>,
    pub elapsed_ms: f64,
    pub expanded: usize,
}
#[repr(C)]
struct ConfigRaw {
    resolution: f64,
    point_resolution: f64,
    robot_height: f64,
    body_to_ground: f64,
    max_step: f64,
    max_step_down: f64,
    max_slope: f64,
    footprint_radius: f64,
    body_z_min: f64,
    body_z_max: f64,
    search_margin: f64,
    footprint: *const f64,
    footprint_len: usize,
    boundary: *const f64,
    boundary_len: usize,
}
#[repr(C)]
#[derive(Default)]
struct ResultRaw {
    points: *mut f64,
    length: usize,
    elapsed_ms: f64,
    expanded: usize,
}
unsafe extern "C" {
    fn pct_create(
        config: *const ConfigRaw,
        cells: *const Cell,
        ncells: usize,
        bands: *const Band,
        nbands: usize,
        points: *const f64,
        npoints: usize,
        err: *mut c_char,
        cap: usize,
    ) -> *mut c_void;
    fn pct_destroy(planner: *mut c_void);
    fn pct_geometry_sizes(planner: *const c_void, ncells: *mut usize, nbands: *mut usize);
    fn pct_geometry_copy(planner: *const c_void, cells: *mut Cell, bands: *mut Band);
    fn pct_result_free(result: *mut ResultRaw);
    fn pct_plan(
        planner: *mut c_void,
        start: *const f64,
        goal: *const f64,
        result: *mut ResultRaw,
        cancel: extern "C" fn(*const c_void) -> bool,
        user: *const c_void,
        err: *mut c_char,
        cap: usize,
    ) -> i32;
    #[cfg(test)]
    fn pct_retry_regression() -> bool;
}
/// 内核由实例唯一持有，所有搜索均通过可变借用串行执行。
pub struct Planner {
    native: NonNull<c_void>,
}
// 原生实例无共享全局可变状态，移动所有权不会产生并发访问。
unsafe impl Send for Planner {}
impl Drop for Planner {
    fn drop(&mut self) {
        unsafe { pct_destroy(self.native.as_ptr()) }
    }
}
fn native_error(error: &[c_char]) -> String {
    unsafe { CStr::from_ptr(error.as_ptr()) }
        .to_string_lossy()
        .into_owned()
}
extern "C" fn cancelled(user: *const c_void) -> bool {
    if user.is_null() {
        planning_cancelled()
    } else {
        unsafe { &*user.cast::<AtomicBool>() }.load(Ordering::Acquire)
    }
}
impl Planner {
    pub fn new(config: Config, points: &[[f64; 3]]) -> Result<Self, String> {
        Self::create(config, &[], &[], points)
    }
    pub fn from_cells(config: Config, cells: &[Cell], bands: &[Band]) -> Result<Self, String> {
        Self::create(config, cells, bands, &[])
    }
    fn create(
        config: Config,
        cells: &[Cell],
        bands: &[Band],
        points: &[[f64; 3]],
    ) -> Result<Self, String> {
        let raw = ConfigRaw {
            resolution: config.resolution,
            point_resolution: config.point_resolution,
            robot_height: config.robot_height,
            body_to_ground: config.body_to_ground,
            max_step: config.max_step,
            max_step_down: config.max_step_down,
            max_slope: config.max_slope,
            footprint_radius: config.footprint_radius,
            body_z_min: config.body_z_min,
            body_z_max: config.body_z_max,
            search_margin: config.search_margin,
            footprint: config.footprint.as_ptr().cast(),
            footprint_len: config.footprint.len(),
            boundary: config.boundary.as_ptr().cast(),
            boundary_len: config.boundary.len(),
        };
        let mut error = [0; 512];
        // 创建期间C++复制全部切片；返回后不保留任何Rust借用。
        let native = unsafe {
            pct_create(
                &raw,
                cells.as_ptr(),
                cells.len(),
                bands.as_ptr(),
                bands.len(),
                points.as_ptr().cast(),
                points.len(),
                error.as_mut_ptr(),
                error.len(),
            )
        };
        NonNull::new(native)
            .map(|native| Self { native })
            .ok_or_else(|| native_error(&error))
    }
    /// 导出相同原生实测分类结果供局部规划复用。
    pub fn geometry(&self) -> (Vec<Cell>, Vec<Band>) {
        let (mut ncells, mut nbands) = (0, 0);
        unsafe { pct_geometry_sizes(self.native.as_ptr(), &mut ncells, &mut nbands) };
        let mut cells = Vec::with_capacity(ncells);
        let mut bands = Vec::with_capacity(nbands);
        unsafe {
            pct_geometry_copy(self.native.as_ptr(), cells.as_mut_ptr(), bands.as_mut_ptr());
            cells.set_len(ncells);
            bands.set_len(nbands);
        }
        (cells, bands)
    }
    pub fn plan(&mut self, start: [f64; 3], goal: [f64; 3]) -> Result<Route, String> {
        self.plan_inner(start, goal, std::ptr::null())
    }
    pub fn plan_cancellable(
        &mut self,
        start: [f64; 3],
        goal: [f64; 3],
        cancel: &AtomicBool,
    ) -> Result<Route, String> {
        self.plan_inner(start, goal, (cancel as *const AtomicBool).cast())
    }
    fn plan_inner(
        &mut self,
        start: [f64; 3],
        goal: [f64; 3],
        cancel: *const c_void,
    ) -> Result<Route, String> {
        let mut result = ResultRaw::default();
        let mut error = [0; 512];
        // 调用同步完成，取消指针在整个原生搜索期间有效。
        let status = unsafe {
            pct_plan(
                self.native.as_ptr(),
                start.as_ptr(),
                goal.as_ptr(),
                &mut result,
                cancelled,
                cancel,
                error.as_mut_ptr(),
                error.len(),
            )
        };
        let route = if status == 0 && result.length > 0 {
            let values = unsafe { std::slice::from_raw_parts(result.points, result.length * 3) };
            Ok(Route {
                path: values.chunks_exact(3).map(|v| [v[0], v[1], v[2]]).collect(),
                elapsed_ms: result.elapsed_ms,
                expanded: result.expanded,
            })
        } else {
            Err(native_error(&error))
        };
        unsafe { pct_result_free(&mut result) };
        route
    }
}

/// 原始点分类与Planner::new共用C++实现，保留实际测量高度与占据区间。
pub fn classify_points(
    config: &Config,
    points: &[[f64; 3]],
) -> Result<(Vec<Cell>, Vec<Band>), String> {
    Ok(Planner::new(config.clone(), points)?.geometry())
}

#[cfg(test)]
mod tests {
    use super::*;
    fn config() -> Config {
        Config {
            resolution: 0.1,
            point_resolution: 0.03,
            footprint_radius: 0.12,
            max_step: 0.16,
            robot_height: 0.45,
            body_to_ground: 0.4,
            body_z_min: -0.2,
            body_z_max: 0.15,
            search_margin: 1.0,
            ..Default::default()
        }
    }
    fn flat_cells(x: std::ops::Range<i32>, y: std::ops::Range<i32>) -> Vec<Cell> {
        x.flat_map(|x| y.clone().map(move |y| Cell {
            x, y, ground: 0.0, ceiling: f64::INFINITY,
        })).collect()
    }
    #[test]
    fn entirely_unknown_space_has_no_native_surface_route() {
        let mut p = Planner::new(config(), &[]).unwrap();
        assert!(p.plan([0.05, 0.05, 0.4], [3.05, 0.05, 0.4]).is_err());
    }
    #[test]
    fn missing_ground_strip_does_not_connect_known_platforms() {
        let cells: Vec<_> = flat_cells(0..40,-4..5).into_iter()
            .filter(|c| c.x<15 || c.x>=25).collect();
        let mut p = Planner::from_cells(config(), &cells, &[]).unwrap();
        assert!(p.plan([0.35, 0.05, 0.4], [3.65, 0.05, 0.4]).is_err());
        assert!(p.plan([0.35, 0.05, 0.4], [1.05, 0.05, 0.4]).is_ok());
    }
    #[test]
    fn missing_endpoint_binds_to_an_existing_surface_without_filling_the_map() {
        let cells: Vec<_> = flat_cells(0..40,-4..5).into_iter()
            .filter(|c| [c.x,c.y]!=[0,0]).collect();
        let mut p = Planner::from_cells(config(), &cells, &[]).unwrap();
        let route = p.plan([0.05,0.05,0.4],[3.65,0.05,0.4]).unwrap();
        assert!(route.path.iter().all(|point| {
            let x=(point[0]/0.1).floor() as i32;
            let y=(point[1]/0.1).floor() as i32;
            cells.iter().any(|cell|cell.x==x&&cell.y==y)
        }));
        assert_eq!(p.geometry().0.len(),cells.len());
        assert!(p.plan([0.05,0.05,0.4],[8.0,0.05,0.4]).is_err());
    }
    #[test]
    fn upstream_clearance_cost_prefers_the_interior_without_closing_a_corridor() {
        let cells=flat_cells(0..60,-10..10);
        let mut p=Planner::from_cells(config(),&cells,&[]).unwrap();
        let route=p.plan([0.35,-0.85,0.4],[5.65,-0.85,0.4]).unwrap();
        assert!(route.path.iter().any(|p|p[0]>1.5&&p[0]<4.5&&p[1]>-0.75));
        let mut narrow=Planner::from_cells(config(),&flat_cells(0..60,-2..2),&[]).unwrap();
        assert!(narrow.plan([0.35,0.05,0.4],[5.65,0.05,0.4]).is_ok());
    }
    #[test]
    fn observed_wall_is_avoided_on_measured_ground() {
        let bands: Vec<_> = (-6..=6)
            .map(|y| Band {
                x: 15,
                y,
                low: 0.0,
                high: 1.5,
            })
            .collect();
        let cells = flat_cells(-5..36,-15..16);
        let mut p = Planner::from_cells(config(), &cells, &bands).unwrap();
        assert!(p.plan([1.55, 0.05, 0.4], [1.56, 0.06, 0.4]).is_err());
        let r = p.plan([0.05, 0.05, 0.4], [3.05, 0.05, 0.4]).unwrap();
        assert!(r.path.iter().any(|p| p[1].abs() > 0.7));
        assert!(r
            .path
            .iter()
            .all(|p| !(p[0] > 1.35 && p[0] < 1.75 && p[1].abs() < 0.7)));
        let mut cfg = config();
        cfg.footprint_radius = 0.0;
        let route = Planner::from_cells(cfg, &cells, &bands)
            .unwrap()
            .plan([0.05, 0.05, 0.4], [3.05, 0.05, 0.4])
            .unwrap();
        assert!(route.path.iter().any(|p| p[1] < -0.6 || p[1] > 0.7));
    }
    #[test]
    fn noisy_measurements_do_not_become_voxel_tops() {
        let mut points = Vec::new();
        for x in 0..30 {
            for y in -2..=2 {
                for z in [-0.009, 0.002, 0.007] {
                    points.push([(f64::from(x) + 0.5) * 0.1, (f64::from(y) + 0.5) * 0.1, z]);
                }
            }
        }
        let mut p = Planner::new(config(), &points).unwrap();
        let r = p.plan([0.05, 0.05, 0.4], [2.85, 0.05, 0.4]).unwrap();
        assert!(r.path.iter().all(|p| (p[2] - 0.402).abs() < 1e-6));
    }
    #[test]
    fn failed_search_reuses_the_same_upstream_object() {
        assert!(unsafe { pct_retry_regression() });
    }
    #[test]
    fn stairs_connect_measured_layers() {
        let cells: Vec<_> = (0..40)
            .flat_map(|x| {
                (-4..=4).map(move |y| Cell {
                    x,
                    y,
                    ground: f64::from(x / 5) * 0.12,
                    ceiling: f64::INFINITY,
                })
            })
            .collect();
        let mut cfg = config();
        cfg.boundary = vec![[0.0, -0.4], [4.0, -0.4], [4.0, 0.5], [0.0, 0.5]];
        let bands: Vec<_> = cells
            .iter()
            .map(|s| Band {
                x: s.x,
                y: s.y,
                low: -0.05,
                high: s.ground,
            })
            .collect();
        let mut p = Planner::from_cells(cfg, &cells, &bands).unwrap();
        let r = p.plan([0.25, 0.05, 0.4], [3.75, 0.05, 1.24]).unwrap();
        assert!(r.path.iter().any(|p| p[2] > 1.0));
        assert!(r
            .path
            .windows(2)
            .all(|p| (p[1][2] - p[0][2]).abs() <= 0.160001));
    }
    #[test]
    fn true_footprint_passes_narrow_corridor() {
        let mut cfg = config();
        cfg.footprint = vec![[-0.4, -0.12], [0.4, -0.12], [0.4, 0.12], [-0.4, 0.12]];
        cfg.footprint_radius = 0.42;
        let bands: Vec<_> = (0..40)
            .flat_map(|x| {
                [-3, 2].map(move |y| Band {
                    x,
                    y,
                    low: 0.0,
                    high: 2.0,
                })
            })
            .collect();
        let mut p = Planner::from_cells(cfg, &flat_cells(0..40,-2..2), &bands).unwrap();
        let r = p.plan([0.55, 0.05, 0.4], [3.45, 0.05, 0.4]).unwrap();
        assert!(r.path.iter().all(|p| p[1].abs() < 0.1));
    }
    #[test]
    fn cancellation_reaches_native_search() {
        let mut p = Planner::from_cells(config(), &flat_cells(-5..56,-5..6), &[]).unwrap();
        let cancel = AtomicBool::new(true);
        assert!(p
            .plan_cancellable([0.0, 0.0, 0.4], [5.0, 0.0, 0.4], &cancel)
            .unwrap_err()
            .contains("取消"));
        cancel.store(false, Ordering::Relaxed);
        assert!(p
            .plan_cancellable([0.0, 0.0, 0.4], [5.0, 0.0, 0.4], &cancel)
            .is_ok());
    }
    #[test]
    fn descending_steps_use_the_separate_down_limit() {
        let mut cfg = config();
        cfg.max_step = 0.05;
        cfg.max_step_down = 0.25;
        cfg.boundary = vec![[0.0, -0.4], [3.0, -0.4], [3.0, 0.5], [0.0, 0.5]];
        let cells: Vec<_> = (0..30)
            .flat_map(|x| {
                (-4..=4).map(move |y| Cell {
                    x,
                    y,
                    ground: if x < 15 { 0.2 } else { 0.0 },
                    ceiling: f64::INFINITY,
                })
            })
            .collect();
        let bands: Vec<_> = cells
            .iter()
            .map(|c| Band {
                x: c.x,
                y: c.y,
                low: -0.05,
                high: c.ground,
            })
            .collect();
        let mut p = Planner::from_cells(cfg, &cells, &bands).unwrap();
        assert!(p.plan([0.35, 0.05, 0.6], [2.65, 0.05, 0.4]).is_ok());
        assert!(p.plan([2.65, 0.05, 0.4], [0.35, 0.05, 0.6]).is_err());
    }
    #[test]
    fn distinct_floors_do_not_teleport_between_layers() {
        let mut cfg = config();
        cfg.boundary = vec![[0.0, -0.4], [3.0, -0.4], [3.0, 0.5], [0.0, 0.5]];
        let cells: Vec<_> = (0..30)
            .flat_map(|x| {
                (-4..=4).flat_map(move |y| {
                    [
                        Cell {
                            x,
                            y,
                            ground: 0.0,
                            ceiling: 0.95,
                        },
                        Cell {
                            x,
                            y,
                            ground: 1.0,
                            ceiling: f64::INFINITY,
                        },
                    ]
                })
            })
            .collect();
        let bands: Vec<_> = (0..30)
            .flat_map(|x| {
                (-4..=4).map(move |y| Band {
                    x,
                    y,
                    low: 0.95,
                    high: 1.0,
                })
            })
            .collect();
        let mut p = Planner::from_cells(cfg, &cells, &bands).unwrap();
        assert!(p.plan([0.35, 0.05, 0.4], [2.65, 0.05, 1.4]).is_err());
        assert!(p.plan([0.35, 0.05, 1.4], [2.65, 0.05, 1.4]).is_ok());
    }
    #[test]
    fn shared_raw_classification_has_no_wall_interior_floors() {
        let points: Vec<_> = (0..=40)
            .map(|z| [1.05, 0.05, f64::from(z) * 0.025])
            .collect();
        let (cells, bands) = classify_points(&config(), &points).unwrap();
        assert_eq!(cells.len(), 1);
        assert_eq!(bands.len(), 1);
        assert!(cells[0].ground > 0.98);
        assert_eq!(bands[0].low, 0.0);
        assert_eq!(bands[0].high, 1.0);
    }
    #[test]
    fn measured_ground_noise_does_not_block_the_calibrated_foot() {
        let mut cfg = config();
        cfg.max_step = 0.0;
        cfg.body_z_min = -cfg.body_to_ground;
        let points: Vec<_> = (0..30)
            .flat_map(|x| {
                (-3..=3).flat_map(move |y| {
                    [-0.008, 0.002, 0.007]
                        .map(move |z| [(f64::from(x) + 0.5) * 0.1, (f64::from(y) + 0.5) * 0.1, z])
                })
            })
            .collect();
        let mut p = Planner::new(cfg, &points).unwrap();
        assert!(p.plan([0.35, 0.05, 0.402], [2.65, 0.05, 0.402]).is_ok());
    }
    #[test]
    fn owner_cancellation_scope_reaches_plan_and_restores() {
        let flag = Arc::new(AtomicBool::new(true));
        let mut p = Planner::from_cells(config(), &flat_cells(-5..16,-5..6), &[]).unwrap();
        with_planning_cancellation(flag, || {
            assert!(planning_cancelled());
            assert!(p
                .plan([0.0, 0.0, 0.4], [1.0, 0.0, 0.4])
                .unwrap_err()
                .contains("取消"));
            let explicit = AtomicBool::new(false);
            assert!(p
                .plan_cancellable([0.0, 0.0, 0.4], [1.0, 0.0, 0.4], &explicit)
                .is_ok());
        });
        assert!(!planning_cancelled());
        assert!(p.plan([0.0, 0.0, 0.4], [1.0, 0.0, 0.4]).is_ok());
    }
    #[test]
    fn bounded_map_search_expands_for_a_long_observed_wall() {
        let mut cfg = config();
        cfg.search_margin = 0.5;
        cfg.boundary = vec![[-0.5, -3.0], [3.5, -3.0], [3.5, 3.0], [-0.5, 3.0]];
        let bands: Vec<_> = (-20..=20)
            .map(|y| Band {
                x: 15,
                y,
                low: 0.0,
                high: 2.0,
            })
            .collect();
        let mut p = Planner::from_cells(cfg, &flat_cells(-5..36,-30..31), &bands).unwrap();
        let route = p.plan([0.05, 0.05, 0.4], [3.05, 0.05, 0.4]).unwrap();
        assert!(route.path.iter().any(|point| point[1].abs() > 2.1));
    }
    #[test]
    fn continuous_measured_slope_is_not_misclassified_as_small_steps() {
        for (gradient, allowed) in [(0.2, true), (1.0, false)] {
            let mut cfg = config();
            cfg.max_slope = 25.0_f64.to_radians();
            cfg.boundary = vec![[0.0, -0.2], [2.1, -0.2], [2.1, 0.3], [0.0, 0.3]];
            let cells: Vec<_> = (0..21)
                .flat_map(|x| {
                    (-2..=2).map(move |y| Cell {
                        x,
                        y,
                        ground: (f64::from(x) + 0.5) * 0.1 * gradient,
                        ceiling: f64::INFINITY,
                    })
                })
                .collect();
            let bands: Vec<_> = cells
                .iter()
                .map(|c| Band {
                    x: c.x,
                    y: c.y,
                    low: -0.1,
                    high: c.ground,
                })
                .collect();
            let mut p = Planner::from_cells(cfg, &cells, &bands).unwrap();
            assert_eq!(
                p.plan(
                    [0.25, 0.05, 0.4 + 0.25 * gradient],
                    [1.75, 0.05, 0.4 + 1.75 * gradient]
                )
                .is_ok(),
                allowed
            );
        }
    }
}
