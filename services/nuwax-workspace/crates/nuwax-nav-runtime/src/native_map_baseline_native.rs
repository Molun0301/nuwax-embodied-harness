//! 显式加载基线共享库，仅用于跨版本等价和耗时测试。
// 此测试模块独立声明基线C接口；生产运行时仍禁止不安全代码。
#![allow(unsafe_code)]
use nuwax_nav_pct_native::{Band, Cell, Config, Route, RouteKind};
use std::ffi::{c_char, c_void, CStr, CString};

#[repr(C)]
struct RawConfig {
    values: [f64; 11],
    footprint: *const f64,
    footprint_len: usize,
    boundary: *const f64,
    boundary_len: usize,
}
#[repr(C)]
#[derive(Default)]
struct RawResult {
    points: *mut f64,
    length: usize,
    elapsed_ms: f64,
    expanded: usize,
    kind: i32,
}
type Create = unsafe extern "C" fn(
    *const RawConfig,
    *const Cell,
    usize,
    *const Band,
    usize,
    *const f64,
    usize,
    *mut c_char,
    usize,
) -> *mut c_void;
type Plan = unsafe extern "C" fn(
    *mut c_void,
    *const f64,
    *const f64,
    *mut RawResult,
    Option<extern "C" fn(*const c_void) -> bool>,
    *const c_void,
    bool,
    *mut c_char,
    usize,
) -> i32;
type Destroy = unsafe extern "C" fn(*mut c_void);
type Free = unsafe extern "C" fn(*mut RawResult);
#[link(name = "dl")]
unsafe extern "C" {
    fn dlopen(path: *const c_char, flags: i32) -> *mut c_void;
    fn dlsym(handle: *mut c_void, name: *const c_char) -> *mut c_void;
    fn dlclose(handle: *mut c_void) -> i32;
}

pub(super) struct Baseline {
    handle: *mut c_void,
    create: Create,
    plan: Plan,
    destroy: Destroy,
    free: Free,
}
impl Baseline {
    pub fn open() -> Self {
        let path =
            CString::new(std::env::var("N1_PCT_BASELINE_SO").expect("指定基线共享库路径")).unwrap();
        unsafe {
            // 单独绑定基线自身的C++符号，避免与当前原生桥的同名实现相互覆盖。
            let handle = dlopen(path.as_ptr(), 2 | 8);
            assert!(!handle.is_null(), "基线共享库加载失败");
            Self {
                handle,
                create: std::mem::transmute::<*mut c_void, Create>(dlsym(
                    handle,
                    c"pct_create".as_ptr(),
                )),
                plan: std::mem::transmute::<*mut c_void, Plan>(dlsym(handle, c"pct_plan".as_ptr())),
                destroy: std::mem::transmute::<*mut c_void, Destroy>(dlsym(
                    handle,
                    c"pct_destroy".as_ptr(),
                )),
                free: std::mem::transmute::<*mut c_void, Free>(dlsym(
                    handle,
                    c"pct_result_free".as_ptr(),
                )),
            }
        }
    }

    pub fn plan(
        &self,
        cfg: &Config,
        cells: &[Cell],
        bands: &[Band],
        start: [f64; 3],
        goal: [f64; 3],
        progressive: bool,
    ) -> Result<Route, String> {
        let config = RawConfig {
            values: [
                cfg.resolution,
                cfg.point_resolution,
                cfg.robot_height,
                cfg.body_to_ground,
                cfg.max_step,
                cfg.max_step_down,
                cfg.max_slope,
                cfg.footprint_radius,
                cfg.body_z_min,
                cfg.body_z_max,
                cfg.search_margin,
            ],
            footprint: cfg.footprint.as_ptr().cast(),
            footprint_len: cfg.footprint.len(),
            boundary: cfg.boundary.as_ptr().cast(),
            boundary_len: cfg.boundary.len(),
        };
        let mut error = [0; 512];
        let mut result = RawResult::default();
        unsafe {
            let planner = (self.create)(
                &config,
                cells.as_ptr(),
                cells.len(),
                bands.as_ptr(),
                bands.len(),
                std::ptr::null(),
                0,
                error.as_mut_ptr(),
                error.len(),
            );
            assert!(!planner.is_null());
            let status = (self.plan)(
                planner,
                start.as_ptr(),
                goal.as_ptr(),
                &mut result,
                None,
                std::ptr::null(),
                progressive,
                error.as_mut_ptr(),
                error.len(),
            );
            let route = if status == 0 {
                let path = std::slice::from_raw_parts(result.points, result.length * 3)
                    .chunks_exact(3)
                    .map(|p| [p[0], p[1], p[2]])
                    .collect();
                Ok(Route {
                    path,
                    elapsed_ms: result.elapsed_ms,
                    expanded: result.expanded,
                    kind: if result.kind == 0 {
                        RouteKind::Complete
                    } else {
                        RouteKind::Approach
                    },
                })
            } else {
                Err(CStr::from_ptr(error.as_ptr())
                    .to_string_lossy()
                    .into_owned())
            };
            (self.free)(&mut result);
            (self.destroy)(planner);
            route
        }
    }
}
impl Drop for Baseline {
    fn drop(&mut self) {
        unsafe {
            dlclose(self.handle);
        }
    }
}
