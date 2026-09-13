//! 无中间件二维算法的原始C接口；安全所有权由nuwax-slam-toolbox提供。
#![deny(unsafe_op_in_unsafe_fn)]
use std::ffi::{c_char, c_void};
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub abi_version: u32,
    pub scan_count: u32,
    pub local_window: u32,
    pub max_nodes: u32,
    pub angle_min: f64,
    pub angle_increment: f64,
    pub range_min: f64,
    pub range_max: f64,
    pub range_threshold: f64,
    pub base_from_laser: Pose,
    pub resolution: f64,
    pub minimum_travel: f64,
    pub minimum_heading: f64,
    pub minimum_response: f64,
    pub search_dimension: f64,
    pub search_resolution: f64,
    pub search_smear: f64,
    pub max_scan_gap_s: f64,
    pub loop_closing: u32,
    pub solver_threads: u32,
    pub use_scan_barycenter: u32,
    pub loop_minimum_chain: u32,
    pub link_maximum_distance: f64,
    pub loop_search_distance: f64,
}
#[repr(C)]
pub struct Scan {
    pub timestamp_ns: u64,
    pub sequence: u64,
    pub odom_from_base: Pose,
    pub ranges: *const f64,
    pub count: usize,
    pub use_laser_odometry: u32,
}
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct Output {
    pub timestamp_ns: u64,
    pub sequence: u64,
    pub graph_revision: u64,
    pub optimizations: u64,
    pub odom_from_base: Pose,
    pub map_from_base: Pose,
    pub map_from_odom: Pose,
    pub covariance: [f64; 9],
    pub matching_response: f64,
    pub local_response: f64,
    pub disposition: u32,
    pub node_count: u32,
    pub edge_count: u32,
    pub valid_beams: u32,
}
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct Grid {
    pub graph_revision: u64,
    pub width: u32,
    pub height: u32,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
}
unsafe extern "C" {
    pub fn nw_slam_abi_version() -> u32;
    pub fn nw_slam_validate_config(
        config: *const Config,
        error: *mut c_char,
        capacity: usize,
    ) -> i32;
    pub fn nw_slam_create(
        config: *const Config,
        out: *mut *mut c_void,
        error: *mut c_char,
        capacity: usize,
    ) -> i32;
    pub fn nw_slam_destroy(ctx: *mut c_void);
    pub fn nw_slam_process(
        ctx: *mut c_void,
        scan: *const Scan,
        out: *mut Output,
        error: *mut c_char,
        capacity: usize,
    ) -> i32;
    pub fn nw_slam_grid(
        ctx: *mut c_void,
        out: *mut Grid,
        error: *mut c_char,
        capacity: usize,
    ) -> i32;
    pub fn nw_slam_copy_grid(
        ctx: *mut c_void,
        out: *mut i8,
        count: usize,
        error: *mut c_char,
        capacity: usize,
    ) -> i32;
    pub fn nw_slam_resume(ctx: *mut c_void, error: *mut c_char, capacity: usize) -> i32;
    pub fn nw_slam_save(
        ctx: *mut c_void,
        prefix: *const c_char,
        error: *mut c_char,
        capacity: usize,
    ) -> i32;
    pub fn nw_slam_load(
        ctx: *mut c_void,
        prefix: *const c_char,
        localization: u32,
        seed: Pose,
        error: *mut c_char,
        capacity: usize,
    ) -> i32;
}
