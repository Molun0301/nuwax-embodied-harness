//! DUFOMap（KTH-RPL，BSD-3）去动态判定的安全封装，底层为 UFOMap 八叉树。
//!
//! 判定规则与上游一致：每帧带传感器原点插入后，射线经过的体素记为“曾被观测为空”；
//! 落在这类体素内的点判为动态，其余为静态。
//! - 离线模式：插入全部帧后调用 [`DufoMap::propagate`]，再判定累计点。
//! - 在线模式（论文中的 DUFOMap*）：每插入一帧，就用当时已建的地图判定该帧和已累计的点。
//!
//! 上游来源与本地修改见 crate 根目录的 NOTICE。
use std::ffi::{c_char, c_void, CStr};
use std::ptr::NonNull;

#[repr(C)]
struct RawParams {
    resolution: f64,
    levels: u32,
    min_range: f32,
    max_range: f32,
    inflate_unknown: u32,
    inflate_unknown_compensation: u8,
    inflate_hits_dist: f32,
    ray_passthrough_hits: u8,
    down_sampling: u8,
    simple_ray_casting: u8,
    simple_ray_casting_factor: f32,
    hit_depth: u32,
    miss_depth: u32,
    only_valid: u8,
    sliding_window_size: u32,
}

extern "C" {
    fn nuwax_dufomap_new(params: *const RawParams, err: *mut c_char, err_len: usize) -> *mut c_void;
    fn nuwax_dufomap_free(handle: *mut c_void);
    fn nuwax_dufomap_insert(handle: *mut c_void, xyz: *const f32, n: usize, origin: *const f32,
        propagate: u8, err: *mut c_char, err_len: usize) -> i32;
    fn nuwax_dufomap_propagate(handle: *mut c_void);
    fn nuwax_dufomap_seen_free(handle: *const c_void, xyz: *const f32, n: usize, out: *mut u8) -> i32;
    fn nuwax_dufomap_memory_bytes(handle: *const c_void) -> usize;
    fn nuwax_dufomap_window_memory_bytes(handle: *const c_void) -> usize;
    fn nuwax_dufomap_num_nodes(handle: *const c_void) -> usize;
    fn nuwax_dufomap_clear(handle: *mut c_void);
    fn nuwax_dufomap_clone(handle: *const c_void) -> *mut c_void;
    fn nuwax_dufomap_track_free(handle: *mut c_void);
    fn nuwax_dufomap_free_cells(handle: *mut c_void, out: *mut FreeCell, capacity: usize) -> usize;
}

/// 首次被观测为空的八叉树单元；key 为以分辨率为单位的有符号下界，边长为 2^depth。
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FreeCell {
    pub key: [i32; 3],
    pub depth: u32,
}

/// 插入前的降采样方式，与上游 `down_sampling_method` 相同。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DownSampling {
    None = 0,
    Center = 1,
    Centroid = 2,
    Uniform = 3,
}

/// 对应上游 DUFOMap 配置与积分参数；[`Default`] 取上游默认值。
#[derive(Clone, Debug, PartialEq)]
pub struct Params {
    /// 体素边长（米），论文中的 v。
    pub resolution: f64,
    /// 八叉树层数，决定可表示的空间范围。
    pub levels: u32,
    /// 插入前剔除离传感器过近的点（米），用于去掉机体自身回波。
    pub min_range: f32,
    /// 最大积分距离（米），负数表示不限。
    pub max_range: f32,
    /// 论文中的 d_p：把未知区域外扩的体素数，用来补偿位姿与离散化误差。
    pub inflate_unknown: u32,
    pub inflate_unknown_compensation: bool,
    /// 论文中的 d_s：命中点周围不判为空的保护距离（米）。
    pub inflate_hits_dist: f32,
    pub ray_passthrough_hits: bool,
    pub down_sampling: DownSampling,
    pub simple_ray_casting: bool,
    pub simple_ray_casting_factor: f32,
    pub hit_depth: u32,
    pub miss_depth: u32,
    pub only_valid: bool,
    /// 最近几次插入的射线网格合并后再判空；0（上游默认）与1均只使用当前帧。
    /// 每次插入保留自己的传感器原点，窗口中的命中共同保护静态点。
    pub sliding_window_size: u32,
}

impl Default for Params {
    fn default() -> Self {
        Self {
            resolution: 0.1,
            levels: 20,
            min_range: 0.2,
            max_range: -1.0,
            inflate_unknown: 1,
            inflate_unknown_compensation: true,
            inflate_hits_dist: 0.2,
            ray_passthrough_hits: false,
            down_sampling: DownSampling::Center,
            simple_ray_casting: false,
            simple_ray_casting_factor: 1.0,
            hit_depth: 0,
            miss_depth: 0,
            only_valid: false,
            sliding_window_size: 0,
        }
    }
}

impl Params {
    fn raw(&self) -> RawParams {
        RawParams {
            resolution: self.resolution,
            levels: self.levels,
            min_range: self.min_range,
            max_range: self.max_range,
            inflate_unknown: self.inflate_unknown,
            inflate_unknown_compensation: self.inflate_unknown_compensation as u8,
            inflate_hits_dist: self.inflate_hits_dist,
            ray_passthrough_hits: self.ray_passthrough_hits as u8,
            down_sampling: self.down_sampling as u8,
            simple_ray_casting: self.simple_ray_casting as u8,
            simple_ray_casting_factor: self.simple_ray_casting_factor,
            hit_depth: self.hit_depth,
            miss_depth: self.miss_depth,
            only_valid: self.only_valid as u8,
            sliding_window_size: self.sliding_window_size,
        }
    }
}

const ERROR_LEN: usize = 512;

fn error_text(buffer: &[c_char; ERROR_LEN], fallback: &str) -> String {
    let text = unsafe { CStr::from_ptr(buffer.as_ptr()) }.to_string_lossy().into_owned();
    if text.is_empty() { fallback.into() } else { text }
}

/// 一张 DUFOMap 地图。可在线程间移交，不可并发访问。
#[derive(Debug)]
pub struct DufoMap {
    handle: NonNull<c_void>,
}

// 原生地图只由持有者独占访问。
unsafe impl Send for DufoMap {}

impl DufoMap {
    /// 在线单次积分并交付新增空单元；不会复制整棵树，启用窗口时仅保留有限帧的射线网格。
    pub fn insert_with_free_cells(&mut self, points: &[[f32; 3]], origin: [f32; 3]) -> Result<Vec<FreeCell>, String> {
        unsafe { nuwax_dufomap_track_free(self.handle.as_ptr()) };
        let result = self.insert(points, origin, true);
        let count = unsafe { nuwax_dufomap_free_cells(self.handle.as_ptr(), std::ptr::null_mut(), 0) };
        let mut cells = vec![FreeCell::default(); count];
        unsafe { nuwax_dufomap_free_cells(self.handle.as_ptr(), cells.as_mut_ptr(), cells.len()) };
        result.map(|_| cells)
    }

    pub fn new(params: &Params) -> Result<Self, String> {
        let raw = params.raw();
        let mut err = [0 as c_char; ERROR_LEN];
        let handle = unsafe { nuwax_dufomap_new(&raw, err.as_mut_ptr(), ERROR_LEN) };
        NonNull::new(handle).map(|handle| Self { handle }).ok_or_else(|| error_text(&err, "创建DUFOMap失败"))
    }

    /// 插入一帧。`points` 与 `origin` 同为全局坐标系；`propagate` 为真时立即更新内部节点。
    pub fn insert(&mut self, points: &[[f32; 3]], origin: [f32; 3], propagate: bool) -> Result<(), String> {
        if !origin.iter().all(|v| v.is_finite()) {
            return Err("传感器原点不是有限值".into());
        }
        let mut err = [0 as c_char; ERROR_LEN];
        let code = unsafe {
            nuwax_dufomap_insert(self.handle.as_ptr(), points.as_ptr().cast(), points.len(), origin.as_ptr(),
                propagate as u8, err.as_mut_ptr(), ERROR_LEN)
        };
        if code == 0 { Ok(()) } else { Err(error_text(&err, "插入点云失败")) }
    }

    /// 把逐帧插入时未传播的修改更新到内部节点；离线模式在全部插入后调用一次。
    pub fn propagate(&mut self) {
        unsafe { nuwax_dufomap_propagate(self.handle.as_ptr()) }
    }

    /// 逐点判定是否落在“曾被观测为空”的体素内，结果写入 `out`（真为动态）。
    pub fn seen_free_into(&self, points: &[[f32; 3]], out: &mut Vec<bool>) -> Result<(), String> {
        let mut flags = vec![0u8; points.len()];
        let code = unsafe {
            nuwax_dufomap_seen_free(self.handle.as_ptr(), points.as_ptr().cast(), points.len(), flags.as_mut_ptr())
        };
        if code != 0 {
            return Err("DUFOMap查询失败".into());
        }
        out.clear();
        out.extend(flags.into_iter().map(|flag| flag != 0));
        Ok(())
    }

    pub fn seen_free(&self, points: &[[f32; 3]]) -> Result<Vec<bool>, String> {
        let mut out = Vec::with_capacity(points.len());
        self.seen_free_into(points, &mut out)?;
        Ok(out)
    }

    /// 八叉树已分配的内存（字节），不包含滑动窗口和积分临时内存。
    pub fn memory_bytes(&self) -> usize {
        unsafe { nuwax_dufomap_memory_bytes(self.handle.as_ptr()) }
    }

    /// 滑动窗口常驻内存的估算值（字节）：容器容量、哈希桶及键/网格有效载荷。
    /// 不包含哈希节点实现开销、分配器元数据和积分时的临时并集；未启用窗口或清空后为0。
    pub fn window_memory_bytes(&self) -> usize {
        unsafe { nuwax_dufomap_window_memory_bytes(self.handle.as_ptr()) }
    }

    pub fn num_nodes(&self) -> usize {
        unsafe { nuwax_dufomap_num_nodes(self.handle.as_ptr()) }
    }

    /// 清空地图及窗口射线与下标，保留参数，供新建图会话复用。
    pub fn clear(&mut self) {
        unsafe { nuwax_dufomap_clear(self.handle.as_ptr()) }
    }
}

impl Clone for DufoMap {
    fn clone(&self) -> Self {
        let handle = unsafe { nuwax_dufomap_clone(self.handle.as_ptr()) };
        Self { handle: NonNull::new(handle).expect("复制 DUFOMap 内存不足") }
    }
}

impl Drop for DufoMap {
    fn drop(&mut self) {
        unsafe { nuwax_dufomap_free(self.handle.as_ptr()) }
    }
}

#[cfg(test)]
mod tests;
