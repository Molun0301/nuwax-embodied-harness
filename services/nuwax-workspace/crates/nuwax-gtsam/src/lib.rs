//! GTSAM因子图求解的受控本机接口；跨语言部分只处理有界、已校验的数组。
#![deny(unsafe_op_in_unsafe_fn)]

use std::{
    ffi::{c_char, c_void, CStr},
    sync::atomic::{AtomicBool, Ordering},
};

/// 图节点位姿；四元数顺序为XYZW。
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GraphPose {
    pub translation: [f64; 3],
    pub rotation_xyzw: [f64; 4],
}

/// 相对位姿因子，信息矩阵按平移在前、旋转在后的行主序输入。
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct GraphEdge {
    pub from: u64,
    pub to: u64,
    pub measurement: GraphPose,
    pub information: [f64; 36],
}

/// 机体坐标中的向上方向与角度噪声；零噪声哨兵表示该节点没有方向先验。
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GravityPrior {
    pub direction_body: [f64; 3],
    pub sigma: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
struct RawResult {
    initial_cost: f64,
    final_cost: f64,
    iterations: u32,
    converged: u8,
}

/// 一次完整求解结果；失败与取消不返回部分位姿。
#[derive(Debug)]
pub struct Optimization {
    pub poses: Vec<GraphPose>,
    pub initial_cost: f64,
    pub final_cost: f64,
    pub iterations: u32,
    pub converged: bool,
}

unsafe extern "C" {
    fn nuwax_gtsam_optimize_with_gravity(
        initial: *const GraphPose,
        fixed: *const u8,
        node_count: usize,
        edges: *const GraphEdge,
        edge_count: usize,
        gravity: *const GravityPrior,
        max_iterations: u32,
        gradient_tolerance: f64,
        initial_lambda: f64,
        lambda_scale: f64,
        cancelled: Option<extern "C" fn(*mut c_void) -> i32>,
        cancel_context: *mut c_void,
        output: *mut GraphPose,
        result: *mut RawResult,
        error: *mut c_char,
        error_capacity: usize,
    ) -> i32;
}

extern "C" fn poll_cancel(context: *mut c_void) -> i32 {
    if context.is_null() {
        return 1;
    }
    // 指针来自下方同步调用期间借用的AtomicBool，C++不保留该指针或在返回后调用。
    i32::from(unsafe { &*(context as *const AtomicBool) }.load(Ordering::Acquire))
}

/// 使用GTSAM的阻尼因子图优化，固定节点和米/弧度信息模型由调用者明确给出。
pub fn optimize(
    poses: &[GraphPose],
    fixed: &[bool],
    edges: &[GraphEdge],
    max_iterations: usize,
    gradient_tolerance: f64,
    initial_lambda: f64,
    lambda_scale: f64,
    cancelled: Option<&AtomicBool>,
) -> Result<Optimization, String> {
    optimize_with_gravity(
        poses,
        fixed,
        edges,
        None,
        max_iterations,
        gradient_tolerance,
        initial_lambda,
        lambda_scale,
        cancelled,
    )
}

/// 可选重力先验不约束偏航；与旧入口共用边、固定节点、取消及完整提交语义。
pub fn optimize_with_gravity(
    poses: &[GraphPose],
    fixed: &[bool],
    edges: &[GraphEdge],
    gravity: Option<&[GravityPrior]>,
    max_iterations: usize,
    gradient_tolerance: f64,
    initial_lambda: f64,
    lambda_scale: f64,
    cancelled: Option<&AtomicBool>,
) -> Result<Optimization, String> {
    if poses.is_empty()
        || poses.len() > 2000
        || fixed.len() != poses.len()
        || edges.len() > 8000
        || max_iterations == 0
        || max_iterations > 300
        || gravity.is_some_and(|priors| priors.len() != poses.len())
    {
        return Err("invalid graph bounds".into());
    }
    let fixed: Vec<u8> = fixed.iter().copied().map(u8::from).collect();
    let mut output = poses.to_vec();
    let mut result = RawResult::default();
    let mut error = [0 as c_char; 512];
    let (callback, context) = match cancelled {
        Some(flag) => (
            Some(poll_cancel as extern "C" fn(*mut c_void) -> i32),
            flag as *const AtomicBool as *mut c_void,
        ),
        None => (None, std::ptr::null_mut()),
    };
    // 输入/输出数组长度在同一次同步调用中固定，所有指针存活到原生函数返回。
    // 原生端捕获全部异常，成功才写入完整结果，不能跨语言展开栈或保留内存引用。
    let status = unsafe {
        nuwax_gtsam_optimize_with_gravity(
            poses.as_ptr(),
            fixed.as_ptr(),
            poses.len(),
            edges.as_ptr(),
            edges.len(),
            gravity.map_or(std::ptr::null(), |priors| priors.as_ptr()),
            max_iterations as u32,
            gradient_tolerance,
            initial_lambda,
            lambda_scale,
            callback,
            context,
            output.as_mut_ptr(),
            &mut result,
            error.as_mut_ptr(),
            error.len(),
        )
    };
    match status {
        0 => Ok(Optimization {
            poses: output,
            initial_cost: result.initial_cost,
            final_cost: result.final_cost,
            iterations: result.iterations,
            converged: result.converged != 0,
        }),
        2 => Err("optimization_cancelled".into()),
        _ => {
            // 缓冲初始化为零，原生端在写入错误说明时也保证末尾零字节。
            Err(unsafe { CStr::from_ptr(error.as_ptr()) }
                .to_string_lossy()
                .into_owned())
        }
    }
}

/// 点到平面匹配的结果，变换将源点映射到目标坐标。
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Registration {
    pub transformation: GraphPose,
    pub mse: f64,
    pub iterations: u32,
    pub converged: u8,
}
unsafe extern "C" {
    fn nuwax_pcl_point_to_plane(
        source: *const f64,
        source_count: usize,
        target: *const f64,
        target_count: usize,
        max_distance: f64,
        iterations: u32,
        result: *mut Registration,
    ) -> i32;
}

/// 调用成熟点云库的点到平面匹配，保留源/目标方向。
pub fn point_to_plane(
    source: &[[f64; 3]],
    target: &[[f64; 3]],
    max_distance: f64,
    iterations: usize,
) -> Result<Registration, String> {
    let mut result = Registration {
        transformation: GraphPose {
            translation: [0.0; 3],
            rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
        },
        mse: f64::INFINITY,
        iterations: 0,
        converged: 0,
    };
    if iterations > 300 {
        return Err("registration iteration limit".into());
    }
    // 固定长度数组是连续的三坐标布局，原生端不保留输入指针；异常在C++内终止。
    let status = unsafe {
        nuwax_pcl_point_to_plane(
            source.as_ptr().cast(),
            source.len(),
            target.as_ptr().cast(),
            target.len(),
            max_distance,
            iterations as u32,
            &mut result,
        )
    };
    if status == 0 {
        Ok(result)
    } else {
        Err("point-to-plane registration failed".into())
    }
}
