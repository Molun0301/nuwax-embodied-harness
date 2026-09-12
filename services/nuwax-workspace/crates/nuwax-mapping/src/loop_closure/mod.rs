//! 回环检测与GTSAM位姿图优化
//! 关键设计：
//! - 生产优化使用共同GTSAM本机接口，支持目标架构源码构建
//! - 完整三维位姿图与物理噪声模型，旧数学求解器只保留诊断接口
//! - 回环搜索、历史子图匹配和独立几何覆盖验证

pub mod coordinator;
pub mod eval;
pub mod optimizer;
pub mod pgo;
pub mod pgo_auto;
mod registration;
mod scan_context;
pub mod se3;
pub mod utils;

pub use coordinator::{
    CorrectionKeyframe, CorrectionTimeline, LoopClosureCoordinator, LoopClosurePipelineConfig,
    LoopClosurePipelineError, LoopClosureRecord, ProcessFrameResult,
};
pub use eval::EvalLoopClosure;
pub use optimizer::{GaussNewtonOptimizer, LevenbergMarquardt, OptimizerConfig};
pub use pgo::{PoseEdge, PoseGraph, PoseNode};
pub use pgo_auto::{AutoLoopDetector, LoopConstraint};

/// 生产位姿图优化门面，使用GTSAM阻尼求解完整SE(3)因子图。
/// 数学诊断用的旧Gauss-Newton接口保留，生产路径不再使用其稠密求解器。
pub struct PgoOptimizer {
    inner: GaussNewtonOptimizer,
    config: OptimizerConfig,
}

impl PgoOptimizer {
    /// 默认配置构造
    #[must_use]
    pub fn new() -> Self {
        Self {
            inner: GaussNewtonOptimizer::new(OptimizerConfig::default()),
            config: OptimizerConfig::default(),
        }
    }

    /// 自定义配置构造
    #[must_use]
    pub fn with_config(config: OptimizerConfig) -> Self {
        Self {
            inner: GaussNewtonOptimizer::new(config),
            config,
        }
    }

    /// 优化位姿图，返回最终代价（caller 用于判定收敛 / fallback）
    #[must_use]
    pub fn optimize(&self, graph: &mut PoseGraph) -> f64 {
        if !self.config.is_valid() {
            return f64::NAN;
        }
        if graph.nodes.is_empty() || graph.edges.is_empty() {
            return 0.0;
        }
        let mut ids: Vec<_> = graph.nodes.keys().copied().collect();
        ids.sort_unstable();
        let index: std::collections::HashMap<_, _> = ids
            .iter()
            .enumerate()
            .map(|(i, id)| (*id, i as u64))
            .collect();
        let to_wire = |pose: &nalgebra::Isometry3<f64>| {
            let q = pose.rotation.quaternion();
            nuwax_gtsam::GraphPose {
                translation: pose.translation.vector.into(),
                rotation_xyzw: [q.i, q.j, q.k, q.w],
            }
        };
        let poses: Vec<_> = ids
            .iter()
            .map(|id| to_wire(&graph.nodes[id].pose))
            .collect();
        let fixed: Vec<_> = ids.iter().map(|id| graph.nodes[id].fixed).collect();
        let mut edges = Vec::with_capacity(graph.edges.len());
        for edge in &graph.edges {
            let (Some(&from), Some(&to)) = (index.get(&edge.from), index.get(&edge.to)) else {
                return f64::NAN;
            };
            let information = std::array::from_fn(|i| edge.information[(i / 6, i % 6)]);
            edges.push(nuwax_gtsam::GraphEdge {
                from,
                to,
                measurement: to_wire(&edge.measurement),
                information,
            });
        }
        let result = match nuwax_gtsam::optimize(
            &poses,
            &fixed,
            &edges,
            self.config.max_iterations,
            self.config.gradient_tolerance,
            self.config.initial_lambda,
            self.config.lambda_scale,
            None,
        ) {
            Ok(result) => result,
            Err(error) => {
                tracing::warn!(%error,"GTSAM位姿图求解未提交结果");
                return f64::NAN;
            }
        };
        if !result.converged {
            return f64::NAN;
        }
        for (id, pose) in ids.into_iter().zip(result.poses) {
            let node = graph.nodes.get_mut(&id).expect("节点来自同一图");
            if node.fixed {
                continue;
            }
            let q = pose.rotation_xyzw;
            node.pose = nalgebra::Isometry3::from_parts(
                nalgebra::Translation3::from(nalgebra::Vector3::from(pose.translation)),
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    q[3], q[0], q[1], q[2],
                )),
            );
        }
        result.final_cost
    }

    /// 内部 GN 求解器（caller 想直接调 optimize 时用）
    #[must_use]
    pub fn gauss_newton(&self) -> &GaussNewtonOptimizer {
        &self.inner
    }
}

impl Default for PgoOptimizer {
    fn default() -> Self {
        Self::new()
    }
}
