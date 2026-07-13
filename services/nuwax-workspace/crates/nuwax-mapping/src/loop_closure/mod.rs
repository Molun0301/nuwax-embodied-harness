//! 回环检测 + 自研 PGO
//! 关键设计：
//! - PGO 自研 Gauss-Newton 求解器（**不引入 gtsam-rs**：aarch64 无 prebuilt）
//! - 位姿图数据结构 + 二元边模型 + Levenberg-Marquardt 回退
//! - 自动回环检测基于 kiddo KD-tree 距离阈值

pub mod eval;
pub mod optimizer;
pub mod pgo;
pub mod pgo_auto;
pub mod se3;
pub mod utils;

pub use eval::EvalLoopClosure;
pub use optimizer::{GaussNewtonOptimizer, LevenbergMarquardt, OptimizerConfig};
pub use pgo::{PoseEdge, PoseGraph, PoseNode};
pub use pgo_auto::AutoLoopDetector;

/// PGO 优化器门面（rework22 P0-09.3）
///
/// 对外暴露统一名字 `PgoOptimizer`，内部默认是 `GaussNewtonOptimizer`；
/// 提供 `optimize(&mut PoseGraph)` API，把代价归一化后回 caller。
///
/// 选择 GN 作为默认而非 LM 的原因：
/// - GN 在残差较小时收敛更快（典型场景：回环边初次加入，残差已经较小）
/// - LM 在残差较大时更稳健；当 GN 收敛失败时 caller 可 fallback 到 LM。
pub struct PgoOptimizer {
    inner: GaussNewtonOptimizer,
}

impl PgoOptimizer {
    /// 默认配置构造
    #[must_use]
    pub fn default() -> Self {
        Self {
            inner: GaussNewtonOptimizer::new(OptimizerConfig::default()),
        }
    }

    /// 自定义配置构造
    #[must_use]
    pub fn with_config(config: OptimizerConfig) -> Self {
        Self {
            inner: GaussNewtonOptimizer::new(config),
        }
    }

    /// 优化位姿图，返回最终代价（caller 用于判定收敛 / fallback）
    #[must_use]
    pub fn optimize(&self, graph: &mut PoseGraph) -> f64 {
        self.inner.optimize(graph)
    }

    /// 内部 GN 求解器（caller 想直接调 optimize 时用）
    #[must_use]
    pub fn gauss_newton(&self) -> &GaussNewtonOptimizer {
        &self.inner
    }
}

impl Default for PgoOptimizer {
    fn default() -> Self {
        Self::default()
    }
}
