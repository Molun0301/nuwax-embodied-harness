//! 回环检测 + 自研 PGO
//! 关键设计：
//! - PGO 自研 Gauss-Newton 求解器（**不引入 gtsam-rs**：aarch64 无 prebuilt）
//! - 位姿图数据结构 + 二元边模型 + Levenberg-Marquardt 回退
//! - 自动回环检测基于 kiddo KD-tree 距离阈值

pub mod eval;
pub mod optimizer;
pub mod pgo;
pub mod pgo_auto;
pub mod utils;

pub use eval::EvalLoopClosure;
pub use optimizer::{GaussNewtonOptimizer, LevenbergMarquardt, OptimizerConfig};
pub use pgo::{PoseEdge, PoseGraph, PoseNode};
pub use pgo_auto::AutoLoopDetector;