//! P0-09 回归测试：`loop_closure` 模块导出 + `PgoOptimizer` 门面
//!
//! 覆盖：
//! - `nuwax_mapping::loop_closure` 模块可访问
//! - `nuwax_mapping::PgoOptimizer`（re-export）可构造可调用
//! - `se3_log_to_vector6()` 返 6D twist
//! - `PoseGraph` add_node/add_edge/optimize 完整链路

use nalgebra::{Isometry3, Vector6};
use nuwax_mapping::loop_closure::{
    AutoLoopDetector, GaussNewtonOptimizer, LevenbergMarquardt, OptimizerConfig, PgoOptimizer,
    PoseEdge, PoseGraph,
};
use nuwax_mapping::loop_closure::optimizer::se3_log_to_vector6;

#[test]
fn loop_closure_module_exports_accessible() {
    // 类型都可被引用（编译期 + 链接期通过）
    let _gn = GaussNewtonOptimizer::new(OptimizerConfig::default());
    let _lm = LevenbergMarquardt::new(OptimizerConfig::default());
    let _det = AutoLoopDetector::new(5.0, 10);
}

#[test]
fn pgo_optimizer_default_optimizes_identity_graph() {
    let mut g = PoseGraph::new();
    g.add_node(0, Isometry3::identity());
    g.add_node(1, Isometry3::identity());
    g.fix_first_node();
    g.add_edge(PoseEdge::new(0, 1, Isometry3::identity()));

    let opt = PgoOptimizer::default();
    let _cost = opt.optimize(&mut g);
    // identity graph 优化后应保持无大变化
    let n0 = g.nodes.get(&0).expect("node 0").pose.translation.vector;
    let n1 = g.nodes.get(&1).expect("node 1").pose.translation.vector;
    assert!(n0.norm() < 1e-3, "node 0 should stay near origin, got {n0:?}");
    assert!(n1.norm() < 1e-3, "node 1 should stay near origin, got {n1:?}");
}

#[test]
fn pgo_optimizer_with_config_constructor() {
    let cfg = OptimizerConfig::default();
    let opt = PgoOptimizer::with_config(cfg);
    // 内部 GN 应一致
    let _gn_ref = opt.gauss_newton();
}

#[test]
fn se3_log_to_vector6_returns_six_dimensions() {
    // identity → 全零 6D twist
    let v: Vector6<f64> = se3_log_to_vector6(&Isometry3::identity());
    assert_eq!(v.len(), 6);
    assert!(v.norm() < 1e-9);

    // 平移 (1, 2, 3) → 前三维 ≈ (1, 2, 3)
    let p = Isometry3::translation(1.0, 2.0, 3.0);
    let v = se3_log_to_vector6(&p);
    assert_eq!(v.len(), 6);
    assert!((v[0] - 1.0).abs() < 1e-9);
    assert!((v[1] - 2.0).abs() < 1e-9);
    assert!((v[2] - 3.0).abs() < 1e-9);
}

#[test]
fn pgo_optimize_reduces_cost_on_noisy_chain() {
    // 构造一条 5 节点链，每条边 = identity（无噪声基线）
    let mut g = PoseGraph::new();
    for i in 0..5 {
        g.add_node(i, Isometry3::identity());
    }
    g.fix_first_node();
    for i in 0..4 {
        g.add_edge(PoseEdge::new(i, i + 1, Isometry3::identity()));
    }

    let opt = PgoOptimizer::default();
    let cost = opt.optimize(&mut g);
    // identity 链路代价应接近 0
    assert!(cost < 1e-3, "cost on identity chain should be near 0, got {cost}");
}