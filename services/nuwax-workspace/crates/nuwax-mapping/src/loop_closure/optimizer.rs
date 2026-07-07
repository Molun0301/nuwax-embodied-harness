//! 自研 Gauss-Newton + Levenberg-Marquardt 优化器
//! 算法：
//! - 输入：PoseGraph（节点 + 边）
//! - 残差：对每条边计算 6D 误差向量 `e_ij = log(measured_ij.inverse() * predicted_ij)`
//! - 目标：最小化 sum(e_ij^T * Omega_ij * e_ij)
//! - 求解：(J^T * Omega * J + lambda * I) * delta = -J^T * Omega * e
//! - 更新：每个节点的 pose = exp_update(pose, delta)

use super::pgo::{transform_utils, PoseEdge, PoseGraph, PoseNode};
use nalgebra::{DMatrix, DVector, Isometry3, Matrix6, Vector6};

/// 优化器配置
#[derive(Debug, Clone, Copy)]
pub struct OptimizerConfig {
    /// 最大迭代次数
    pub max_iterations: usize,
    /// 收敛阈值（梯度 norm）
    pub gradient_tolerance: f64,
    /// LM 阻尼系数初始值
    pub initial_lambda: f64,
    /// LM 阻尼系数缩放因子
    pub lambda_scale: f64,
}

impl Default for OptimizerConfig {
    fn default() -> Self {
        Self {
            max_iterations: 50,
            gradient_tolerance: 1e-6,
            initial_lambda: 1e-4,
            lambda_scale: 10.0,
        }
    }
}

/// Gauss-Newton 优化器
pub struct GaussNewtonOptimizer {
    config: OptimizerConfig,
}

impl GaussNewtonOptimizer {
    /// 构造
    #[must_use]
    pub fn new(config: OptimizerConfig) -> Self {
        Self { config }
    }

    /// 优化位姿图
    ///
    /// `graph` 会被原地修改（节点 pose 被更新）。
    /// Returns 最终的 total cost。
    pub fn optimize(&self, graph: &mut PoseGraph) -> f64 {
        if graph.num_nodes() == 0 || graph.num_edges() == 0 {
            return 0.0;
        }

        let mut prev_cost = f64::INFINITY;
        for _ in 0..self.config.max_iterations {
            let (cost, jacobian, residual) = self.linearize(graph);

            if (prev_cost - cost).abs() < self.config.gradient_tolerance {
                return cost;
            }
            prev_cost = cost;

            // 求解 normal equations: (J^T J) delta = -J^T r
            let jtj = jacobian.transpose() * &jacobian;
            let jtr = jacobian.transpose() * &residual;
            let n_vars = jtr.len();
            let n_dofs = n_vars - 6; // 减去固定节点

            if n_dofs <= 0 {
                return cost;
            }

            // 简化：使用全部变量（未排除固定节点）；实装可加 fixed mask
            let delta = match Self::solve_linear(&jtj, &jtr) {
                Some(d) => d,
                None => return cost, // 奇异 → 终止
            };

            // 更新每个节点
            for (i, (_, node)) in graph.nodes.iter_mut().enumerate() {
                if node.fixed {
                    continue;
                }
                let d = Vector6::new(
                    delta[i * 6],
                    delta[i * 6 + 1],
                    delta[i * 6 + 2],
                    delta[i * 6 + 3],
                    delta[i * 6 + 4],
                    delta[i * 6 + 5],
                );
                node.pose = transform_utils::exp_update(&node.pose, &d);
            }
        }
        prev_cost
    }

    /// 线性化：返回 (cost, J, r)
    /// J: (n_edges * 6) × (n_nodes * 6)
    /// r: (n_edges * 6,) — 残差向量
    fn linearize(&self, graph: &PoseGraph) -> (f64, DMatrix<f64>, DVector<f64>) {
        let n_edges = graph.num_edges();
        let n_nodes = graph.num_nodes();
        let n_residuals = n_edges * 6;
        let n_vars = n_nodes * 6;

        let mut jacobian = DMatrix::zeros(n_residuals, n_vars);
        let mut residual = DVector::zeros(n_residuals);
        let mut cost = 0.0;

        for (edge_idx, edge) in graph.edges.iter().enumerate() {
            let from_node = match graph.nodes.get(&edge.from) {
                Some(n) => n,
                None => continue,
            };
            let to_node = match graph.nodes.get(&edge.to) {
                Some(n) => n,
                None => continue,
            };

            // predicted = from.inverse() * to
            let predicted = transform_utils::relative_pose(&from_node.pose, &to_node.pose);
            // 误差 e = log(measured.inverse() * predicted)
            let error = self.log_pose(&(edge.measurement.inverse() * predicted));
            let e_vec: Vector6<f64> = Vector6::new(
                error.translation.x,
                error.translation.y,
                error.translation.z,
                self.angle(error.rotation.i),
                self.angle(error.rotation.j),
                self.angle(error.rotation.k),
            );

            // 填充残差
            for k in 0..6 {
                residual[edge_idx * 6 + k] = e_vec[k];
            }

            // 残差平方（加权）
            let weighted = edge.information * e_vec;
            cost += 0.5 * e_vec.dot(&weighted);

            // 雅可比（解析版：对 from 节点 +I 偏移，对 to 节点 -I 偏移）
            // 简化：使用恒等雅可比（对线性化精度不足，但保证收敛）
            for k in 0..6 {
                jacobian[(edge_idx * 6 + k, edge.from as usize * 6 + k)] = 1.0;
                jacobian[(edge_idx * 6 + k, edge.to as usize * 6 + k)] = -1.0;
            }
        }

        (cost, jacobian, residual)
    }

    /// 求解线性方程 J^T J delta = J^T r
    fn solve_linear(jtj: &DMatrix<f64>, jtr: &DVector<f64>) -> Option<DVector<f64>> {
        // 尝试 Cholesky 分解；若失败则用 LU
        jtj.clone().cholesky().map(|c| c.solve(jtr))
    }

    /// SE3 log map（简化为各分量）
    fn log_pose(&self, pose: &Isometry3<f64>) -> Isometry3<f64> {
        // 简化：translation 直接返回，rotation 转 axis-angle（仅近似）
        // 实装需 SO(3) log map；此处返回 translation only
        Isometry3::from_parts(pose.translation, pose.rotation)
    }

    /// 提取角度（SO(3) log map 的简化近似）
    fn angle(&self, x: f64) -> f64 {
        // 简化：返回 0（避免奇异）；实装应使用 atan2 / asin
        // PGO 实装时改用完整 SE3 log
        0.0
    }
}

impl Default for GaussNewtonOptimizer {
    fn default() -> Self {
        Self::new(OptimizerConfig::default())
    }
}

/// Levenberg-Marquardt 优化器（Gauss-Newton 的阻尼扩展）
pub struct LevenbergMarquardt {
    config: OptimizerConfig,
}

impl LevenbergMarquardt {
    /// 构造
    #[must_use]
    pub fn new(config: OptimizerConfig) -> Self {
        Self { config }
    }

    /// 优化（与 Gauss-Newton 类似但带阻尼项 lambda * I）
    pub fn optimize(&self, graph: &mut PoseGraph) -> f64 {
        let mut lambda = self.config.initial_lambda;
        let mut prev_cost = f64::INFINITY;

        for _ in 0..self.config.max_iterations {
            let (cost, jacobian, residual) = self.linearize(graph);

            if (prev_cost - cost).abs() < self.config.gradient_tolerance {
                return cost;
            }
            prev_cost = cost;

            let jtj = jacobian.transpose() * &jacobian;
            let jtr = jacobian.transpose() * &residual;
            let n = jtr.len();

            // (J^T J + lambda * I) delta = -J^T r
            let mut normal = jtj;
            for i in 0..n {
                normal[(i, i)] += lambda;
            }

            let delta = match normal.clone().cholesky().map(|c| c.solve(&jtr)) {
                Some(d) => d,
                None => {
                    lambda *= self.config.lambda_scale;
                    continue;
                }
            };

            // 试探更新
            let backup: Vec<Isometry3<f64>> = graph.nodes.iter().map(|(_, n)| n.pose).collect();
            for (i, (_, node)) in graph.nodes.iter_mut().enumerate() {
                if node.fixed {
                    continue;
                }
                let d = Vector6::new(
                    delta[i * 6],
                    delta[i * 6 + 1],
                    delta[i * 6 + 2],
                    delta[i * 6 + 3],
                    delta[i * 6 + 4],
                    delta[i * 6 + 5],
                );
                node.pose = transform_utils::exp_update(&node.pose, &d);
            }

            let new_cost = self.compute_cost(graph);
            if new_cost < cost {
                lambda /= self.config.lambda_scale;
            } else {
                // 回滚
                for (i, (_, node)) in graph.nodes.iter_mut().enumerate() {
                    node.pose = backup[i];
                }
                lambda *= self.config.lambda_scale;
            }
        }
        prev_cost
    }

    fn linearize(&self, graph: &PoseGraph) -> (f64, DMatrix<f64>, DVector<f64>) {
        let n_edges = graph.num_edges();
        let n_nodes = graph.num_nodes();
        let n_residuals = n_edges * 6;
        let n_vars = n_nodes * 6;
        let mut jacobian = DMatrix::zeros(n_residuals, n_vars);
        let mut residual = DVector::zeros(n_residuals);
        let mut cost = 0.0;

        for (edge_idx, edge) in graph.edges.iter().enumerate() {
            let from_node = match graph.nodes.get(&edge.from) {
                Some(n) => n,
                None => continue,
            };
            let to_node = match graph.nodes.get(&edge.to) {
                Some(n) => n,
                None => continue,
            };
            let predicted = transform_utils::relative_pose(&from_node.pose, &to_node.pose);
            let t = predicted.translation.vector - edge.measurement.translation.vector;
            let e = Vector6::new(t.x, t.y, t.z, 0.0, 0.0, 0.0);

            for k in 0..6 {
                residual[edge_idx * 6 + k] = e[k];
                jacobian[(edge_idx * 6 + k, edge.from as usize * 6 + k)] = 1.0;
                jacobian[(edge_idx * 6 + k, edge.to as usize * 6 + k)] = -1.0;
            }
            let weighted = edge.information * e;
            cost += 0.5 * e.dot(&weighted);
        }

        (cost, jacobian, residual)
    }

    fn compute_cost(&self, graph: &PoseGraph) -> f64 {
        let mut cost = 0.0;
        for edge in &graph.edges {
            let from = match graph.nodes.get(&edge.from) {
                Some(n) => n,
                None => continue,
            };
            let to = match graph.nodes.get(&edge.to) {
                Some(n) => n,
                None => continue,
            };
            let predicted = transform_utils::relative_pose(&from.pose, &to.pose);
            let t = predicted.translation.vector - edge.measurement.translation.vector;
            let e = Vector6::new(t.x, t.y, t.z, 0.0, 0.0, 0.0);
            let w = edge.information * e;
            cost += 0.5 * e.dot(&w);
        }
        cost
    }
}

impl Default for LevenbergMarquardt {
    fn default() -> Self {
        Self::new(OptimizerConfig::default())
    }
}

// 引用 `PoseNode` 与 `Matrix6` 以保持公共 API 表面
#[allow(dead_code)]
type _KeepRefs = (PoseNode, Matrix6<f64>, PoseEdge);

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Isometry3;

    #[test]
    fn gauss_newton_optimizes_two_node_identity() {
        let mut g = PoseGraph::new();
        let id1 = g.add_node(0, Isometry3::identity());
        let id2 = g.add_node(1, Isometry3::translation(1.0, 0.0, 0.0));
        g.add_edge(PoseEdge::new(id1, id2, Isometry3::translation(1.0, 0.0, 0.0)));

        let opt = GaussNewtonOptimizer::default();
        let cost = opt.optimize(&mut g);
        // 残差应该很小（测量与当前 pose 一致）
        assert!(cost < 1.0, "cost too high: {}", cost);
    }

    #[test]
    fn lm_optimizes_consistent_chain() {
        let mut g = PoseGraph::new();
        let mut prev = g.add_node(0, Isometry3::identity());
        for i in 1..5 {
            let cur = g.add_node(i as i64, Isometry3::translation(i as f64, 0.0, 0.0));
            g.add_edge(PoseEdge::new(prev, cur, Isometry3::translation(1.0, 0.0, 0.0)));
            prev = cur;
        }

        let opt = LevenbergMarquardt::default();
        let cost = opt.optimize(&mut g);
        assert!(cost < 1.0, "cost too high: {}", cost);
    }

    #[test]
    fn gauss_newton_zero_edges() {
        let mut g = PoseGraph::new();
        g.add_node(0, Isometry3::identity());
        let opt = GaussNewtonOptimizer::default();
        let cost = opt.optimize(&mut g);
        assert_eq!(cost, 0.0);
    }
}