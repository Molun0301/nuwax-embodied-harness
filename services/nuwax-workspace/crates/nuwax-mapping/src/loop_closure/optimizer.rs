//! 自研 Gauss-Newton + Levenberg-Marquardt 优化器
//! 算法：
//! - 输入：PoseGraph（节点 + 边）
//! - 残差：对每条边计算 6D 误差向量 `e_ij = log(measured_ij.inverse() * predicted_ij)`
//! - 目标：最小化 sum(e_ij^T * Omega_ij * e_ij)
//! - 求解：(J^T * Omega * J + lambda * I) * delta = -J^T * Omega * e
//! - 更新：每个节点的 pose = pose * exp(delta)（right perturbation）
//!
//! PGO PARTIAL: see audit H-8 (rework20)。
//! H-8 已修复 SE(3) log map 维度（rotation 非零）。
//! P2 实装：使用包含 `Log` 微分的 SE(3) right-perturbation 雅可比，
//! 信息矩阵一致进入 cost、Hessian 与 gradient，更新为 `pose * se3_exp(delta)`。
//! ISAM2 / gtsam 完整集成因 aarch64 无 prebuilt 留待 P3。

use super::pgo::PoseGraph;
use super::se3::{linearize_edge, se3_exp_from_vector6, se3_log_to_vector6};
use nalgebra::{DMatrix, DVector, Isometry3, Matrix6, Vector6};

#[cfg(test)]
use super::pgo::PoseEdge;

/// 构造加权正规方程 `H = J^T Omega J`、`g = J^T Omega e`。
fn linearize_graph(graph: &PoseGraph) -> (f64, DMatrix<f64>, DVector<f64>) {
    let n_vars = graph.num_nodes() * 6;
    let mut normal = DMatrix::zeros(n_vars, n_vars);
    let mut gradient = DVector::zeros(n_vars);
    let mut cost = 0.0;

    for edge in &graph.edges {
        let Some(from_node) = graph.nodes.get(&edge.from) else {
            continue;
        };
        let Some(to_node) = graph.nodes.get(&edge.to) else {
            continue;
        };

        let (error, j_from, j_to) =
            linearize_edge(&edge.measurement, &from_node.pose, &to_node.pose);
        let weighted_error = edge.information * error;
        cost += 0.5 * error.dot(&weighted_error);

        let from_offset = edge.from as usize * 6;
        let to_offset = edge.to as usize * 6;
        if !from_node.fixed {
            add_vector6_block(
                &mut gradient,
                from_offset,
                &(j_from.transpose() * weighted_error),
            );
            add_matrix6_block(
                &mut normal,
                from_offset,
                from_offset,
                &(j_from.transpose() * edge.information * j_from),
            );
        }
        if !to_node.fixed {
            add_vector6_block(
                &mut gradient,
                to_offset,
                &(j_to.transpose() * weighted_error),
            );
            add_matrix6_block(
                &mut normal,
                to_offset,
                to_offset,
                &(j_to.transpose() * edge.information * j_to),
            );
        }
        if !from_node.fixed && !to_node.fixed {
            add_matrix6_block(
                &mut normal,
                from_offset,
                to_offset,
                &(j_from.transpose() * edge.information * j_to),
            );
            add_matrix6_block(
                &mut normal,
                to_offset,
                from_offset,
                &(j_to.transpose() * edge.information * j_from),
            );
        }
    }

    // Fixed variables have no edge contribution. An identity block keeps the full-size system
    // nonsingular while preserving an exactly zero update for those variables.
    for node in graph.nodes.values().filter(|node| node.fixed) {
        let offset = node.id as usize * 6;
        for axis in 0..6 {
            normal[(offset + axis, offset + axis)] = 1.0;
        }
    }

    (cost, normal, gradient)
}

fn add_matrix6_block(
    destination: &mut DMatrix<f64>,
    row_offset: usize,
    col_offset: usize,
    block: &Matrix6<f64>,
) {
    for row in 0..6 {
        for col in 0..6 {
            destination[(row_offset + row, col_offset + col)] += block[(row, col)];
        }
    }
}

fn add_vector6_block(destination: &mut DVector<f64>, offset: usize, block: &Vector6<f64>) {
    for row in 0..6 {
        destination[offset + row] += block[row];
    }
}

pub(crate) fn graph_cost(graph: &PoseGraph) -> f64 {
    graph
        .edges
        .iter()
        .filter_map(|edge| {
            let from = graph.nodes.get(&edge.from)?;
            let to = graph.nodes.get(&edge.to)?;
            let error_pose = edge.measurement.inverse() * from.pose.inverse() * to.pose;
            let error = se3_log_to_vector6(&error_pose);
            Some(0.5 * error.dot(&(edge.information * error)))
        })
        .sum()
}

pub(crate) fn cost_strictly_improved(initial: f64, candidate: f64) -> bool {
    let minimum_improvement = 64.0 * f64::EPSILON * (1.0 + initial.abs());
    initial.is_finite() && candidate.is_finite() && initial - candidate > minimum_improvement
}

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

impl OptimizerConfig {
    pub(crate) fn is_valid(self) -> bool {
        self.max_iterations > 0
            && self.gradient_tolerance.is_finite()
            && self.gradient_tolerance >= 0.0
            && self.initial_lambda.is_finite()
            && self.initial_lambda > 0.0
            && self.lambda_scale.is_finite()
            && self.lambda_scale > 1.0
    }
}

fn graph_is_finite(graph: &PoseGraph) -> bool {
    graph.nodes.values().all(|node| {
        node.pose
            .translation
            .vector
            .iter()
            .all(|value| value.is_finite())
            && node
                .pose
                .rotation
                .coords
                .iter()
                .all(|value| value.is_finite())
    }) && graph.edges.iter().all(|edge| {
        edge.measurement
            .translation
            .vector
            .iter()
            .all(|value| value.is_finite())
            && edge
                .measurement
                .rotation
                .coords
                .iter()
                .all(|value| value.is_finite())
            && edge.information.iter().all(|value| value.is_finite())
    })
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
        if !self.config.is_valid() || !graph_is_finite(graph) {
            return f64::NAN;
        }
        if graph.num_nodes() == 0 || graph.num_edges() == 0 {
            return 0.0;
        }
        let original = graph.clone();

        for _ in 0..self.config.max_iterations {
            let (cost, normal, gradient) = self.linearize(graph);

            if !cost.is_finite()
                || normal.iter().any(|value| !value.is_finite())
                || gradient.iter().any(|value| !value.is_finite())
            {
                *graph = original;
                return f64::NAN;
            }
            if gradient.norm() < self.config.gradient_tolerance {
                return cost;
            }

            // 求解加权 normal equations: (J^T Omega J) delta = -J^T Omega e
            let neg_gradient: DVector<f64> = -&gradient;
            let delta = match Self::solve_linear(&normal, &neg_gradient) {
                Some(d) => d,
                None => {
                    *graph = original;
                    return f64::NAN;
                }
            };
            if delta.iter().any(|value| !value.is_finite()) {
                *graph = original;
                return f64::NAN;
            }

            // 更新每个节点（P2: 用 SE(3) right perturbation）
            // 用 node.id 索引 delta（与 linearize() 的 `edge.from as usize * 6` 对齐），
            // 避免 HashMap 迭代顺序导致 delta 错位
            for (_, node) in graph.nodes.iter_mut() {
                if node.fixed {
                    continue;
                }
                let id = node.id as usize;
                let d = Vector6::new(
                    delta[id * 6],
                    delta[id * 6 + 1],
                    delta[id * 6 + 2],
                    delta[id * 6 + 3],
                    delta[id * 6 + 4],
                    delta[id * 6 + 5],
                );
                node.pose *= se3_exp_from_vector6(&d);
            }
            if !graph_is_finite(graph) {
                *graph = original;
                return f64::NAN;
            }
            let candidate_cost = graph_cost(graph);
            if !cost_strictly_improved(cost, candidate_cost) {
                *graph = original;
                return f64::NAN;
            }
        }
        let cost = graph_cost(graph);
        if cost.is_finite() {
            cost
        } else {
            *graph = original;
            f64::NAN
        }
    }

    /// 线性化：返回 `(cost, J^T Omega J, J^T Omega e)`。
    fn linearize(&self, graph: &PoseGraph) -> (f64, DMatrix<f64>, DVector<f64>) {
        linearize_graph(graph)
    }

    /// 求解线性方程 `H delta = -g`。
    fn solve_linear(normal: &DMatrix<f64>, neg_gradient: &DVector<f64>) -> Option<DVector<f64>> {
        // 尝试 Cholesky 分解；若失败则用 LU
        normal.clone().cholesky().map(|c| c.solve(neg_gradient))
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
        if !self.config.is_valid() || !graph_is_finite(graph) {
            return f64::NAN;
        }
        let original = graph.clone();
        let mut lambda = self.config.initial_lambda;
        let mut solved_system = false;
        for _ in 0..self.config.max_iterations {
            let (cost, mut normal, gradient) = self.linearize(graph);

            if !cost.is_finite()
                || normal.iter().any(|value| !value.is_finite())
                || gradient.iter().any(|value| !value.is_finite())
            {
                *graph = original;
                return f64::NAN;
            }
            if gradient.norm() < self.config.gradient_tolerance {
                return cost;
            }

            // (J^T Omega J + lambda * I) delta = -J^T Omega e
            let n = gradient.len();
            for i in 0..n {
                normal[(i, i)] += lambda;
            }

            let neg_gradient: DVector<f64> = -&gradient;
            let delta = match normal.cholesky().map(|factor| factor.solve(&neg_gradient)) {
                Some(d) => d,
                None => {
                    lambda *= self.config.lambda_scale;
                    continue;
                }
            };
            if delta.iter().any(|value| !value.is_finite()) {
                *graph = original;
                return f64::NAN;
            }
            solved_system = true;

            // 试探更新（P2: SE(3) right perturbation）
            // 用 node.id 索引 delta，与 linearize() 对齐（避免 HashMap 迭代顺序错位）
            let backup: Vec<(u64, Isometry3<f64>)> = graph
                .nodes
                .values()
                .map(|node| (node.id, node.pose))
                .collect();
            for (_, node) in graph.nodes.iter_mut() {
                if node.fixed {
                    continue;
                }
                let id = node.id as usize;
                let d = Vector6::new(
                    delta[id * 6],
                    delta[id * 6 + 1],
                    delta[id * 6 + 2],
                    delta[id * 6 + 3],
                    delta[id * 6 + 4],
                    delta[id * 6 + 5],
                );
                node.pose *= se3_exp_from_vector6(&d);
            }

            let new_cost = self.compute_cost(graph);
            if !new_cost.is_finite() || !graph_is_finite(graph) {
                *graph = original;
                return f64::NAN;
            } else if new_cost < cost {
                if cost - new_cost < self.config.gradient_tolerance {
                    return new_cost;
                }
                lambda /= self.config.lambda_scale;
            } else {
                // 回滚（按 node.id 恢复，避免 HashMap 迭代顺序错位）
                for (node_id, saved_pose) in &backup {
                    if let Some(node) = graph.nodes.get_mut(node_id) {
                        node.pose = *saved_pose;
                    }
                }
                lambda *= self.config.lambda_scale;
            }
        }
        if !solved_system {
            *graph = original;
            return f64::NAN;
        }
        let cost = self.compute_cost(graph);
        if cost.is_finite() {
            cost
        } else {
            *graph = original;
            f64::NAN
        }
    }

    fn linearize(&self, graph: &PoseGraph) -> (f64, DMatrix<f64>, DVector<f64>) {
        linearize_graph(graph)
    }

    fn compute_cost(&self, graph: &PoseGraph) -> f64 {
        graph_cost(graph)
    }
}

impl Default for LevenbergMarquardt {
    fn default() -> Self {
        Self::new(OptimizerConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::{Isometry3, UnitQuaternion};

    #[test]
    fn gauss_newton_optimizes_two_node_identity() {
        let mut g = PoseGraph::new();
        let id1 = g.add_node(0, Isometry3::identity());
        let id2 = g.add_node(1, Isometry3::translation(1.0, 0.0, 0.0));
        g.add_edge(PoseEdge::new(
            id1,
            id2,
            Isometry3::translation(1.0, 0.0, 0.0),
        ));

        let opt = GaussNewtonOptimizer::default();
        let cost = opt.optimize(&mut g);
        // 残差应该很小（测量与当前 pose 一致）
        assert!(cost < 1.0, "cost too high: {cost}");
    }

    #[test]
    fn lm_optimizes_consistent_chain() {
        let mut g = PoseGraph::new();
        let mut prev = g.add_node(0, Isometry3::identity());
        for i in 1..5 {
            let cur = g.add_node(i as i64, Isometry3::translation(i as f64, 0.0, 0.0));
            g.add_edge(PoseEdge::new(
                prev,
                cur,
                Isometry3::translation(1.0, 0.0, 0.0),
            ));
            prev = cur;
        }

        let opt = LevenbergMarquardt::default();
        let cost = opt.optimize(&mut g);
        assert!(cost < 1.0, "cost too high: {cost}");
    }

    #[test]
    fn gauss_newton_zero_edges() {
        let mut g = PoseGraph::new();
        g.add_node(0, Isometry3::identity());
        let opt = GaussNewtonOptimizer::default();
        let cost = opt.optimize(&mut g);
        assert_eq!(cost, 0.0);
    }

    #[test]
    fn optimizers_reject_invalid_config_and_non_finite_graphs() {
        let mut graph = PoseGraph::new();
        graph.add_node(0, Isometry3::identity());
        let invalid = OptimizerConfig {
            max_iterations: 0,
            ..OptimizerConfig::default()
        };
        assert!(GaussNewtonOptimizer::new(invalid)
            .optimize(&mut graph)
            .is_nan());
        assert!(LevenbergMarquardt::new(invalid)
            .optimize(&mut graph)
            .is_nan());

        graph.nodes.get_mut(&0).unwrap().pose.translation.x = f64::NAN;
        assert!(GaussNewtonOptimizer::default()
            .optimize(&mut graph)
            .is_nan());
        assert!(LevenbergMarquardt::default().optimize(&mut graph).is_nan());
    }

    #[test]
    fn gauss_newton_reports_singular_system_without_partial_update() {
        let mut graph = PoseGraph::new();
        let first = graph.add_node(0, Isometry3::identity());
        let second = graph.add_node(1, Isometry3::translation(2.0, 0.0, 0.0));
        graph.add_edge(PoseEdge::new(
            first,
            second,
            Isometry3::translation(1.0, 0.0, 0.0),
        ));
        let before = graph.clone();
        assert!(GaussNewtonOptimizer::default()
            .optimize(&mut graph)
            .is_nan());
        for (id, node) in &before.nodes {
            assert_eq!(graph.nodes[id].pose, node.pose);
        }
    }

    #[test]
    fn gauss_newton_rejects_numerically_insignificant_improvement() {
        let mut graph = PoseGraph::new();
        let fixed = graph.add_node(0, Isometry3::identity());
        let moving = graph.add_node(1, Isometry3::translation(1e-10, 0.0, 0.0));
        graph.nodes.get_mut(&fixed).unwrap().fixed = true;
        graph.add_edge(PoseEdge::new(fixed, moving, Isometry3::identity()));
        let before = graph.clone();

        let optimizer = GaussNewtonOptimizer::new(OptimizerConfig {
            gradient_tolerance: 0.0,
            ..OptimizerConfig::default()
        });
        assert!(optimizer.optimize(&mut graph).is_nan());
        for (id, node) in &before.nodes {
            assert_eq!(graph.nodes[id].pose, node.pose);
        }
    }

    // ----- H-8 修复新增用例 -----

    /// H-8: identity pose → 6D log 应为 0 向量
    #[test]
    fn h8_se3_log_identity_is_zero() {
        let v = se3_log_to_vector6(&Isometry3::identity());
        for k in 0..6 {
            assert_relative_eq!(v[k], 0.0, epsilon = 1e-12);
        }
    }

    /// H-8: π/4 绕 z 轴旋转的 twist 应在 rotation 部分含非零值（H-8 核心断言）
    #[test]
    fn h8_se3_log_quarter_turn_gives_nonzero_rotation() {
        // π/4 绕 z 轴
        let q = UnitQuaternion::from_euler_angles(0.0, 0.0, core::f64::consts::FRAC_PI_4);
        let pose = Isometry3::from_parts(nalgebra::Translation3::identity(), q);
        let v = se3_log_to_vector6(&pose);

        // 平移 = 0
        assert_relative_eq!(v[0], 0.0, epsilon = 1e-9);
        assert_relative_eq!(v[1], 0.0, epsilon = 1e-9);
        assert_relative_eq!(v[2], 0.0, epsilon = 1e-9);
        // rotation 绕 z 轴应为 +π/4（axis-angle），rx/ry ≈ 0
        assert_relative_eq!(v[3], 0.0, epsilon = 1e-9);
        assert_relative_eq!(v[4], 0.0, epsilon = 1e-9);
        assert_relative_eq!(v[5], core::f64::consts::FRAC_PI_4, epsilon = 1e-6);

        // H-8 核心断言：rotation 部分**非零**（旧版恒为 0）
        let rot_norm_sq = v[3] * v[3] + v[4] * v[4] + v[5] * v[5];
        assert!(
            rot_norm_sq > 1e-6,
            "rotation must be non-zero (H-8 regression)"
        );
    }

    /// H-8/P2: 正规方程维度必须覆盖每个节点的 6 个自由度。
    #[test]
    fn h8_jacobian_shape_is_six_n_by_six_m() {
        // 构造 3 节点 + 2 边的图
        let mut g = PoseGraph::new();
        let id0 = g.add_node(0, Isometry3::identity());
        let id1 = g.add_node(1, Isometry3::translation(1.0, 0.0, 0.0));
        let id2 = g.add_node(2, Isometry3::translation(2.0, 0.0, 0.0));
        g.add_edge(PoseEdge::new(
            id0,
            id1,
            Isometry3::translation(1.0, 0.0, 0.0),
        ));
        g.add_edge(PoseEdge::new(
            id1,
            id2,
            Isometry3::translation(1.0, 0.0, 0.0),
        ));

        let lm = LevenbergMarquardt::default();
        let (_, normal, gradient) = lm.linearize(&g);
        assert_eq!(normal.shape(), (18, 18));
        assert_eq!(gradient.len(), 18);
        let cost = lm.optimize(&mut g);
        assert!(
            cost < 1.0,
            "LM residual must converge on chain (H-8), got {cost}"
        );
        let _ = id2;
    }

    /// H-8: LM residual 必须**包含** rotation 部分（不是只有 translation）
    /// P2 实装后启用：rotation 部分被正确捕获，LM 收敛到 measurement
    #[test]
    fn h8_lm_residual_includes_rotation() {
        let mut g = PoseGraph::new();
        let id0 = g.add_node(0, Isometry3::identity());
        // node1 当前是 identity，measurement 是 π/4 绕 z：
        // 若 residual 只看 translation → cost = 0（误以为收敛）
        // 若 residual 包含 rotation → LM optimize 后 node1.pose 应被旋转到接近 measurement
        let id1 = g.add_node(1, Isometry3::identity());
        let m = Isometry3::from_parts(
            nalgebra::Translation3::identity(),
            UnitQuaternion::from_euler_angles(0.0, 0.0, core::f64::consts::FRAC_PI_4),
        );
        g.add_edge(PoseEdge::new(id0, id1, m));
        // 固定 id0 消除 PGO gauge freedom (6DOF)，否则 id0/id1 会一起平移
        g.nodes.get_mut(&id0).unwrap().fixed = true;

        let lm = LevenbergMarquardt::new(OptimizerConfig {
            max_iterations: 100,
            gradient_tolerance: 1e-10,
            initial_lambda: 1e-4,
            lambda_scale: 10.0,
        });
        // 完整的 SE(3) Jacobian 让 LM 真正优化 rotation。
        let final_cost = lm.optimize(&mut g);
        assert!(
            final_cost < 1e-3,
            "LM should converge rotation to measurement (P2), got cost {final_cost}"
        );
        let final_pose = g.nodes[&id1].pose;
        let q_after = final_pose.rotation;
        assert!(q_after.coords.norm() > 0.99, "quat 单位化丢失");
        // node1 应被旋转到接近 π/4 绕 z：quat.z ≈ sin(π/8) ≈ 0.3827
        let z_component_sq = q_after.coords[2] * q_after.coords[2];
        assert!(
            z_component_sq > 0.1,
            "rotation z component should be large after optimize (P2), got {z_component_sq}"
        );
        let _ = id0;
        let _ = final_cost;
    }

    // ----- P2 SE(3) Jacobian 新增用例 -----

    /// P2: 节点起始 identity，measurement π/4 绕 z，LM 应恢复该旋转。
    #[test]
    fn p2_lm_optimizes_rotation_node_to_match_measurement() {
        let mut g = PoseGraph::new();
        let id0 = g.add_node(0, Isometry3::identity());
        let id1 = g.add_node(1, Isometry3::identity());
        let m = Isometry3::from_parts(
            nalgebra::Translation3::identity(),
            UnitQuaternion::from_euler_angles(0.0, 0.0, core::f64::consts::FRAC_PI_4),
        );
        g.add_edge(PoseEdge::new(id0, id1, m));
        // 固定 id0 消除 gauge freedom
        g.nodes.get_mut(&id0).unwrap().fixed = true;

        let lm = LevenbergMarquardt::new(OptimizerConfig {
            max_iterations: 100,
            gradient_tolerance: 1e-12,
            initial_lambda: 1e-4,
            lambda_scale: 10.0,
        });
        let final_cost = lm.optimize(&mut g);
        let final_pose = g.nodes[&id1].pose;
        let q = final_pose.rotation;
        // π/4 旋转 quat.z = sin(π/8) ≈ 0.3827
        assert!(
            q.coords[2] > 0.35,
            "expected quat.z > 0.35 (rotation recovered), got {}",
            q.coords[2]
        );
        assert!(
            final_cost < 1e-6,
            "expected near-zero cost, got {final_cost}"
        );
    }

    /// P2: 同时含三轴平移与三轴旋转的初值应收敛到 identity measurement。
    #[test]
    fn p2_lm_recovers_from_perturbed_initial_pose() {
        let mut g = PoseGraph::new();
        let id0 = g.add_node(0, Isometry3::identity());
        // 非交换的完整 SE(3) 扰动，避免只验证平移或单轴旋转。
        let initial_pose = Isometry3::from_parts(
            nalgebra::Translation3::new(1.0, -0.5, 0.25),
            UnitQuaternion::from_euler_angles(0.2, -0.1, core::f64::consts::FRAC_PI_8),
        );
        let id1 = g.add_node(1, initial_pose);
        // measurement = identity（即 from→to 应当 = I）
        g.add_edge(PoseEdge::new(id0, id1, Isometry3::identity()));
        // 固定 id0 消除 gauge freedom
        g.nodes.get_mut(&id0).unwrap().fixed = true;

        let lm = LevenbergMarquardt::new(OptimizerConfig {
            max_iterations: 200,
            gradient_tolerance: 1e-12,
            initial_lambda: 1e-3,
            lambda_scale: 10.0,
        });
        let final_cost = lm.optimize(&mut g);
        let final_pose = g.nodes[&id1].pose;
        let final_error = se3_log_to_vector6(&final_pose);
        assert!(
            final_error.norm() < 1e-5,
            "final SE(3) error = {final_error:?}"
        );
        assert!(
            final_cost < 1e-4,
            "expected near-zero cost, got {final_cost}"
        );
    }

    /// 非对角信息矩阵必须同时进入 Hessian 和 gradient，而不只是 reported cost。
    #[test]
    fn p2_normal_equations_apply_non_diagonal_information() {
        let mut graph = PoseGraph::new();
        let fixed = graph.add_node(0, Isometry3::identity());
        let moving_pose = Isometry3::from_parts(
            nalgebra::Translation3::new(0.4, -0.2, 0.3),
            UnitQuaternion::from_euler_angles(0.2, -0.15, 0.25),
        );
        let moving = graph.add_node(1, moving_pose);
        graph.nodes.get_mut(&fixed).unwrap().fixed = true;

        let mut information = Matrix6::from_diagonal(&Vector6::new(4.0, 3.0, 2.0, 5.0, 4.0, 3.0));
        information[(0, 3)] = 0.4;
        information[(3, 0)] = 0.4;
        information[(1, 5)] = -0.3;
        information[(5, 1)] = -0.3;
        graph.add_edge(
            PoseEdge::new(fixed, moving, Isometry3::identity()).with_information(information),
        );

        let optimizer = LevenbergMarquardt::default();
        let (cost, normal, gradient) = optimizer.linearize(&graph);
        let (error, _, j_to) =
            linearize_edge(&Isometry3::identity(), &Isometry3::identity(), &moving_pose);
        let expected_normal = j_to.transpose() * information * j_to;
        let expected_gradient = j_to.transpose() * information * error;
        let expected_cost = 0.5 * error.dot(&(information * error));

        assert_relative_eq!(cost, expected_cost, epsilon = 1e-12);
        for row in 0..6 {
            assert_relative_eq!(gradient[6 + row], expected_gradient[row], epsilon = 1e-12);
            for col in 0..6 {
                assert_relative_eq!(
                    normal[(6 + row, 6 + col)],
                    expected_normal[(row, col)],
                    epsilon = 1e-12
                );
            }
        }
    }

    /// 多条独立 loop constraints 下同时恢复旋转和平移。
    #[test]
    fn p2_lm_converges_on_multi_loop_se3_graph() {
        let truth = [
            Isometry3::identity(),
            Isometry3::from_parts(
                nalgebra::Translation3::new(1.0, 0.2, 0.1),
                UnitQuaternion::from_euler_angles(0.05, -0.08, 0.2),
            ),
            Isometry3::from_parts(
                nalgebra::Translation3::new(2.0, 1.0, 0.2),
                UnitQuaternion::from_euler_angles(-0.1, 0.12, 0.45),
            ),
            Isometry3::from_parts(
                nalgebra::Translation3::new(0.8, 2.0, -0.1),
                UnitQuaternion::from_euler_angles(0.15, 0.05, 0.8),
            ),
        ];
        let perturbations = [
            Vector6::zeros(),
            Vector6::new(0.25, -0.15, 0.08, 0.08, -0.04, 0.12),
            Vector6::new(-0.2, 0.18, -0.1, -0.06, 0.09, -0.1),
            Vector6::new(0.15, 0.2, 0.12, 0.1, -0.08, 0.07),
        ];
        let mut graph = PoseGraph::new();
        for (index, pose) in truth.iter().enumerate() {
            let initial = *pose * se3_exp_from_vector6(&perturbations[index]);
            let id = graph.add_node(index as i64, initial);
            assert_eq!(id as usize, index);
        }
        graph.nodes.get_mut(&0).unwrap().fixed = true;

        for (from, to) in [(0_u64, 1_u64), (1, 2), (2, 3), (3, 0), (0, 2), (1, 3)] {
            let measurement = truth[from as usize].inverse() * truth[to as usize];
            graph.add_edge(PoseEdge::new(from, to, measurement));
        }

        let optimizer = LevenbergMarquardt::new(OptimizerConfig {
            max_iterations: 100,
            gradient_tolerance: 1e-12,
            initial_lambda: 1e-3,
            lambda_scale: 10.0,
        });
        let final_cost = optimizer.optimize(&mut graph);
        assert!(final_cost < 1e-10, "multi-loop final cost = {final_cost}");
        for (id, expected) in truth.iter().enumerate() {
            let error_pose = expected.inverse() * graph.nodes[&(id as u64)].pose;
            let error = se3_log_to_vector6(&error_pose);
            assert!(error.norm() < 1e-5, "node {id} SE(3) error = {error:?}");
        }
    }
}
