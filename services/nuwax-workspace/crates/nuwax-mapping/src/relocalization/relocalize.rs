//! ICP（Iterative Closest Point）算法
//! 算法：
//! 1. 对每个 source 点，在 target 中查找最近邻（kiddo KD-tree）
//! 2. 用 SVD 计算 best-fit 变换
//! 3. 应用变换，重复直到收敛

use kiddo::KdTree;
use nalgebra::{Isometry3, Matrix3, Point3, Translation3, UnitQuaternion, Vector3};

/// ICP 配置
#[derive(Debug, Clone, Copy)]
pub struct IcpConfig {
    /// 最大迭代次数
    pub max_iterations: usize,
    /// 收敛阈值（mean squared error）
    pub mse_tolerance: f64,
    /// 最大对应距离（m）—— 超过此值的对应视为 outlier 丢弃
    pub max_correspondence_distance: f64,
}

impl Default for IcpConfig {
    fn default() -> Self {
        Self {
            max_iterations: 50,
            mse_tolerance: 1e-6,
            max_correspondence_distance: 1.0,
        }
    }
}

/// ICP 结果
#[derive(Debug, Clone, Copy)]
pub struct IcpResult {
    /// 最终变换（source → target）
    pub transformation: Isometry3<f64>,
    /// 最终 MSE
    pub mse: f64,
    /// 实际迭代次数
    pub iterations: usize,
    /// 是否收敛
    pub converged: bool,
}

/// ICP 求解器
pub struct IcpSolver {
    config: IcpConfig,
}

impl IcpSolver {
    /// 构造
    #[must_use]
    pub fn new(config: IcpConfig) -> Self {
        Self { config }
    }

    /// 运行 ICP
    ///
    /// `source` 与 `target` 是两组 3D 点；返回 source → target 的刚体变换。
    #[must_use]
    pub fn align(&self, source: &[Point3<f64>], target: &[Point3<f64>]) -> IcpResult {
        if source.is_empty() || target.is_empty() {
            return IcpResult {
                transformation: Isometry3::identity(),
                mse: f64::INFINITY,
                iterations: 0,
                converged: false,
            };
        }

        // 构建 target KD-tree
        let mut tree: KdTree<f64, 3> = KdTree::new();
        for (i, p) in target.iter().enumerate() {
            tree.add(&[p.x, p.y, p.z], i as u64);
        }

        let mut current = Isometry3::identity();
        let mut prev_mse = f64::INFINITY;

        for iter in 0..self.config.max_iterations {
            // 阶段 1：变换 source → current
            let transformed: Vec<Point3<f64>> = source.iter().map(|p| current * p).collect();

            // 阶段 2：最近邻 + 过滤 outlier
            // kiddo v5 nearest_one::<A>() 返回 NearestNeighbour<A, T>：
            //   .distance = 距离平方（已过滤 Item=K 的 DistanceMetric）
            //   .item     = T = 索引
            let max_corr_sq = self.config.max_correspondence_distance.powi(2);
            let mut correspondences: Vec<(Point3<f64>, Point3<f64>)> = Vec::new();
            for sp in &transformed {
                let nn = tree.nearest_one::<kiddo::SquaredEuclidean>(&[sp.x, sp.y, sp.z]);
                if nn.distance <= max_corr_sq && (nn.item as usize) < target.len() {
                    correspondences.push((*sp, target[nn.item as usize]));
                }
            }

            if correspondences.len() < 3 {
                return IcpResult {
                    transformation: current,
                    mse: prev_mse,
                    iterations: iter,
                    converged: false,
                };
            }

            // 阶段 3：SVD best-fit
            let delta = Self::compute_best_fit(&correspondences);
            // ICP 标准更新：sp = current * source，delta * sp ≈ target，
            // 所以新的 total = delta * current（组合变换），即 current := delta * current
            current = delta * current;

            // 阶段 4：计算 MSE
            let mut sum_sq = 0.0;
            for (sp, tp) in &correspondences {
                let mapped = delta * sp;
                let diff = tp - mapped;
                sum_sq += diff.norm_squared();
            }
            let mse = sum_sq / correspondences.len() as f64;

            if (prev_mse - mse).abs() < self.config.mse_tolerance {
                return IcpResult {
                    transformation: current,
                    mse,
                    iterations: iter + 1,
                    converged: true,
                };
            }
            prev_mse = mse;
        }

        IcpResult {
            transformation: current,
            mse: prev_mse,
            iterations: self.config.max_iterations,
            converged: false,
        }
    }

    /// SVD best-fit 变换：返回把 src 对应点集映射到 dst 对应点集的刚体变换
    fn compute_best_fit(correspondences: &[(Point3<f64>, Point3<f64>)]) -> Isometry3<f64> {
        let n = correspondences.len() as f64;

        // 计算质心
        let mut src_mean = Vector3::zeros();
        let mut dst_mean = Vector3::zeros();
        for (s, d) in correspondences {
            src_mean += s.coords;
            dst_mean += d.coords;
        }
        src_mean /= n;
        dst_mean /= n;

        // 中心化 + 协方差矩阵
        let mut h = Matrix3::zeros();
        for (s, d) in correspondences {
            let s_c = s.coords - src_mean;
            let d_c = d.coords - dst_mean;
            h += s_c * d_c.transpose();
        }

        // SVD
        let svd = nalgebra::linalg::SVD::new(h, true, true);
        let u = svd.u.unwrap_or(Matrix3::identity());
        let v_t = svd.v_t.unwrap_or(Matrix3::identity());

        // 计算 det(V * U^T) 防止反射
        let det = (v_t.transpose() * u.transpose()).determinant();
        let mut d = Matrix3::identity();
        if det < 0.0 {
            d[(2, 2)] = -1.0;
        }
        let r = v_t.transpose() * d * u.transpose();

        let rotation = UnitQuaternion::from_matrix(&r);
        let translation = dst_mean - rotation * src_mean;

        Isometry3::from_parts(Translation3::from(translation), rotation)
    }
}

impl Default for IcpSolver {
    fn default() -> Self {
        Self::new(IcpConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::{point, vector};

    #[test]
    fn icp_identity_alignment() {
        // source 与 target 完全相同
        let pts = vec![
            point![0.0, 0.0, 0.0],
            point![1.0, 0.0, 0.0],
            point![0.0, 1.0, 0.0],
        ];
        let solver = IcpSolver::default();
        let res = solver.align(&pts, &pts);
        assert!(res.converged);
        assert!(res.mse < 1e-6);
    }

    #[test]
    fn icp_translation_recovery() {
        let target = vec![
            point![0.0, 0.0, 0.0],
            point![1.0, 0.0, 0.0],
            point![0.0, 1.0, 0.0],
        ];
        // source 平移了 (2, 3, 0)——最大对应距离需 ≥ 3.6 才能保留所有对应点
        let source: Vec<Point3<f64>> = target.iter().map(|p| p + vector![2.0, 3.0, 0.0]).collect();
        let cfg = IcpConfig {
            max_correspondence_distance: 10.0, // 放大以通过初值过滤
            ..IcpConfig::default()
        };
        let solver = IcpSolver::new(cfg);
        let res = solver.align(&source, &target);
        // 估计变换应能把 source 映射回 target：translation ≈ (-2, -3, 0)
        let t = res.transformation.translation;
        assert_relative_eq!(t.x, -2.0, epsilon = 0.1);
        assert_relative_eq!(t.y, -3.0, epsilon = 0.1);
        assert!(res.mse < 0.1);
    }

    #[test]
    fn icp_empty_inputs() {
        let solver = IcpSolver::default();
        let res = solver.align(&[], &[]);
        assert!(!res.converged);
        assert_eq!(res.iterations, 0);
    }
}
