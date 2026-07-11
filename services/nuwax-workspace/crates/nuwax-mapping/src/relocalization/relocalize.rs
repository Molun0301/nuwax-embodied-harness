//! ICP（Iterative Closest Point）算法
//! 算法：
//! 1. 对每个 source 点，在 target 中查找最近邻（kiddo KD-tree）
//! 2. 用 SVD 计算 best-fit 变换
//! 3. 应用变换，重复直到收敛
//!
//! RELOCALIZE PARTIAL：see audit H-9 (rework20)。
//! 当前实装已加入 gravity filter（z-axis tilt 阈值）与 wall subset selection
//! （基于法线 z 分量的水平约束）。FPFH / RANSAC 完整管线（dimos `_preprocess` +
//! `_ransac` + multi-scale 多启动）划归 P2。

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

// =====================================================================
// H-9 stubs：gravity filter + wall subset（FPFH/RANSAC 留 P2）
// =====================================================================

/// gravity 倾斜阈值（度）
///
/// 对齐 dimos `GRAVITY_TILT_MAX_DEG = 10.0`：
/// 候选变换的 z 轴与世界 z-up 夹角超过此值视为异常候选（典型：z-degenerate
/// floor/floor-only 对齐），应被丢弃。
pub const GRAVITY_TILT_MAX_DEG: f64 = 10.0;

/// 墙子集法线 z 分量阈值
///
/// 对齐 dimos `wall_subset` 的 `mask = |nrm[:, 2]| < 0.7`：
/// 法线接近水平的点（即墙面）被保留，法线接近垂直的点（地板/天花板）被过滤。
pub const WALL_NORMAL_Z_MAX: f64 = 0.7;

/// 计算变换相对 world z-up 的倾斜角（度）。
///
/// 返回 `Isometry3::rotation` 的局部 z 轴与 `[0, 0, 1]` 之间的夹角。
/// identity 旋转 → 0°；90° roll/pitch → 90°。
pub fn gravity_tilt_deg(transform: &Isometry3<f64>) -> f64 {
    let z_local = transform.rotation * Vector3::z();
    let cos = z_local.z.clamp(-1.0, 1.0);
    cos.acos().to_degrees()
}

/// 检查变换是否"重力合宜"（z 轴不超过阈值倾斜）。
pub fn is_upright(transform: &Isometry3<f64>, max_tilt_deg: f64) -> bool {
    gravity_tilt_deg(transform) <= max_tilt_deg
}

/// 在一组候选变换中过滤掉重力不合宜的；全不合宜时回退到原集合
/// （与 dimos `pool = upright if upright else candidates` 行为一致）。
pub fn filter_upright(candidates: &[Isometry3<f64>], max_tilt_deg: f64) -> Vec<Isometry3<f64>> {
    let upright: Vec<Isometry3<f64>> = candidates
        .iter()
        .copied()
        .filter(|t| is_upright(t, max_tilt_deg))
        .collect();
    if upright.is_empty() {
        candidates.to_vec()
    } else {
        upright
    }
}

/// 墙子集：根据法线 z 分量过滤（|nrm.z| < 阈值 → 保留）。
///
/// 返回值是与输入等长的 `Vec<bool>`，true 表示该点被保留为墙体候选。
/// 调用方据此过滤 `PointCloud`：
///
/// ```ignore
/// let keep = wall_subset_mask(&normals, WALL_NORMAL_Z_MAX);
/// let walls: Vec<Point3<f64>> = points.iter().zip(keep.iter())
/// .filter_map(|(p, &k)| k.then_some(*p)).collect();
/// ```
pub fn wall_subset_mask(normals: &[Vector3<f64>], threshold: f64) -> Vec<bool> {
    normals.iter().map(|n| n.z.abs() < threshold).collect()
}

// =====================================================================
// H-9 stubs：FPFH 描述子 + RANSAC + 多阶段 ICP 入口（完整实装留 P2）
// =====================================================================

/// FPFH（Fast Point Feature Histogram）描述子 — 占位
///
/// 对齐 dimos `_preprocess` 中 FPFH 计算。本期仅返回 `Vec<f32>` 占位（长度 0）；
/// 真实 FPFH（33 维直方图）实装需 normals + kd-tree + 角度 binning，留 P2。
///
/// TODO(audit H-9): 接 Open3D `compute_fpfh_feature` 或自研 33-bin hist
#[must_use]
pub fn compute_fpfh_stub(_points: &[Point3<f64>], _normals: &[Vector3<f64>]) -> Vec<f32> {
    Vec::new()
}

/// RANSAC 单次拟合：随机采样 3 对对应点 + SVD best-fit → 刚体变换（占位）
///
/// 完整 RANSAC 含：
/// 1. 随机采样 3 对（最小刚体拟合样本）
/// 2. SVD best-fit 求变换
/// 3. 评估 inlier 数（残差 < inlier_threshold）
/// 4. 迭代 max_iters 次或 best_inlier_count 稳定
///
/// 本期只暴露入口签名为 caller 接入对齐，实装用 `IcpSolver::align` 的单次 best-fit
/// 近似（用所有点而非 RANSAC 子集），正确率会偏低但语义对齐。
///
/// TODO(audit H-9): 真 RANSAC 留 P2
#[must_use]
pub fn ransac_align_stub(
    source: &[Point3<f64>],
    target: &[Point3<f64>],
    max_iters: usize,
    inlier_threshold: f64,
) -> Isometry3<f64> {
    let _ = (max_iters, inlier_threshold);
    let solver = IcpSolver::default();
    let result = solver.align(source, target);
    result.transformation
}

/// 多阶段 ICP（粗→细）— 占位入口
///
/// 完整 multi-scale：粗尺度（max_corr 大）→ 细尺度（max_corr 小）串接，
/// 每阶段用上一阶段结果初始化下一阶段。
///
/// 本期实装：两次串接（粗 1.0m → 细 0.1m），符合 dimos `multi_scale_icp` 入口语义。
///
/// TODO(audit H-9): 全阶段实装 + scale-aware 接受阈值留 P2
#[must_use]
pub fn multiscale_icp(
    source: &[Point3<f64>],
    target: &[Point3<f64>],
    coarse_distance: f64,
    fine_distance: f64,
) -> IcpResult {
    // 阶段 1：粗尺度（max_corr = coarse_distance）
    let coarse_cfg = IcpConfig {
        max_correspondence_distance: coarse_distance,
        ..IcpConfig::default()
    };
    let coarse = IcpSolver::new(coarse_cfg).align(source, target);

    // 阶段 2：用粗变换初始化 + 细尺度（max_corr = fine_distance）
    let fine_cfg = IcpConfig {
        max_correspondence_distance: fine_distance,
        ..IcpConfig::default()
    };
    let transformed: Vec<Point3<f64>> = source
        .iter()
        .map(|p| coarse.transformation * p)
        .collect();
    let fine = IcpSolver::new(fine_cfg).align(&transformed, target);

    // 合并：total = fine * coarse
    IcpResult {
        transformation: fine.transformation * coarse.transformation,
        mse: fine.mse,
        iterations: coarse.iterations + fine.iterations,
        converged: fine.converged,
    }
}

/// 完整 relocation pipeline（FPFH + RANSAC + multi-scale ICP）— stub
///
/// 对应 dimos `RelocalizationModule._preprocess` + `_ransac` + `multi_scale_icp` 全链路。
/// 本期骨架版只走 multi-scale ICP + gravity filter；FPFH/RANSAC 留 P2。
///
/// TODO(audit H-9): 接 FPFH 描述子 + 真 RANSAC 留 P2（不影响本期 compile/run）
#[must_use]
pub fn relocation_pipeline(
    source: &[Point3<f64>],
    target: &[Point3<f64>],
) -> IcpResult {
    // 步骤 1：FPFH stub（暂时返回空 → 后续 RANSAC 直接走 ICP）
    let _fpfh = compute_fpfh_stub(source, &[]);

    // 步骤 2：multi-scale ICP（粗 1.0m → 细 0.1m）
    let result = multiscale_icp(source, target, 1.0, 0.1);

    // 步骤 3：gravity filter（z-tilt > 10° 视为异常 → 回退 identity）
    if !is_upright(&result.transformation, GRAVITY_TILT_MAX_DEG) {
        return IcpResult {
            transformation: Isometry3::identity(),
            mse: f64::INFINITY,
            iterations: result.iterations,
            converged: false,
        };
    }

    result
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

    // ----- H-9 修复新增用例 -----

    /// H-9：identity 旋转的 gravity 倾斜角应为 0°
    #[test]
    fn h9_gravity_tilt_identity_is_zero() {
        let t = Isometry3::identity();
        let tilt = gravity_tilt_deg(&t);
        assert!(tilt.abs() < 1e-9, "identity tilt must be 0, got {tilt}");
    }

    /// H-9:绕 y 轴 90° 的旋转 → tilt = 90°（被丢弃）
    #[test]
    fn h9_gravity_tilt_quarter_turn_y_is_ninety() {
        let q = UnitQuaternion::from_euler_angles(0.0, core::f64::consts::FRAC_PI_2, 0.0);
        let t = Isometry3::from_parts(Translation3::identity(), q);
        let tilt = gravity_tilt_deg(&t);
        assert!((tilt - 90.0).abs() < 1e-6, "y-90° rotation tilt must be 90°, got {tilt}");
    }

    /// H-9：filter_upright 应该丢弃倾斜候选；全不合宜时回退
    #[test]
    fn h9_filter_upright_drops_tilted() {
        // 候选：1 identity + 1 倾斜 30°
        let id = Isometry3::identity();
        let tilted = Isometry3::from_parts(
            Translation3::identity(),
            UnitQuaternion::from_euler_angles(0.0, core::f64::consts::FRAC_PI_6, 0.0),
        );
        let candidates = vec![tilted, id];
        let kept = filter_upright(&candidates, GRAVITY_TILT_MAX_DEG);
        assert_eq!(kept.len(), 1, "tilted 30° 候选应被丢弃");

        // 全不合宜：两个都是 30° 倾斜 → 回退
        let all_tilted = vec![tilted, tilted];
        let kept_all = filter_upright(&all_tilted, GRAVITY_TILT_MAX_DEG);
        assert_eq!(kept_all.len(), 2, "all tilted 必须回退到原集合");
    }

    /// H-9：wall_subset_mask |nrm.z| < 0.7 保留水平法线（墙面）
    #[test]
    fn h9_wall_subset_keeps_horizontal_normals() {
        use nalgebra::vector;
        let normals = vec![
            vector![1.0, 0.0, 0.0],    // 墙面 → |n.z|=0 < 0.7 → keep
            vector![0.0, 1.0, 0.0],    // 墙面 → keep
            vector![0.0, 0.0, 1.0],    // 天花板 → |n.z|=1 → drop
            vector![0.0, 0.0, -1.0],   // 地板 → |n.z|=1 → drop
            vector![0.6, 0.6, 0.6],    // |z|=0.6 < 0.7 → keep（边角墙面）
            vector![0.5, 0.5, 0.8],    // |z|=0.8 > 0.7 → drop（接近地板）
        ];
        let mask = wall_subset_mask(&normals, WALL_NORMAL_Z_MAX);
        assert_eq!(mask, vec![true, true, false, false, true, false]);
    }

    /// H-9:多尺度 ICP 入口存在性冒烟测试（确认 `IcpSolver::align` 可对多缩放点云调用）
    /// 实装下 multi-scale 是由 caller 在多个 IcpConfig（不同 max_correspondence_distance）
    /// 间串接 align() 实现；本测试仅校验 API + 不同尺度参数都能成功调用。
    #[test]
    fn h9_multiscale_icp_entry_exists() {
        let pts: Vec<Point3<f64>> = (0..10)
            .map(|i| Point3::new(i as f64 * 0.05, 0.0, 0.0))
            .collect();
        // 粗尺度
        let coarse = IcpConfig {
            max_correspondence_distance: 0.3,
            ..IcpConfig::default()
        };
        // 细尺度
        let fine = IcpConfig {
            max_correspondence_distance: 0.05,
            ..IcpConfig::default()
        };
        let _coarse_res = IcpSolver::new(coarse).align(&pts, &pts);
        let _fine_res = IcpSolver::new(fine).align(&pts, &pts);
        // 两个尺度都能跑通：标记 multi-scale ICP 入口可用
    }
}
