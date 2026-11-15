//! Feature-based global relocalization followed by local ICP refinement.

use super::nearest::NearestPointIndex;
use nalgebra::{Isometry3, Matrix3, Point3, Translation3, UnitQuaternion, Vector3};

use super::features::{
    compute_fpfh, estimate_normals, match_fpfh_indexed, preprocess, voxel_downsample,
    FeatureConfig, FeatureMatch, ProcessedCloud,
};
use super::ransac::{ransac_align, RansacConfig};

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
        if source.is_empty()
            || target.is_empty()
            || source
                .iter()
                .chain(target)
                .any(|p| p.coords.iter().any(|v| !v.is_finite()))
        {
            return IcpResult {
                transformation: Isometry3::identity(),
                mse: f64::INFINITY,
                iterations: 0,
                converged: false,
            };
        }

        self.align_indexed(source, target, &NearestPointIndex::new(target))
    }

    /// 使用调用方已构建的目标索引运行配准。
    pub fn align_indexed(
        &self,
        source: &[Point3<f64>],
        target: &[Point3<f64>],
        tree: &NearestPointIndex,
    ) -> IcpResult {
        let mut current = Isometry3::identity();
        let mut prev_mse = f64::INFINITY;

        for iter in 0..self.config.max_iterations {
            // 阶段 1：变换 source → current
            let transformed: Vec<Point3<f64>> = source.iter().map(|p| current * p).collect();

            // 阶段 2：最近邻 + 过滤 outlier
            let max_corr_sq = self.config.max_correspondence_distance.powi(2);
            let mut correspondences: Vec<(Point3<f64>, Point3<f64>)> = Vec::new();
            for sp in &transformed {
                if let Some((index, distance_squared)) = tree.nearest(sp) {
                    if distance_squared <= max_corr_sq {
                        correspondences.push((*sp, target[index]));
                    }
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

// Gravity and wall-selection helpers retained from the original pipeline.

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

/// Compatibility entry point returning flattened 33-component FPFH descriptors.
///
/// New code should prefer [`compute_fpfh`], which preserves descriptor boundaries.
#[must_use]
pub fn compute_fpfh_stub(points: &[Point3<f64>], normals: &[Vector3<f64>]) -> Vec<f32> {
    compute_fpfh(
        points,
        normals,
        FeatureConfig::default().feature_radius,
        100,
    )
    .into_iter()
    .flat_map(|descriptor| descriptor.into_iter().map(|component| component as f32))
    .collect()
}

/// Compatibility entry point for index-corresponded deterministic geometric RANSAC.
///
/// New code should prefer [`ransac_align`] with FPFH-derived correspondences.
#[must_use]
pub fn ransac_align_stub(
    source: &[Point3<f64>],
    target: &[Point3<f64>],
    max_iters: usize,
    inlier_threshold: f64,
) -> Isometry3<f64> {
    let matches: Vec<FeatureMatch> = (0..source.len().min(target.len()))
        .map(|index| FeatureMatch {
            source_index: index,
            target_index: index,
            descriptor_distance: 0.0,
        })
        .collect();
    ransac_align(
        source,
        target,
        &NearestPointIndex::new(target),
        &matches,
        RansacConfig {
            max_iterations: max_iters,
            inlier_threshold,
            min_inliers: 3,
            min_inlier_ratio: 0.2,
            ..RansacConfig::default()
        },
    )
    .transformation
}

/// Legacy two-stage ICP compatibility entry point.
///
/// 完整 multi-scale：粗尺度（max_corr 大）→ 细尺度（max_corr 小）串接，
/// 每阶段用上一阶段结果初始化下一阶段。
///
/// Two local refinement passes are composed from coarse to fine correspondence radii.
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
    let target_index = NearestPointIndex::new(target);
    let coarse = IcpSolver::new(coarse_cfg).align_indexed(source, target, &target_index);

    // 阶段 2：用粗变换初始化 + 细尺度（max_corr = fine_distance）
    let fine_cfg = IcpConfig {
        max_correspondence_distance: fine_distance,
        ..IcpConfig::default()
    };
    let transformed: Vec<Point3<f64>> = source.iter().map(|p| coarse.transformation * p).collect();
    let fine = IcpSolver::new(fine_cfg).align_indexed(&transformed, target, &target_index);

    // 合并：total = fine * coarse
    IcpResult {
        transformation: fine.transformation * coarse.transformation,
        mse: fine.mse,
        iterations: coarse.iterations + fine.iterations,
        converged: fine.converged,
    }
}

/// 同一次地图加载期间复用的目标数据。
pub struct TargetCache {
    /// 0.1米目标点。
    pub points: Vec<Point3<f64>>,
    /// 精细层最近邻索引。
    pub index: NearestPointIndex,
    /// 地图坐标中的0.1米占据体素。
    pub occupied: std::collections::HashSet<[i64; 3]>,
    global_source: Vec<Point3<f64>>,
    scales: std::sync::OnceLock<Vec<ProcessedCloud>>,
}
impl TargetCache {
    /// 每次加载的第一次重定位构建一次。
    pub fn new(target: &[Point3<f64>], config: RelocalizationConfig) -> Self {
        let points = voxel_downsample(target, config.fine_voxel_size);
        let index = NearestPointIndex::new(&points);
        let occupied = points.iter().map(voxel_key).collect();
        Self {
            points,
            index,
            occupied,
            global_source: target.to_vec(),
            scales: std::sync::OnceLock::new(),
        }
    }

    fn global_scales(&self, config: RelocalizationConfig) -> &[ProcessedCloud] {
        self.scales.get_or_init(|| {
            config
                .voxel_sizes
                .into_iter()
                .map(|size| {
                    preprocess(
                        &self.global_source,
                        FeatureConfig::for_voxel(size, config.max_feature_neighbors),
                    )
                })
                .collect()
        })
    }
}

fn voxel_key(point: &Point3<f64>) -> [i64; 3] {
    [point.x, point.y, point.z].map(|v| (v / 0.1).floor() as i64)
}

/// 在里程计坐标中降采样的源点及其所属帧原点。
pub struct SourceCloud {
    /// 源点。
    pub points: Vec<Point3<f64>>,
    /// 与源点一一对应的射线原点。
    pub origins: Vec<Point3<f64>>,
    walls: Vec<bool>,
}
impl SourceCloud {
    /// 每个0.1米体素保留首个实测点，原点来自该点所属帧。
    pub fn from_rays(rays: impl IntoIterator<Item = (Point3<f64>, Point3<f64>)>) -> Self {
        let mut voxels = std::collections::BTreeMap::new();
        for (point, origin) in rays {
            voxels.entry(voxel_key(&point)).or_insert((point, origin));
        }
        let (points, origins): (Vec<_>, Vec<_>) = voxels.into_values().unzip();
        let walls = wall_subset_mask(&estimate_normals(&points, 0.2, 100), WALL_NORMAL_Z_MAX);
        Self {
            points,
            origins,
            walls,
        }
    }
}

/// 局部和全局共用的墙面与自由空间评分。
#[derive(Clone, Copy, Debug)]
pub struct RelocalizationScore {
    /// 墙面源点中0.15米内有目标点的比例。
    pub wall_fitness: f64,
    /// 射线中段穿过地图占据体素的比例。
    pub penetration: f64,
    /// 墙面内点均方根误差。
    pub rmse: f64,
}
impl RelocalizationScore {
    /// 按任务规定的四个条件判断结果。
    pub fn accepts(&self, transform: &Isometry3<f64>) -> bool {
        self.wall_fitness >= 0.45
            && self.penetration <= 0.15
            && is_upright(transform, GRAVITY_TILT_MAX_DEG)
            && self.rmse.is_finite()
    }
}

/// 对带所属帧射线原点的降采样源点评分。
pub fn score_relocalization(
    source: &SourceCloud,
    transform: Isometry3<f64>,
    target: &TargetCache,
) -> RelocalizationScore {
    let mut walls = 0_usize;
    let mut inliers = 0_usize;
    let mut squared_error = 0.0;
    let mut rays = 0_usize;
    let mut penetrated = 0_usize;
    for ((point, origin), wall) in source.points.iter().zip(&source.origins).zip(&source.walls) {
        if !wall {
            continue;
        }
        walls += 1;
        let point = transform * point;
        if let Some((_, distance)) = target.index.nearest(&point) {
            if distance <= 0.15_f64.powi(2) {
                inliers += 1;
                squared_error += distance;
            }
        }
        let origin = transform * origin;
        let direction = point - origin;
        let length = direction.norm();
        if length <= 0.8 {
            continue;
        }
        rays += 1;
        let direction = direction / length;
        let mut distance = 0.5;
        while distance < length - 0.3 {
            if target
                .occupied
                .contains(&voxel_key(&(origin + direction * distance)))
            {
                penetrated += 1;
                break;
            }
            distance += 0.1;
        }
    }
    RelocalizationScore {
        wall_fitness: inliers as f64 / walls.max(1) as f64,
        penetration: penetrated as f64 / rays.max(1) as f64,
        rmse: if inliers == 0 {
            f64::INFINITY
        } else {
            (squared_error / inliers as f64).sqrt()
        },
    }
}

/// 只在初值下机器人周围水平30米内做点到平面配准。
pub fn local_registration(
    source: &SourceCloud,
    target: &TargetCache,
    initial: Isometry3<f64>,
    odom: Isometry3<f64>,
) -> Result<IcpResult, String> {
    let position = (initial * odom).translation.vector;
    let local_target: Vec<[f64; 3]> = target
        .points
        .iter()
        .filter(|p| (p.x - position.x).powi(2) + (p.y - position.y).powi(2) <= 30.0_f64.powi(2))
        .map(|p| [p.x, p.y, p.z])
        .collect();
    let seeded: Vec<[f64; 3]> = source
        .points
        .iter()
        .map(|p| {
            let p = initial * p;
            [p.x, p.y, p.z]
        })
        .collect();
    let result = nuwax_gtsam::point_to_plane(&seeded, &local_target, 0.8, 50)?;
    let q = result.transformation.rotation_xyzw;
    let delta = Isometry3::from_parts(
        Translation3::from(result.transformation.translation),
        UnitQuaternion::new_normalize(nalgebra::Quaternion::new(q[3], q[0], q[1], q[2])),
    );
    Ok(IcpResult {
        transformation: delta * initial,
        mse: result.mse,
        iterations: result.iterations as usize,
        converged: result.converged != 0,
    })
}

/// Configuration for the complete feature/RANSAC/ICP relocalization pipeline.
#[derive(Debug, Clone, Copy)]
pub struct RelocalizationConfig {
    /// Voxel sizes used to generate independent global-registration candidates.
    pub voxel_sizes: [f64; 3],
    /// Maximum neighbours used for normal estimation and FPFH.
    pub max_feature_neighbors: usize,
    /// RANSAC sample budget at each voxel scale.
    pub ransac_iterations: usize,
    /// Minimum reciprocal feature matches required at a scale.
    pub min_feature_matches: usize,
    /// Voxel edge length used for final candidate scoring and ICP in metres.
    pub fine_voxel_size: f64,
    /// Maximum residual used for final candidate scoring and ICP in metres.
    pub fine_correspondence_distance: f64,
    /// RANSAC候选生成阶段的共识比例；最终接受由统一墙面评分决定。
    pub min_final_fitness: f64,
    /// ICP settings used for the final seeded local refinement.
    pub icp: IcpConfig,
}

impl Default for RelocalizationConfig {
    fn default() -> Self {
        let fine_voxel_size = 0.1;
        let fine_correspondence_distance = fine_voxel_size * 1.5;
        Self {
            voxel_sizes: [0.2, 0.3, 0.8],
            max_feature_neighbors: 100,
            ransac_iterations: 4_096,
            min_feature_matches: 3,
            fine_voxel_size,
            fine_correspondence_distance,
            min_final_fitness: 0.45,
            icp: IcpConfig {
                max_correspondence_distance: fine_correspondence_distance,
                ..IcpConfig::default()
            },
        }
    }
}

/// Stage diagnostics and accepted registration produced by relocalization.
#[derive(Debug, Clone, Copy)]
pub struct RelocalizationResult {
    /// Accepted source-to-target registration, or a non-converged identity on rejection.
    pub registration: IcpResult,
    /// Largest reciprocal FPFH match set observed at any configured scale.
    pub feature_matches: usize,
    /// Largest RANSAC geometric consensus observed at any configured scale.
    pub ransac_inliers: usize,
    /// 最终墙面适配度。
    pub fitness: f64,
    /// 两个不同位置同样符合地图，调用方应报告歧义。
    pub ambiguous: bool,
}

/// Run the complete multi-scale FPFH, deterministic RANSAC, and seeded ICP pipeline.
#[must_use]
pub fn relocation_pipeline_with_config(
    source: &[Point3<f64>],
    target: &[Point3<f64>],
    config: RelocalizationConfig,
) -> RelocalizationResult {
    let source = SourceCloud::from_rays(source.iter().map(|p| (*p, Point3::origin())));
    relocation_pipeline_cached(&source, &TargetCache::new(target, config), config)
}

/// 已加载地图上的全局配准，目标预处理由会话持有。
pub fn relocation_pipeline_cached(
    source: &SourceCloud,
    target: &TargetCache,
    config: RelocalizationConfig,
) -> RelocalizationResult {
    let fine_source = &source.points;
    if !valid_relocalization_config(config) || fine_source.len() < 3 || target.points.len() < 3 {
        return rejected_relocalization(0, 0, 0);
    }
    let mut maximum_feature_matches = 0;
    let mut maximum_ransac_inliers = 0;
    let mut ransac_iterations = 0;
    let mut candidates = Vec::new();

    for (voxel_size, target_features) in config
        .voxel_sizes
        .into_iter()
        .zip(target.global_scales(config))
    {
        let feature_config = FeatureConfig::for_voxel(voxel_size, config.max_feature_neighbors);
        let source_features = preprocess(fine_source, feature_config);
        if source_features.descriptors.len() != source_features.points.len()
            || target_features.descriptors.len() != target_features.points.len()
        {
            continue;
        }

        let feature_matches = match_fpfh_indexed(
            &source_features.descriptors,
            &target_features.descriptors,
            &source_features.feature_index,
            &target_features.feature_index,
        );
        maximum_feature_matches = maximum_feature_matches.max(feature_matches.len());
        if feature_matches.len() < config.min_feature_matches {
            continue;
        }

        let ransac = ransac_align(
            &source_features.points,
            &target_features.points,
            &target_features.index,
            &feature_matches,
            RansacConfig {
                max_iterations: config.ransac_iterations,
                inlier_threshold: voxel_size * 1.5,
                min_inliers: 3,
                min_inlier_ratio: config.min_final_fitness.min(0.5),
                ..RansacConfig::default()
            },
        );
        ransac_iterations += ransac.iterations;
        maximum_ransac_inliers = maximum_ransac_inliers.max(ransac.inlier_count);
        if !ransac.converged {
            continue;
        }

        consider_candidate(source, target, ransac.transformation, &mut candidates);
        consider_candidate(
            source,
            target,
            ransac.transformation * centroid_yaw_flip(&fine_source),
            &mut candidates,
        );
    }

    finish_global_candidates(
        source,
        target,
        config,
        candidates,
        maximum_feature_matches,
        maximum_ransac_inliers,
        ransac_iterations,
    )
}

fn different_positions(a: &Isometry3<f64>, b: &Isometry3<f64>) -> bool {
    (a.translation.vector - b.translation.vector).norm() > 1.0
        || (a.rotation.inverse() * b.rotation).angle().to_degrees() > 20.0
}

fn sort_candidates(candidates: &mut [(Isometry3<f64>, RelocalizationScore)]) {
    candidates.sort_by(|a, b| {
        b.1.wall_fitness
            .total_cmp(&a.1.wall_fitness)
            .then(a.1.penetration.total_cmp(&b.1.penetration))
    });
}

fn finish_global_candidates(
    source: &SourceCloud,
    target: &TargetCache,
    config: RelocalizationConfig,
    mut candidates: Vec<(Isometry3<f64>, RelocalizationScore)>,
    feature_matches: usize,
    ransac_inliers: usize,
    mut iterations: usize,
) -> RelocalizationResult {
    sort_candidates(&mut candidates);
    let Some(first) = candidates.first().copied() else {
        return rejected_relocalization(feature_matches, ransac_inliers, iterations);
    };
    let second = candidates
        .iter()
        .skip(1)
        .find(|candidate| different_positions(&first.0, &candidate.0))
        .copied();
    let mut refined = Vec::new();
    for (candidate, _) in std::iter::once(first).chain(second) {
        let seeded: Vec<_> = source.points.iter().map(|p| candidate * p).collect();
        let mut icp = config.icp;
        icp.max_correspondence_distance = icp
            .max_correspondence_distance
            .min(config.fine_correspondence_distance);
        let refinement = IcpSolver::new(icp).align_indexed(&seeded, &target.points, &target.index);
        iterations += refinement.iterations;
        let transform = refinement.transformation * candidate;
        refined.push((transform, score_relocalization(source, transform, target)));
    }
    sort_candidates(&mut refined);
    let (transformation, score) = refined[0];
    if !score.accepts(&transformation) {
        let mut result = rejected_relocalization(feature_matches, ransac_inliers, iterations);
        result.fitness = score.wall_fitness;
        return result;
    }
    if refined.get(1).is_some_and(|(second, runner_up)| {
        different_positions(&transformation, second)
            && runner_up.accepts(second)
            && runner_up.wall_fitness >= score.wall_fitness * 0.9
    }) {
        let mut result = rejected_relocalization(feature_matches, ransac_inliers, iterations);
        result.ambiguous = true;
        result.fitness = score.wall_fitness;
        return result;
    }
    RelocalizationResult {
        registration: IcpResult {
            transformation,
            mse: score.rmse * score.rmse,
            iterations,
            converged: true,
        },
        feature_matches,
        ransac_inliers,
        fitness: score.wall_fitness,
        ambiguous: false,
    }
}

/// Run relocalization with parity-oriented defaults.
#[must_use]
pub fn relocation_pipeline(source: &[Point3<f64>], target: &[Point3<f64>]) -> IcpResult {
    relocation_pipeline_with_config(source, target, RelocalizationConfig::default()).registration
}

fn valid_relocalization_config(config: RelocalizationConfig) -> bool {
    config
        .voxel_sizes
        .iter()
        .all(|size| size.is_finite() && *size > 0.0)
        && config.max_feature_neighbors >= 3
        && config.ransac_iterations > 0
        && config.min_feature_matches >= 3
        && config.fine_voxel_size.is_finite()
        && config.fine_voxel_size > 0.0
        && config.fine_correspondence_distance.is_finite()
        && config.fine_correspondence_distance > 0.0
        && config.min_final_fitness.is_finite()
        && (0.0..=1.0).contains(&config.min_final_fitness)
        && config.icp.max_iterations > 0
        && config.icp.mse_tolerance.is_finite()
        && config.icp.mse_tolerance >= 0.0
        && config.icp.max_correspondence_distance.is_finite()
        && config.icp.max_correspondence_distance > 0.0
}

fn consider_candidate(
    source: &SourceCloud,
    target: &TargetCache,
    transformation: Isometry3<f64>,
    candidates: &mut Vec<(Isometry3<f64>, RelocalizationScore)>,
) {
    candidates.push((
        transformation,
        score_relocalization(source, transformation, target),
    ));
}

fn centroid_yaw_flip(source: &[Point3<f64>]) -> Isometry3<f64> {
    let centroid = source
        .iter()
        .fold(Vector3::zeros(), |sum, point| sum + point.coords)
        / source.len() as f64;
    let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), core::f64::consts::PI);
    let flat_centroid = Vector3::new(centroid.x, centroid.y, 0.0);
    let translation = flat_centroid - rotation * flat_centroid;
    Isometry3::from_parts(Translation3::from(translation), rotation)
}

fn rejected_relocalization(
    feature_matches: usize,
    ransac_inliers: usize,
    iterations: usize,
) -> RelocalizationResult {
    RelocalizationResult {
        registration: IcpResult {
            transformation: Isometry3::identity(),
            mse: f64::INFINITY,
            iterations,
            converged: false,
        },
        feature_matches,
        ransac_inliers,
        fitness: 0.0,
        ambiguous: false,
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
        assert!(
            (tilt - 90.0).abs() < 1e-6,
            "y-90° rotation tilt must be 90°, got {tilt}"
        );
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
            vector![1.0, 0.0, 0.0],  // 墙面 → |n.z|=0 < 0.7 → keep
            vector![0.0, 1.0, 0.0],  // 墙面 → keep
            vector![0.0, 0.0, 1.0],  // 天花板 → |n.z|=1 → drop
            vector![0.0, 0.0, -1.0], // 地板 → |n.z|=1 → drop
            vector![0.6, 0.6, 0.6],  // |z|=0.6 < 0.7 → keep（边角墙面）
            vector![0.5, 0.5, 0.8],  // |z|=0.8 > 0.7 → drop（接近地板）
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
    #[test]
    fn adjacent_identical_rooms_are_globally_ambiguous() {
        let points = synthetic_room();
        let source = SourceCloud::from_rays(points.iter().map(|p| (*p, Point3::origin())));
        let a = Isometry3::translation(30.0, 20.0, 0.0);
        let b = Isometry3::translation(42.0, 20.0, 0.0);
        let target: Vec<_> = points.iter().flat_map(|p| [a * p, b * p]).collect();
        let config = RelocalizationConfig::default();
        let target = TargetCache::new(&target, config);
        let result = relocation_pipeline_cached(&source, &target, config);
        assert!(
            !result.registration.converged && result.ambiguous,
            "相同房间须报告歧义：{result:?}"
        );
        // 同一候选附近的重复结果不能挤掉另一个房间。
        let candidates = [a, a, b]
            .into_iter()
            .map(|p| (p, score_relocalization(&source, p, &target)))
            .collect();
        let result = finish_global_candidates(&source, &target, config, candidates, 0, 0, 0);
        assert!(result.ambiguous, "两个平移不同的房间须进入歧义出口");
        let single = vec![(a, score_relocalization(&source, a, &target))];
        assert!(
            finish_global_candidates(&source, &target, config, single, 0, 0, 0)
                .registration
                .converged
        );
        assert!(!different_positions(
            &a,
            &(a * Isometry3::translation(1.0, 0.0, 0.0))
        ));
        assert!(different_positions(
            &a,
            &(a * Isometry3::translation(1.01, 0.0, 0.0))
        ));
        assert!(different_positions(
            &a,
            &(a * Isometry3::from_parts(
                Translation3::identity(),
                UnitQuaternion::from_euler_angles(0.0, 0.0, 21_f64.to_radians())
            ))
        ));
    }

    #[test]
    fn same_floor_and_ceiling_cannot_accept_a_room_with_different_walls() {
        let points = synthetic_room();
        let source = SourceCloud::from_rays(points.iter().map(|p| (*p, Point3::origin())));
        let truth = Isometry3::translation(30.0, 20.0, 0.0);
        let mut target: Vec<_> = points.iter().map(|p| truth * p).collect();
        // 甲房间保留相同地面和天花板，墙面布局扩大，乙房间为实际观测。
        target.extend(
            points
                .iter()
                .filter(|p| p.z == -1.0 || p.z == 1.73)
                .copied(),
        );
        target.extend(
            points
                .iter()
                .filter(|p| p.z != -1.0 && p.z != 1.73)
                .map(|p| Point3::new(p.x * 2.0, p.y * 2.0, p.z)),
        );
        let target = TargetCache::new(&target, RelocalizationConfig::default());
        let wrong = score_relocalization(&source, Isometry3::identity(), &target);
        let correct = score_relocalization(&source, truth, &target);
        assert!(
            wrong.wall_fitness < 0.45,
            "错误房间不能靠水平面通过：{wrong:?}"
        );
        assert!(!wrong.accepts(&Isometry3::identity()));
        assert!(correct.accepts(&truth), "乙房间真值应通过：{correct:?}");
    }

    #[test]
    fn penetration_uses_each_frames_origin_and_excludes_both_ray_ends() {
        let points: Vec<_> = (-12..=12)
            .flat_map(|y| {
                (-12..=12)
                    .map(move |z| Point3::new(3.05, (y as f64 + 0.5) * 0.1, (z as f64 + 0.5) * 0.1))
            })
            .collect();
        let mut target = points.clone();
        target.extend(points.iter().map(|p| Point3::new(1.55, p.y, p.z)));
        let target = TargetCache::new(&target, RelocalizationConfig::default());
        let through = SourceCloud::from_rays(points.iter().map(|p| (*p, Point3::origin())));
        let score = score_relocalization(&through, Isometry3::identity(), &target);
        assert_eq!(score.wall_fitness, 1.0);
        assert!(score.penetration > 0.9, "完全贴墙仍须排除穿墙：{score:?}");
        assert!(!score.accepts(&Isometry3::identity()));
        let clear = SourceCloud::from_rays(points.iter().map(|p| (*p, Point3::new(2.0, p.y, p.z))));
        let score = score_relocalization(&clear, Isometry3::identity(), &target);
        assert!(
            score.accepts(&Isometry3::identity()),
            "帧原点改变后不经过内墙：{score:?}"
        );
        // 占据点只在0.5米起点排除段或0.3米终点排除段内。
        let mut endpoints = points.clone();
        endpoints.extend(points.iter().flat_map(|p| {
            [
                Point3::from(p.coords.normalize() * 0.2),
                p - p.coords.normalize() * 0.1,
            ]
        }));
        let endpoints = TargetCache::new(&endpoints, RelocalizationConfig::default());
        let score = score_relocalization(&through, Isometry3::identity(), &endpoints);
        assert_eq!(score.penetration, 0.0, "两端排除段不能计为穿透：{score:?}");
    }

    #[test]
    fn source_downsampling_preserves_the_selected_frames_ray_origin() {
        let a = Point3::new(0.01, 0.02, 0.03);
        let b = Point3::new(0.08, 0.07, 0.06);
        let c = Point3::new(0.15, 0.02, 0.03);
        let source = SourceCloud::from_rays([
            (a, Point3::new(1.0, 2.0, 3.0)),
            (b, Point3::new(4.0, 5.0, 6.0)),
            (c, Point3::new(7.0, 8.0, 9.0)),
        ]);
        assert_eq!(source.points, vec![a, c]);
        assert_eq!(
            source.origins,
            vec![Point3::new(1.0, 2.0, 3.0), Point3::new(7.0, 8.0, 9.0)]
        );
    }

    pub(crate) fn synthetic_room() -> Vec<Point3<f64>> {
        let mut points = Vec::new();
        for x in 0..47 {
            for y in 0..32 {
                let (x, y) = (-3.0 + x as f64 * 0.13, -2.0 + y as f64 * 0.13);
                points.extend([Point3::new(x, y, -1.0), Point3::new(x, y, 1.73)]);
            }
        }
        for z in 0..22 {
            let z = -1.0 + z as f64 * 0.13;
            for x in 0..47 {
                let x = -3.0 + x as f64 * 0.13;
                points.extend([Point3::new(x, -2.0, z), Point3::new(x, 2.03, z)]);
            }
            for y in 0..32 {
                let y = -2.0 + y as f64 * 0.13;
                points.extend([Point3::new(-3.0, y, z), Point3::new(2.98, y, z)]);
            }
        }
        points
    }

    #[test]
    fn local_registration_leaves_global_features_unbuilt() {
        let points = synthetic_room();
        let config = RelocalizationConfig::default();
        let target = TargetCache::new(&points, config);
        let source = SourceCloud::from_rays(points.iter().map(|p| (*p, Point3::origin())));
        assert!(target.scales.get().is_none());
        for _ in 0..2 {
            let local = local_registration(
                &source,
                &target,
                Isometry3::identity(),
                Isometry3::identity(),
            )
            .unwrap();
            assert!(score_relocalization(&source, local.transformation, &target)
                .accepts(&local.transformation));
            assert!(target.scales.get().is_none());
        }
        let started = std::time::Instant::now();
        let _ = relocation_pipeline_cached(&source, &target, config);
        let first = target.scales.get().unwrap();
        let _ = relocation_pipeline_cached(&source, &target, config);
        assert!(std::ptr::eq(first, target.scales.get().unwrap()));
        assert_eq!(first.len(), 3);
        eprintln!(
            "全局首次使用后缓存三个尺度，重复调用复用，验证耗时={:?}",
            started.elapsed()
        );
    }

    #[test]
    fn target_cache_builds_and_timing() {
        use super::super::{features::descriptor_builds, nearest::builds};
        let source = synthetic_room();
        let truth = Isometry3::translation(30.0, 20.0, 0.0);
        let mut target: Vec<_> = source.iter().map(|p| truth * p).collect();
        target.extend(
            source
                .iter()
                .map(|p| Point3::new(p.x * 0.7 + 42.0, p.y * 1.3 + 20.0, p.z)),
        );
        let config = RelocalizationConfig::default();
        let mut watched = vec![("精细点".into(), voxel_downsample(&target, 0.1))];
        let mut descriptors = Vec::new();
        for scale in config.voxel_sizes {
            let cloud = preprocess(
                &target,
                FeatureConfig::for_voxel(scale, config.max_feature_neighbors),
            );
            watched.push((format!("尺度{scale}"), cloud.points));
            descriptors.push((format!("特征{scale}"), cloud.descriptors));
        }
        builds::watch(watched);
        descriptor_builds::watch(descriptors);
        let started = std::time::Instant::now();
        let target = TargetCache::new(&target, config);
        let created = started.elapsed();
        assert_eq!(
            builds::counts(),
            vec![
                ("精细点".into(), 1),
                ("尺度0.2".into(), 0),
                ("尺度0.3".into(), 0),
                ("尺度0.8".into(), 0)
            ]
        );
        assert!(descriptor_builds::counts().iter().all(|(_, n)| *n == 0));
        let source_cloud = SourceCloud::from_rays(source.iter().map(|p| (*p, Point3::origin())));
        let result = relocation_pipeline_cached(&source_cloud, &target, config);
        let cold = started.elapsed();
        assert!(builds::counts().iter().all(|(_, n)| *n == 1));
        assert!(descriptor_builds::counts().iter().all(|(_, n)| *n == 1));
        let started = std::time::Instant::now();
        let source_cloud = SourceCloud::from_rays(source.iter().map(|p| (*p, Point3::origin())));
        let _ = relocation_pipeline_cached(&source_cloud, &target, config);
        eprintln!(
            "源点={} 目标点={} 精细缓存创建={:?} 首次总耗时={:?} 复用总耗时={:?} 结果={} 点索引={:?} 特征索引={:?}",
            source.len(),
            12968,
            created,
            cold,
            started.elapsed(),
            result.registration.converged,
            builds::counts(),
            descriptor_builds::counts()
        );
        assert!(builds::counts().iter().all(|(_, n)| *n == 1));
        assert!(descriptor_builds::counts().iter().all(|(_, n)| *n == 1));
        builds::watch(Vec::new());
        descriptor_builds::watch(Vec::new());
    }
}
