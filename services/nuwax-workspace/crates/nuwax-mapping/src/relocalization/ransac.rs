use super::nearest::NearestPointIndex;
use nalgebra::{Isometry3, Matrix3, Point3, Translation3, UnitQuaternion, Vector3};

use super::features::FeatureMatch;

/// Deterministic geometric RANSAC configuration.
#[derive(Debug, Clone, Copy)]
pub struct RansacConfig {
    /// Maximum number of sampled correspondence triplets.
    pub max_iterations: usize,
    /// Desired probability that at least one all-inlier triplet is sampled.
    pub confidence: f64,
    /// Maximum source-to-target residual for an inlier in metres.
    pub inlier_threshold: f64,
    /// Minimum preserved edge-length ratio for a sampled triangle.
    pub edge_length_ratio: f64,
    /// Minimum number of geometric inliers required for acceptance.
    pub min_inliers: usize,
    /// Minimum geometric inlier ratio required for acceptance.
    pub min_inlier_ratio: f64,
}

impl Default for RansacConfig {
    fn default() -> Self {
        Self {
            max_iterations: 4_096,
            confidence: 0.995,
            inlier_threshold: 0.3,
            edge_length_ratio: 0.9,
            min_inliers: 3,
            min_inlier_ratio: 0.2,
        }
    }
}

/// Result of deterministic feature-correspondence RANSAC.
#[derive(Debug, Clone, Copy)]
pub struct RansacResult {
    /// Estimated source-to-target rigid transformation.
    pub transformation: Isometry3<f64>,
    /// Number of source points agreeing with the target cloud.
    pub inlier_count: usize,
    /// Fraction of source points agreeing with the target cloud.
    pub inlier_ratio: f64,
    /// Root-mean-square residual of the inliers in metres.
    pub rmse: f64,
    /// Number of triplets evaluated.
    pub iterations: usize,
    /// Whether the configured consensus gates were satisfied.
    pub converged: bool,
}

/// 已计算变换的诊断指标，由调用方决定如何呈现。
#[derive(Debug, Clone, Copy)]
pub struct AlignmentScore {
    /// 对应距离内的源点数。
    pub inlier_count: usize,
    /// 有对应关系的源点比例。
    pub fitness: f64,
    /// 对应点均方根误差。
    pub rmse: f64,
}

/// Estimate a rigid transform from feature matches using deterministic 3-point RANSAC.
#[must_use]
pub fn ransac_align(
    source: &[Point3<f64>],
    target: &[Point3<f64>],
    matches: &[FeatureMatch],
    config: RansacConfig,
) -> RansacResult {
    if source.len() < 3
        || target.len() < 3
        || matches.len() < 3
        || !finite_points(source)
        || !finite_points(target)
        || config.max_iterations == 0
        || !valid_config(config)
    {
        return failed_result(0);
    }

    let target_tree = point_tree(target);
    let mut generator = DeterministicGenerator::new(
        0x9e37_79b9_7f4a_7c15 ^ source.len() as u64 ^ ((target.len() as u64) << 32),
    );
    let mut best: Option<(Isometry3<f64>, AlignmentScore)> = None;
    let mut iterations = 0;

    let mut iteration_limit = config.max_iterations;
    for iteration in 0..config.max_iterations {
        if iteration >= iteration_limit {
            break;
        }
        let Some(sample) = sample_triplet(matches.len(), iteration, &mut generator) else {
            break;
        };
        iterations += 1;
        let sampled_matches = [matches[sample[0]], matches[sample[1]], matches[sample[2]]];
        if sampled_matches.iter().any(|feature_match| {
            feature_match.source_index >= source.len() || feature_match.target_index >= target.len()
        }) {
            continue;
        }

        let pairs = sampled_matches.map(|feature_match| {
            (
                source[feature_match.source_index],
                target[feature_match.target_index],
            )
        });
        if !triangle_is_valid(&pairs, config.edge_length_ratio) {
            continue;
        }

        let transform = best_fit(&pairs);
        let score = evaluate_with_tree(source, &target_tree, transform, config.inlier_threshold);
        if better_score(score, best.map(|(_, current)| current)) {
            best = Some((transform, score));
            iteration_limit = iteration_limit.min(required_iterations(
                score.fitness,
                config.confidence,
                config.max_iterations,
            ));
        }
    }

    let Some((best_transform, _)) = best else {
        return failed_result(iterations);
    };

    let consensus_pairs: Vec<_> = matches
        .iter()
        .filter(|feature_match| {
            feature_match.source_index < source.len() && feature_match.target_index < target.len()
        })
        .filter_map(|feature_match| {
            let source_point = source[feature_match.source_index];
            let target_point = target[feature_match.target_index];
            ((target_point - best_transform * source_point).norm() <= config.inlier_threshold)
                .then_some((source_point, target_point))
        })
        .collect();
    let refined = if consensus_pairs.len() >= 3 {
        best_fit(&consensus_pairs)
    } else {
        best_transform
    };
    let score = evaluate_with_tree(source, &target_tree, refined, config.inlier_threshold);
    let converged = score.inlier_count >= config.min_inliers
        && score.fitness >= config.min_inlier_ratio
        && score.rmse.is_finite();

    RansacResult {
        transformation: refined,
        inlier_count: score.inlier_count,
        inlier_ratio: score.fitness,
        rmse: score.rmse,
        iterations,
        converged,
    }
}

fn finite_points(points: &[Point3<f64>]) -> bool {
    points
        .iter()
        .all(|point| point.coords.iter().all(|value| value.is_finite()))
}

/// 计算现有变换的对应关系诊断，不负责结果提交。
pub fn evaluate_alignment(
    source: &[Point3<f64>],
    target: &[Point3<f64>],
    transform: Isometry3<f64>,
    threshold: f64,
) -> AlignmentScore {
    if source.is_empty() || target.is_empty() || !threshold.is_finite() || threshold <= 0.0 {
        return AlignmentScore {
            inlier_count: 0,
            fitness: 0.0,
            rmse: f64::INFINITY,
        };
    }
    evaluate_with_tree(source, &point_tree(target), transform, threshold)
}

fn valid_config(config: RansacConfig) -> bool {
    config.inlier_threshold.is_finite()
        && config.inlier_threshold > 0.0
        && config.edge_length_ratio.is_finite()
        && config.edge_length_ratio > 0.0
        && config.edge_length_ratio <= 1.0
        && config.confidence.is_finite()
        && config.confidence > 0.0
        && config.confidence < 1.0
        && config.min_inlier_ratio.is_finite()
        && (0.0..=1.0).contains(&config.min_inlier_ratio)
}

fn required_iterations(inlier_ratio: f64, confidence: f64, maximum: usize) -> usize {
    if inlier_ratio >= 1.0 {
        return 1;
    }
    if inlier_ratio <= 0.0 {
        return maximum;
    }
    let miss_probability = 1.0 - inlier_ratio.powi(3);
    if miss_probability <= 0.0 || miss_probability >= 1.0 {
        return maximum;
    }
    ((1.0 - confidence).ln() / miss_probability.ln())
        .ceil()
        .max(1.0) as usize
}

fn failed_result(iterations: usize) -> RansacResult {
    RansacResult {
        transformation: Isometry3::identity(),
        inlier_count: 0,
        inlier_ratio: 0.0,
        rmse: f64::INFINITY,
        iterations,
        converged: false,
    }
}

fn point_tree(points: &[Point3<f64>]) -> NearestPointIndex {
    NearestPointIndex::new(points)
}

fn evaluate_with_tree(
    source: &[Point3<f64>],
    target_tree: &NearestPointIndex,
    transform: Isometry3<f64>,
    threshold: f64,
) -> AlignmentScore {
    let threshold_sq = threshold * threshold;
    let mut inlier_count = 0;
    let mut squared_error = 0.0;
    for point in source {
        let transformed = transform * point;
        let Some((_,distance))=target_tree.nearest(&transformed) else {continue;};
        if distance <= threshold_sq {
            inlier_count += 1;
            squared_error += distance;
        }
    }
    AlignmentScore {
        inlier_count,
        fitness: inlier_count as f64 / source.len().max(1) as f64,
        rmse: if inlier_count == 0 {
            f64::INFINITY
        } else {
            (squared_error / inlier_count as f64).sqrt()
        },
    }
}

fn better_score(candidate: AlignmentScore, current: Option<AlignmentScore>) -> bool {
    let Some(current) = current else {
        return true;
    };
    candidate.inlier_count > current.inlier_count
        || (candidate.inlier_count == current.inlier_count && candidate.rmse < current.rmse)
}

fn triangle_is_valid(pairs: &[(Point3<f64>, Point3<f64>); 3], minimum_edge_ratio: f64) -> bool {
    let source_area = (pairs[1].0 - pairs[0].0)
        .cross(&(pairs[2].0 - pairs[0].0))
        .norm();
    let target_area = (pairs[1].1 - pairs[0].1)
        .cross(&(pairs[2].1 - pairs[0].1))
        .norm();
    if source_area <= 1e-8 || target_area <= 1e-8 {
        return false;
    }

    [(0, 1), (0, 2), (1, 2)].into_iter().all(|(left, right)| {
        let source_length = (pairs[left].0 - pairs[right].0).norm();
        let target_length = (pairs[left].1 - pairs[right].1).norm();
        if source_length <= 1e-9 || target_length <= 1e-9 {
            return false;
        }
        let ratio = source_length.min(target_length) / source_length.max(target_length);
        ratio >= minimum_edge_ratio
    })
}

fn best_fit(correspondences: &[(Point3<f64>, Point3<f64>)]) -> Isometry3<f64> {
    let count = correspondences.len() as f64;
    let (source_mean, target_mean) = correspondences.iter().fold(
        (Vector3::zeros(), Vector3::zeros()),
        |(source_sum, target_sum), (source, target)| {
            (source_sum + source.coords, target_sum + target.coords)
        },
    );
    let source_mean = source_mean / count;
    let target_mean = target_mean / count;
    let covariance = correspondences
        .iter()
        .fold(Matrix3::zeros(), |sum, (source, target)| {
            sum + (source.coords - source_mean) * (target.coords - target_mean).transpose()
        });
    let svd = nalgebra::linalg::SVD::new(covariance, true, true);
    let u = svd.u.unwrap_or_else(Matrix3::identity);
    let v_t = svd.v_t.unwrap_or_else(Matrix3::identity);
    let mut reflection = Matrix3::identity();
    if (v_t.transpose() * u.transpose()).determinant() < 0.0 {
        reflection[(2, 2)] = -1.0;
    }
    let rotation_matrix = v_t.transpose() * reflection * u.transpose();
    let rotation = UnitQuaternion::from_matrix(&rotation_matrix);
    let translation = target_mean - rotation * source_mean;
    Isometry3::from_parts(Translation3::from(translation), rotation)
}

fn sample_triplet(
    match_count: usize,
    iteration: usize,
    generator: &mut DeterministicGenerator,
) -> Option<[usize; 3]> {
    if match_count < 3 {
        return None;
    }
    if iteration == 0 {
        return Some([0, 1, 2]);
    }
    for _ in 0..32 {
        let first = generator.index(match_count);
        let second = generator.index(match_count);
        let third = generator.index(match_count);
        if first != second && first != third && second != third {
            return Some([first, second, third]);
        }
    }
    None
}

struct DeterministicGenerator {
    state: u64,
}

impl DeterministicGenerator {
    fn new(seed: u64) -> Self {
        Self { state: seed.max(1) }
    }

    fn index(&mut self, upper_bound: usize) -> usize {
        let mut value = self.state;
        value ^= value << 13;
        value ^= value >> 7;
        value ^= value << 17;
        self.state = value;
        value as usize % upper_bound
    }
}
