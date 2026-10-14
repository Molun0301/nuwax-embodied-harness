//! 回环匹配证据：完整扫描的双向覆盖与实际对应点的几何支撑。

use std::collections::BTreeSet;

use crate::relocalization::nearest::NearestPointIndex;
use nalgebra::{Isometry3, Matrix3, Point3, Vector3};

/// 第二主轴方差至少为第一主轴的千分之一，避免直线或近直线对应任意确定绕轴旋转。
const MIN_AXIS_SPREAD_RATIO: f64 = 1e-3;

pub(super) struct LoopRegistrationEvidence {
    pub source_inliers: usize,
    pub target_inliers: usize,
    pub unique_target_inliers: usize,
    pub source_overlap: f64,
    pub target_overlap: f64,
    pub mse: f64,
    pub source_spread: f64,
    pub target_spread: f64,
    pub lever_arm_m2: f64,
}

impl LoopRegistrationEvidence {
    pub fn rejection_reason(
        &self,
        min_points: usize,
        min_overlap: f64,
        max_mse: f64,
    ) -> Option<&'static str> {
        if self.source_inliers < min_points
            || self.target_inliers < min_points
            || self.unique_target_inliers < min_points
        {
            Some("insufficient_correspondences")
        } else if self.source_overlap < min_overlap || self.target_overlap < min_overlap {
            Some("insufficient_overlap")
        } else if !self.mse.is_finite() || self.mse > max_mse {
            Some("alignment_error")
        } else if self.source_spread < MIN_AXIS_SPREAD_RATIO
            || self.target_spread < MIN_AXIS_SPREAD_RATIO
        {
            Some("degenerate_correspondences")
        } else {
            None
        }
    }
}

pub(super) fn evaluate_loop_registration(
    source: &[Point3<f64>],
    target: &[Point3<f64>],
    transform: &Isometry3<f64>,
    max_correspondence_distance: f64,
) -> LoopRegistrationEvidence {
    let transformed: Vec<Point3<f64>> = source.iter().map(|point| transform * point).collect();
    let target_tree = NearestPointIndex::new(target);
    let source_tree = NearestPointIndex::new(&transformed);
    let max_distance_squared = max_correspondence_distance.powi(2);
    let mut source_support = Vec::new();
    let mut matched_targets = BTreeSet::new();
    let mut squared_error = 0.0;
    for point in &transformed {
        if let Some((index, distance_squared)) = target_tree.nearest(point) {
            if distance_squared <= max_distance_squared {
                source_support.push(*point);
                matched_targets.insert(index);
                squared_error += distance_squared;
            }
        }
    }
    let mut target_inliers = 0;
    for point in target {
        if let Some((_, distance_squared)) = source_tree.nearest(point) {
            if distance_squared <= max_distance_squared {
                target_inliers += 1;
                squared_error += distance_squared;
            }
        }
    }
    let target_support: Vec<_> = matched_targets.iter().map(|index| target[*index]).collect();
    let source_inliers = source_support.len();
    let total = source_inliers + target_inliers;
    let (source_spread, source_radius) = axis_spread(&source_support);
    let (target_spread, target_radius) = axis_spread(&target_support);
    LoopRegistrationEvidence {
        source_inliers,
        target_inliers,
        unique_target_inliers: target_support.len(),
        source_overlap: source_inliers as f64 / source.len() as f64,
        target_overlap: target_inliers as f64 / target.len() as f64,
        mse: if total == 0 {
            f64::INFINITY
        } else {
            squared_error / total as f64
        },
        source_spread,
        target_spread,
        lever_arm_m2: source_radius.min(target_radius),
    }
}

fn axis_spread(points: &[Point3<f64>]) -> (f64, f64) {
    if points.len() < 3 {
        return (0.0, 0.0);
    }
    // 先减首点再求质心，避免地图原点较远时大坐标平方的相消误差。
    let origin = points[0];
    let mean = points
        .iter()
        .fold(Vector3::zeros(), |sum, p| sum + (p - origin))
        / points.len() as f64;
    let covariance = points.iter().fold(Matrix3::zeros(), |sum, point| {
        let delta = (point - origin) - mean;
        sum + delta * delta.transpose()
    }) / points.len() as f64;
    if covariance.iter().any(|value| !value.is_finite()) {
        return (0.0, 0.0);
    }
    let mut eigenvalues = covariance.symmetric_eigen().eigenvalues.as_slice().to_vec();
    eigenvalues.sort_by(f64::total_cmp);
    if eigenvalues[2] <= f64::EPSILON {
        return (0.0, 0.0);
    }
    (
        (eigenvalues[1] / eigenvalues[2]).max(0.0),
        covariance.trace().max(0.0),
    )
}
