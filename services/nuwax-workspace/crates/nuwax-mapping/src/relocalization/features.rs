use std::collections::BTreeMap;

use kiddo::KdTree;
use nalgebra::{linalg::SymmetricEigen, Matrix3, Point3, Vector3};

/// Number of bins in each angular component of an FPFH descriptor.
pub const FPFH_BINS: usize = 11;

/// Number of scalar components in an FPFH descriptor.
pub const FPFH_DIM: usize = FPFH_BINS * 3;

/// Fast Point Feature Histogram descriptor.
pub type FpfhDescriptor = [f64; FPFH_DIM];

/// A reciprocal nearest-neighbour match in FPFH descriptor space.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FeatureMatch {
    /// Index of the source-cloud point.
    pub source_index: usize,
    /// Index of the target-cloud point.
    pub target_index: usize,
    /// Squared Euclidean distance between the two descriptors.
    pub descriptor_distance: f64,
}

/// Preprocessing parameters used before feature matching.
#[derive(Debug, Clone, Copy)]
pub struct FeatureConfig {
    /// Voxel edge length in metres.
    pub voxel_size: f64,
    /// Radius used for local covariance normal estimation in metres.
    pub normal_radius: f64,
    /// Radius used for SPFH/FPFH neighbourhoods in metres.
    pub feature_radius: f64,
    /// Maximum number of neighbours considered per point.
    pub max_neighbors: usize,
}

impl FeatureConfig {
    pub(crate) fn for_voxel(voxel_size: f64, max_neighbors: usize) -> Self {
        Self {
            voxel_size,
            normal_radius: voxel_size * 2.0,
            feature_radius: voxel_size * 5.0,
            max_neighbors,
        }
    }
}

impl Default for FeatureConfig {
    fn default() -> Self {
        Self::for_voxel(0.2, 100)
    }
}

#[derive(Debug, Clone)]
pub(crate) struct ProcessedCloud {
    pub(crate) points: Vec<Point3<f64>>,
    pub(crate) descriptors: Vec<FpfhDescriptor>,
}

/// Deterministically voxel-downsample a cloud by averaging points in each cell.
#[must_use]
pub fn voxel_downsample(points: &[Point3<f64>], voxel_size: f64) -> Vec<Point3<f64>> {
    if !voxel_size.is_finite() || voxel_size <= 0.0 {
        return Vec::new();
    }

    let finite_points: Vec<&Point3<f64>> = points
        .iter()
        .filter(|point| point.coords.iter().all(|value| value.is_finite()))
        .collect();
    if finite_points.is_empty() {
        return Vec::new();
    }
    // Open3D anchors voxel_down_sample at half a voxel below the cloud's
    // minimum bound, rather than at the world origin.
    let minimum_bound = finite_points.iter().fold(
        Vector3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
        |minimum, point| minimum.zip_map(&point.coords, f64::min),
    ) - Vector3::repeat(voxel_size * 0.5);

    let mut voxels: BTreeMap<(i64, i64, i64), (Vector3<f64>, usize)> = BTreeMap::new();
    for point in finite_points {
        let key = (
            ((point.x - minimum_bound.x) / voxel_size).floor() as i64,
            ((point.y - minimum_bound.y) / voxel_size).floor() as i64,
            ((point.z - minimum_bound.z) / voxel_size).floor() as i64,
        );
        let entry = voxels.entry(key).or_insert((Vector3::zeros(), 0));
        entry.0 += point.coords;
        entry.1 += 1;
    }

    voxels
        .into_values()
        .map(|(sum, count)| Point3::from(sum / count as f64))
        .collect()
}

/// Estimate one PCA normal per point using a bounded nearest-neighbour search.
#[must_use]
pub fn estimate_normals(
    points: &[Point3<f64>],
    radius: f64,
    max_neighbors: usize,
) -> Vec<Vector3<f64>> {
    if points.len() < 3
        || points
            .iter()
            .any(|point| !point.coords.iter().all(|value| value.is_finite()))
        || !radius.is_finite()
        || radius <= 0.0
        || max_neighbors < 3
    {
        return Vec::new();
    }

    let tree = point_tree(points);
    let radius_sq = radius * radius;
    let query_count = max_neighbors.min(points.len()).max(3);

    points
        .iter()
        .enumerate()
        .map(|(point_index, point)| {
            let nearest = tree
                .nearest_n::<kiddo::SquaredEuclidean>(&[point.x, point.y, point.z], query_count);
            let mut neighbours: Vec<usize> = nearest
                .iter()
                .filter(|neighbour| {
                    neighbour.item as usize != point_index && neighbour.distance <= radius_sq
                })
                .map(|neighbour| neighbour.item as usize)
                .collect();
            if neighbours.len() < 2 {
                neighbours = nearest
                    .iter()
                    .filter(|neighbour| neighbour.item as usize != point_index)
                    .take(2)
                    .map(|neighbour| neighbour.item as usize)
                    .collect();
            }

            let mut local_points = Vec::with_capacity(neighbours.len() + 1);
            local_points.push(*point);
            local_points.extend(neighbours.into_iter().map(|index| points[index]));
            pca_normal(&local_points)
        })
        .collect()
}

/// Compute one 33-component FPFH descriptor per point.
#[must_use]
pub fn compute_fpfh(
    points: &[Point3<f64>],
    normals: &[Vector3<f64>],
    radius: f64,
    max_neighbors: usize,
) -> Vec<FpfhDescriptor> {
    if points.len() < 3
        || points.len() != normals.len()
        || points
            .iter()
            .any(|point| !point.coords.iter().all(|value| value.is_finite()))
        || normals
            .iter()
            .any(|normal| !normal.iter().all(|value| value.is_finite()))
        || !radius.is_finite()
        || radius <= 0.0
        || max_neighbors < 3
    {
        return Vec::new();
    }

    let tree = point_tree(points);
    let radius_sq = radius * radius;
    let query_count = max_neighbors.min(points.len()).max(3);
    let neighbours: Vec<Vec<(usize, f64)>> = points
        .iter()
        .enumerate()
        .map(|(point_index, point)| {
            let nearest = tree
                .nearest_n::<kiddo::SquaredEuclidean>(&[point.x, point.y, point.z], query_count);
            let mut within: Vec<(usize, f64)> = nearest
                .iter()
                .filter(|neighbour| {
                    neighbour.item as usize != point_index && neighbour.distance <= radius_sq
                })
                .map(|neighbour| (neighbour.item as usize, neighbour.distance.sqrt()))
                .collect();
            if within.len() < 2 {
                within = nearest
                    .iter()
                    .filter(|neighbour| neighbour.item as usize != point_index)
                    .take(2)
                    .map(|neighbour| (neighbour.item as usize, neighbour.distance.sqrt()))
                    .collect();
            }
            within
        })
        .collect();

    let spfh: Vec<FpfhDescriptor> = (0..points.len())
        .map(|index| spfh_descriptor(index, points, normals, &neighbours[index]))
        .collect();

    (0..points.len())
        .map(|index| {
            let mut descriptor = spfh[index];
            let mut weight_sum = 1.0;
            for &(neighbour_index, distance) in &neighbours[index] {
                let weight = 1.0 / distance.max(1e-9);
                for (component, neighbour_component) in
                    descriptor.iter_mut().zip(&spfh[neighbour_index])
                {
                    *component += neighbour_component * weight;
                }
                weight_sum += weight;
            }
            for component in &mut descriptor {
                *component /= weight_sum;
            }
            normalize_histograms(&mut descriptor);
            descriptor
        })
        .collect()
}

/// Match two FPFH sets with reciprocal nearest-neighbour filtering.
#[must_use]
pub fn match_fpfh(source: &[FpfhDescriptor], target: &[FpfhDescriptor]) -> Vec<FeatureMatch> {
    if source.is_empty() || target.is_empty() {
        return Vec::new();
    }

    let target_tree = DescriptorTree::new(target);
    let source_tree = DescriptorTree::new(source);
    let source_to_target: Vec<_> = source
        .iter()
        .map(|descriptor| target_tree.nearest(descriptor))
        .collect();
    let target_to_source: Vec<_> = target
        .iter()
        .map(|descriptor| source_tree.nearest(descriptor))
        .collect();

    let mut matches: Vec<FeatureMatch> = source_to_target
        .iter()
        .enumerate()
        .filter_map(|(source_index, nearest)| {
            let target_index = nearest.0;
            (target_to_source[target_index].0 == source_index).then_some(FeatureMatch {
                source_index,
                target_index,
                descriptor_distance: nearest.1,
            })
        })
        .collect();
    matches.sort_by(|left, right| {
        left.descriptor_distance
            .total_cmp(&right.descriptor_distance)
            .then(left.source_index.cmp(&right.source_index))
            .then(left.target_index.cmp(&right.target_index))
    });
    matches
}

pub(crate) fn preprocess(points: &[Point3<f64>], config: FeatureConfig) -> ProcessedCloud {
    let points = voxel_downsample(points, config.voxel_size);
    let normals = estimate_normals(&points, config.normal_radius, config.max_neighbors);
    let descriptors = compute_fpfh(
        &points,
        &normals,
        config.feature_radius,
        config.max_neighbors,
    );
    ProcessedCloud {
        points,
        descriptors,
    }
}

fn point_tree(points: &[Point3<f64>]) -> KdTree<f64, 3> {
    let mut tree = KdTree::new();
    for (index, point) in points.iter().enumerate() {
        tree.add(&[point.x, point.y, point.z], index as u64);
    }
    tree
}

fn pca_normal(points: &[Point3<f64>]) -> Vector3<f64> {
    if points.len() < 3 {
        return Vector3::z();
    }
    let mean = points
        .iter()
        .fold(Vector3::zeros(), |sum, point| sum + point.coords)
        / points.len() as f64;
    let covariance = points.iter().fold(Matrix3::zeros(), |sum, point| {
        let centered = point.coords - mean;
        sum + centered * centered.transpose()
    }) / points.len() as f64;
    let eigen = SymmetricEigen::new(covariance);
    let smallest = (0..3)
        .min_by(|left, right| eigen.eigenvalues[*left].total_cmp(&eigen.eigenvalues[*right]))
        .unwrap_or(0);
    let mut normal = eigen.eigenvectors.column(smallest).into_owned();
    if normal.norm_squared() <= 1e-18 || !normal.iter().all(|value| value.is_finite()) {
        return Vector3::z();
    }
    normal.normalize_mut();

    let orientation = points[0].coords - mean;
    let orientation_dot = normal.dot(&orientation);
    if orientation_dot < -1e-9 || (orientation_dot.abs() <= 1e-9 && canonical_sign(&normal) < 0.0) {
        normal = -normal;
    }
    normal
}

fn canonical_sign(vector: &Vector3<f64>) -> f64 {
    for component in vector.iter() {
        if component.abs() > 1e-9 {
            return component.signum();
        }
    }
    1.0
}

fn spfh_descriptor(
    point_index: usize,
    points: &[Point3<f64>],
    normals: &[Vector3<f64>],
    neighbours: &[(usize, f64)],
) -> FpfhDescriptor {
    let mut histogram = [0.0; FPFH_DIM];
    let point = points[point_index];
    let u = normals[point_index]
        .try_normalize(1e-12)
        .unwrap_or_else(Vector3::z);

    for &(neighbour_index, distance) in neighbours {
        if distance <= 1e-12 {
            continue;
        }
        let direction = (points[neighbour_index] - point) / distance;
        let Some(v) = direction.cross(&u).try_normalize(1e-12) else {
            continue;
        };
        let w = u.cross(&v);
        let mut neighbour_normal = normals[neighbour_index]
            .try_normalize(1e-12)
            .unwrap_or_else(Vector3::z);
        if u.dot(&neighbour_normal) < 0.0 {
            neighbour_normal = -neighbour_normal;
        }

        let alpha = v.dot(&neighbour_normal).clamp(-1.0, 1.0);
        let phi = u.dot(&direction).clamp(-1.0, 1.0);
        let theta = w.dot(&neighbour_normal).atan2(u.dot(&neighbour_normal));
        histogram[bounded_bin(alpha, -1.0, 1.0)] += 1.0;
        histogram[FPFH_BINS + bounded_bin(phi, -1.0, 1.0)] += 1.0;
        histogram
            [FPFH_BINS * 2 + bounded_bin(theta, -core::f64::consts::PI, core::f64::consts::PI)] +=
            1.0;
    }
    normalize_histograms(&mut histogram);
    histogram
}

fn bounded_bin(value: f64, lower: f64, upper: f64) -> usize {
    let unit = ((value - lower) / (upper - lower)).clamp(0.0, 1.0 - f64::EPSILON);
    (unit * FPFH_BINS as f64).floor() as usize
}

fn normalize_histograms(descriptor: &mut FpfhDescriptor) {
    for section in 0..3 {
        let start = section * FPFH_BINS;
        let end = start + FPFH_BINS;
        let sum: f64 = descriptor[start..end].iter().sum();
        if sum > 0.0 {
            for component in &mut descriptor[start..end] {
                *component /= sum;
            }
        }
    }
}

struct DescriptorTree<'a> {
    descriptors: &'a [FpfhDescriptor],
    root: DescriptorNode,
}

impl<'a> DescriptorTree<'a> {
    fn new(descriptors: &'a [FpfhDescriptor]) -> Self {
        let indices: Vec<usize> = (0..descriptors.len()).collect();
        Self {
            descriptors,
            root: DescriptorNode::build(descriptors, indices),
        }
    }

    fn nearest(&self, query: &FpfhDescriptor) -> (usize, f64) {
        let mut best = (usize::MAX, f64::INFINITY);
        self.root.search(self.descriptors, query, &mut best);
        (best.0, best.1 * best.1)
    }
}

enum DescriptorNode {
    Leaf(Vec<usize>),
    Branch {
        pivot: usize,
        radius: f64,
        inner: Box<DescriptorNode>,
        outer: Box<DescriptorNode>,
    },
}

impl DescriptorNode {
    const LEAF_SIZE: usize = 16;

    fn build(descriptors: &[FpfhDescriptor], mut indices: Vec<usize>) -> Self {
        if indices.len() <= Self::LEAF_SIZE {
            return Self::Leaf(indices);
        }

        let pivot = indices.pop().expect("non-empty descriptor node");
        indices.sort_by(|left, right| {
            descriptor_distance(&descriptors[pivot], &descriptors[*left])
                .total_cmp(&descriptor_distance(
                    &descriptors[pivot],
                    &descriptors[*right],
                ))
                .then(left.cmp(right))
        });
        let midpoint = indices.len() / 2;
        let outer_indices = indices.split_off(midpoint);
        let radius = outer_indices.first().map_or(0.0, |index| {
            descriptor_distance(&descriptors[pivot], &descriptors[*index])
        });
        Self::Branch {
            pivot,
            radius,
            inner: Box::new(Self::build(descriptors, indices)),
            outer: Box::new(Self::build(descriptors, outer_indices)),
        }
    }

    fn search(
        &self,
        descriptors: &[FpfhDescriptor],
        query: &FpfhDescriptor,
        best: &mut (usize, f64),
    ) {
        match self {
            Self::Leaf(indices) => {
                for &index in indices {
                    update_nearest(index, descriptor_distance(query, &descriptors[index]), best);
                }
            }
            Self::Branch {
                pivot,
                radius,
                inner,
                outer,
            } => {
                let pivot_distance = descriptor_distance(query, &descriptors[*pivot]);
                update_nearest(*pivot, pivot_distance, best);
                if pivot_distance < *radius {
                    inner.search(descriptors, query, best);
                    if pivot_distance + best.1 >= *radius {
                        outer.search(descriptors, query, best);
                    }
                } else {
                    outer.search(descriptors, query, best);
                    if pivot_distance - best.1 <= *radius {
                        inner.search(descriptors, query, best);
                    }
                }
            }
        }
    }
}

fn descriptor_distance(left: &FpfhDescriptor, right: &FpfhDescriptor) -> f64 {
    left.iter()
        .zip(right)
        .map(|(left, right)| (left - right) * (left - right))
        .sum::<f64>()
        .sqrt()
}

fn update_nearest(index: usize, distance: f64, best: &mut (usize, f64)) {
    if distance < best.1 || (distance == best.1 && index < best.0) {
        *best = (index, distance);
    }
}
