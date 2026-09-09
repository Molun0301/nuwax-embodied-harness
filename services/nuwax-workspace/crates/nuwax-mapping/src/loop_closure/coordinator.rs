//! Runtime-facing keyframe, loop-constraint, optimization, and correction pipeline.

use std::collections::BTreeMap;

use nalgebra::{Isometry3, Matrix6, Point3, Translation3};
use thiserror::Error;

use super::optimizer::{cost_strictly_improved, graph_cost, OptimizerConfig};
use super::pgo::{PoseEdge, PoseGraph};
use super::registration::evaluate_loop_registration;
use super::PgoOptimizer;
use crate::relocalization::{IcpConfig, IcpSolver};

/// Configuration for the complete loop-closure coordinator.
#[derive(Debug, Clone, Copy)]
pub struct LoopClosurePipelineConfig {
    /// Translation required before a frame becomes a keyframe.
    pub keyframe_translation: f64,
    /// Rotation angle required before a frame becomes a keyframe.
    pub keyframe_rotation: f64,
    /// Maximum odometry-space distance for loop candidate search.
    pub loop_search_radius: f64,
    /// Minimum time separation between loop candidates.
    pub min_loop_separation_secs: f64,
    /// Minimum points required in each keyframe cloud for ICP.
    pub min_keyframe_points: usize,
    /// Maximum accepted ICP mean squared error.
    pub max_loop_mse: f64,
    /// 最终匹配在当前及历史扫描中各自至少覆盖的点比例。
    pub min_loop_overlap_ratio: f64,
    /// ICP configuration used to create loop measurements.
    pub icp: IcpConfig,
    /// Pose-graph optimizer configuration.
    pub optimizer: OptimizerConfig,
}

impl Default for LoopClosurePipelineConfig {
    fn default() -> Self {
        Self {
            keyframe_translation: 0.5,
            keyframe_rotation: 0.2,
            loop_search_radius: 1.0,
            min_loop_separation_secs: 30.0,
            min_keyframe_points: 10,
            max_loop_mse: 0.05,
            min_loop_overlap_ratio: 0.5,
            icp: IcpConfig {
                max_correspondence_distance: 1.0,
                ..IcpConfig::default()
            },
            optimizer: OptimizerConfig::default(),
        }
    }
}

/// One immutable keyframe used to interpolate drift correction.
#[derive(Debug, Clone)]
pub struct CorrectionKeyframe {
    /// Source timestamp.
    pub timestamp: f64,
    /// Raw odometry-frame body pose.
    pub local_pose: Isometry3<f64>,
    /// Optimized body pose.
    pub optimized_pose: Isometry3<f64>,
}

/// Time-indexed pose correction, matching the DimOS clipping and interpolation semantics.
#[derive(Debug, Clone, Default)]
pub struct CorrectionTimeline {
    keyframes: Vec<CorrectionKeyframe>,
}

impl CorrectionTimeline {
    /// Build a correction timeline from strictly increasing finite timestamps.
    ///
    /// # Errors
    /// Returns an error when a timestamp is non-finite, duplicated, or out of order.
    pub fn new(keyframes: Vec<CorrectionKeyframe>) -> Result<Self, LoopClosurePipelineError> {
        validate_correction_timestamps(&keyframes)?;
        Ok(Self { keyframes })
    }

    /// Return the keyframes backing this timeline.
    #[must_use]
    pub fn keyframes(&self) -> &[CorrectionKeyframe] {
        &self.keyframes
    }

    /// Interpolate `optimized * local.inverse()` at `timestamp`, clipping outside the range.
    pub fn correction_at(
        &self,
        timestamp: f64,
    ) -> Result<Isometry3<f64>, LoopClosurePipelineError> {
        if !timestamp.is_finite() {
            return Err(LoopClosurePipelineError::InvalidTimestamp(timestamp));
        }
        validate_correction_timestamps(&self.keyframes)?;
        let first = self
            .keyframes
            .first()
            .ok_or(LoopClosurePipelineError::NoKeyframes)?;
        if self.keyframes.len() == 1 || timestamp <= first.timestamp {
            return Ok(correction(first));
        }
        let last = self
            .keyframes
            .last()
            .ok_or(LoopClosurePipelineError::NoKeyframes)?;
        if timestamp >= last.timestamp {
            return Ok(correction(last));
        }

        let upper = self
            .keyframes
            .partition_point(|keyframe| keyframe.timestamp < timestamp);
        let lower = upper
            .checked_sub(1)
            .ok_or(LoopClosurePipelineError::InvalidCorrectionTimeline)?;
        let lower_keyframe = self
            .keyframes
            .get(lower)
            .ok_or(LoopClosurePipelineError::InvalidCorrectionTimeline)?;
        let upper_keyframe = self
            .keyframes
            .get(upper)
            .ok_or(LoopClosurePipelineError::InvalidCorrectionTimeline)?;
        let duration = upper_keyframe.timestamp - lower_keyframe.timestamp;
        let alpha = (timestamp - lower_keyframe.timestamp) / duration;
        let lower_correction = correction(lower_keyframe);
        let upper_correction = correction(upper_keyframe);
        let translation = lower_correction
            .translation
            .vector
            .lerp(&upper_correction.translation.vector, alpha);
        let rotation = lower_correction
            .rotation
            .slerp(&upper_correction.rotation, alpha);
        Ok(Isometry3::from_parts(
            Translation3::from(translation),
            rotation,
        ))
    }

    /// Apply the interpolated correction to a raw pose.
    pub fn correct_pose(
        &self,
        timestamp: f64,
        raw_pose: &Isometry3<f64>,
    ) -> Result<Isometry3<f64>, LoopClosurePipelineError> {
        Ok(self.correction_at(timestamp)? * raw_pose)
    }
}

fn validate_correction_timestamps(
    keyframes: &[CorrectionKeyframe],
) -> Result<(), LoopClosurePipelineError> {
    for keyframe in keyframes {
        if !keyframe.timestamp.is_finite() {
            return Err(LoopClosurePipelineError::InvalidTimestamp(
                keyframe.timestamp,
            ));
        }
    }
    if keyframes
        .windows(2)
        .any(|window| window[0].timestamp >= window[1].timestamp)
    {
        return Err(LoopClosurePipelineError::InvalidCorrectionTimeline);
    }
    Ok(())
}

fn correction(keyframe: &CorrectionKeyframe) -> Isometry3<f64> {
    keyframe.optimized_pose * keyframe.local_pose.inverse()
}

#[derive(Debug, Clone)]
struct StoredKeyframe {
    timestamp: f64,
    local_pose: Isometry3<f64>,
    points: Vec<Point3<f64>>,
}

/// An accepted point-cloud loop edge.
#[derive(Debug, Clone)]
pub struct LoopClosureRecord {
    /// Historic keyframe node.
    pub from: u64,
    /// Current keyframe node.
    pub to: u64,
    /// ICP measurement from current body coordinates into the historic body frame.
    pub measurement: Isometry3<f64>,
    /// 最终双向对应的均方误差。
    pub mse: f64,
    /// 当前扫描中落在对应距离内的点数。
    pub source_inliers: usize,
    /// 历史扫描中落在对应距离内的点数。
    pub target_inliers: usize,
    /// 当前扫描中实际对应的点比例。
    pub source_overlap: f64,
    /// 历史扫描中实际对应的点比例。
    pub target_overlap: f64,
}

/// Result of processing one mapping frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProcessFrameResult {
    /// Frame did not contain enough points to be a keyframe.
    InsufficientPoints,
    /// Motion since the prior keyframe was below both thresholds.
    BelowKeyframeThreshold,
    /// A keyframe and sequential odometry constraint were added.
    KeyframeAdded {
        /// Newly allocated graph node.
        node_id: u64,
    },
    /// A keyframe, measured loop edge, and optimized correction were added.
    LoopClosed {
        /// Newly allocated graph node.
        node_id: u64,
        /// Historic graph node matched by ICP.
        historic_id: u64,
    },
}

/// Validation and runtime errors from the loop-closure pipeline.
#[derive(Debug, Error, PartialEq)]
pub enum LoopClosurePipelineError {
    /// A timestamp was not finite.
    #[error("timestamp must be finite, got {0}")]
    InvalidTimestamp(f64),
    /// An input pose contained a non-finite component.
    #[error("pose must contain only finite values")]
    NonFinitePose,
    /// An input point contained a non-finite coordinate.
    #[error("point cloud must contain only finite values")]
    NonFinitePoint,
    /// The coordinator configuration is invalid.
    #[error("invalid loop-closure pipeline configuration: {0}")]
    InvalidConfig(&'static str),
    /// Correction was requested before a keyframe existed.
    #[error("pose correction requires at least one keyframe")]
    NoKeyframes,
    /// Correction keyframe timestamps were duplicated or out of order.
    #[error("correction keyframe timestamps must be strictly increasing")]
    InvalidCorrectionTimeline,
}

/// Owns one mapping session's keyframes, ICP constraints, pose graph, and corrections.
pub struct LoopClosureCoordinator {
    config: LoopClosurePipelineConfig,
    graph: PoseGraph,
    keyframes: BTreeMap<u64, StoredKeyframe>,
    loops: Vec<LoopClosureRecord>,
    timeline: CorrectionTimeline,
}

impl LoopClosureCoordinator {
    /// Create an empty mapping-session coordinator.
    pub fn new(config: LoopClosurePipelineConfig) -> Result<Self, LoopClosurePipelineError> {
        validate_config(config)?;
        Ok(Self {
            config,
            graph: PoseGraph::new(),
            keyframes: BTreeMap::new(),
            loops: Vec::new(),
            timeline: CorrectionTimeline::default(),
        })
    }

    /// Process one odometry pose and body-frame point cloud.
    pub fn process_frame(
        &mut self,
        timestamp: f64,
        local_pose: Isometry3<f64>,
        points: &[Point3<f64>],
    ) -> Result<ProcessFrameResult, LoopClosurePipelineError> {
        if !timestamp.is_finite() {
            return Err(LoopClosurePipelineError::InvalidTimestamp(timestamp));
        }
        if !is_finite_pose(&local_pose) {
            return Err(LoopClosurePipelineError::NonFinitePose);
        }
        if points.iter().any(|point| !is_finite_point(point)) {
            return Err(LoopClosurePipelineError::NonFinitePoint);
        }
        if points.len() < self.config.min_keyframe_points {
            return Ok(ProcessFrameResult::InsufficientPoints);
        }
        if let Some((_, last)) = self.keyframes.last_key_value() {
            if timestamp <= last.timestamp {
                return Err(LoopClosurePipelineError::InvalidCorrectionTimeline);
            }
            let delta = last.local_pose.inverse() * local_pose;
            if delta.translation.vector.norm() < self.config.keyframe_translation
                && delta.rotation.angle() < self.config.keyframe_rotation
            {
                return Ok(ProcessFrameResult::BelowKeyframeThreshold);
            }
        }

        let previous = self
            .keyframes
            .last_key_value()
            .map(|(id, frame)| (*id, frame.local_pose));
        // 图中的节点属于优化坐标；新节点沿上一优化位姿传播原始里程计增量。
        // 直接写回原始位姿会丢失已有校正，并给下一次优化制造虚假残差。
        let initial_pose = previous.map_or(local_pose, |(previous_id, previous_pose)| {
            self.graph.nodes[&previous_id].pose * (previous_pose.inverse() * local_pose)
        });
        let node_id = self.graph.add_node(timestamp.floor() as i64, initial_pose);
        if let Some((previous_id, previous_pose)) = previous {
            self.graph.add_edge(PoseEdge::new(
                previous_id,
                node_id,
                previous_pose.inverse() * local_pose,
            ));
        } else {
            self.graph.fix_first_node();
        }
        self.keyframes.insert(
            node_id,
            StoredKeyframe {
                timestamp,
                local_pose,
                points: points.to_vec(),
            },
        );

        let loop_result = self.find_and_measure_loop(node_id);
        let result = if let Some(record) = loop_result {
            let information_scale = 1.0 / record.mse.max(1e-6);
            let mut optimized_graph = self.graph.clone();
            optimized_graph.add_edge(
                PoseEdge::new(record.from, record.to, record.measurement)
                    .with_information(Matrix6::identity() * information_scale),
            );
            let initial_cost = graph_cost(&optimized_graph);
            let historic_id = record.from;
            let final_cost =
                PgoOptimizer::with_config(self.config.optimizer).optimize(&mut optimized_graph);
            if cost_strictly_improved(initial_cost, final_cost)
                && has_valid_correction(&self.graph, &optimized_graph)
            {
                self.graph = optimized_graph;
                self.loops.push(record);
                ProcessFrameResult::LoopClosed {
                    node_id,
                    historic_id,
                }
            } else {
                ProcessFrameResult::KeyframeAdded { node_id }
            }
        } else {
            ProcessFrameResult::KeyframeAdded { node_id }
        };
        self.refresh_timeline()?;
        Ok(result)
    }

    /// Current optimized pose graph.
    #[must_use]
    pub fn graph(&self) -> &PoseGraph {
        &self.graph
    }

    /// Accepted measured loop constraints.
    #[must_use]
    pub fn loops(&self) -> &[LoopClosureRecord] {
        &self.loops
    }

    /// Current drift-correction timeline.
    #[must_use]
    pub fn corrections(&self) -> &CorrectionTimeline {
        &self.timeline
    }

    /// Reset all session-owned graph state while retaining configuration.
    pub fn reset(&mut self) {
        self.graph = PoseGraph::new();
        self.keyframes.clear();
        self.loops.clear();
        self.timeline = CorrectionTimeline::default();
    }

    fn find_and_measure_loop(&self, current_id: u64) -> Option<LoopClosureRecord> {
        let current = self.keyframes.get(&current_id)?;
        let mut candidates: Vec<(f64, u64, &StoredKeyframe)> = self
            .keyframes
            .iter()
            .filter(|(id, historic)| {
                **id != current_id
                    && (current.timestamp - historic.timestamp).abs()
                        >= self.config.min_loop_separation_secs
            })
            .filter_map(|(id, historic)| {
                let distance = (current.local_pose.translation.vector
                    - historic.local_pose.translation.vector)
                    .norm();
                if distance > self.config.loop_search_radius {
                    return None;
                }
                // 时间间隔和邻近位置不能证明回环；必须确实离开过历史邻域再返回。
                // 这也排除相邻关键帧、原地转向及沿重复结构的缓慢直行。
                let departed = self
                    .keyframes
                    .range((
                        std::ops::Bound::Excluded(*id),
                        std::ops::Bound::Excluded(current_id),
                    ))
                    .any(|(_, middle)| {
                        (middle.local_pose.translation.vector
                            - historic.local_pose.translation.vector)
                            .norm()
                            > self.config.loop_search_radius
                    });
                if !departed {
                    tracing::debug!(
                        event = "pgo.loop.rejected",
                        reason = "not_revisited",
                        current_id,
                        historic_id = *id,
                        "轨迹尚未离开并重访历史邻域"
                    );
                    return None;
                }
                Some((distance, *id, historic))
            })
            .collect();
        candidates.sort_by(|left, right| left.0.total_cmp(&right.0));

        for (_, historic_id, historic) in candidates {
            // 两帧点分别属于各自机体坐标，先用里程计相对位姿对齐。
            // 从单位变换直接匹配会在转向或重复结构中收敛到错误回环。
            let historic_from_current = historic.local_pose.inverse() * current.local_pose;
            let seeded_points: Vec<_> = current
                .points
                .iter()
                .map(|point| historic_from_current * point)
                .collect();
            if seeded_points.iter().any(|point| !is_finite_point(point)) {
                continue;
            }
            let registration =
                IcpSolver::new(self.config.icp).align(&seeded_points, &historic.points);
            if !registration.converged
                || !registration.mse.is_finite()
                || registration.mse > self.config.max_loop_mse
                || !is_finite_pose(&registration.transformation)
            {
                continue;
            }
            // 不能用少量被筛选内点的低误差证明整帧回环；在最终变换上独立核对两帧。
            let evidence = evaluate_loop_registration(
                &seeded_points,
                &historic.points,
                &registration.transformation,
                self.config.icp.max_correspondence_distance,
            );
            if let Some(reason) = evidence.rejection_reason(
                self.config.min_keyframe_points,
                self.config.min_loop_overlap_ratio,
                self.config.max_loop_mse,
            ) {
                tracing::debug!(
                    event = "pgo.loop.rejected",
                    reason,
                    current_id,
                    historic_id,
                    source_inliers = evidence.source_inliers,
                    target_inliers = evidence.target_inliers,
                    source_overlap = evidence.source_overlap,
                    target_overlap = evidence.target_overlap,
                    mse = evidence.mse,
                    "回环点云匹配缺少充分几何证据"
                );
                continue;
            }
            return Some(LoopClosureRecord {
                from: historic_id,
                to: current_id,
                // 求解器返回历史坐标中的残差，约束须还原为完整机体间变换。
                measurement: registration.transformation * historic_from_current,
                mse: evidence.mse,
                source_inliers: evidence.source_inliers,
                target_inliers: evidence.target_inliers,
                source_overlap: evidence.source_overlap,
                target_overlap: evidence.target_overlap,
            });
        }
        None
    }

    fn refresh_timeline(&mut self) -> Result<(), LoopClosurePipelineError> {
        let keyframes = self
            .keyframes
            .iter()
            .filter_map(|(id, stored)| {
                self.graph.nodes.get(id).map(|node| CorrectionKeyframe {
                    timestamp: stored.timestamp,
                    local_pose: stored.local_pose,
                    optimized_pose: node.pose,
                })
            })
            .collect();
        self.timeline = CorrectionTimeline::new(keyframes)?;
        Ok(())
    }
}

fn validate_config(config: LoopClosurePipelineConfig) -> Result<(), LoopClosurePipelineError> {
    if !config.keyframe_translation.is_finite() || config.keyframe_translation < 0.0 {
        return Err(LoopClosurePipelineError::InvalidConfig(
            "keyframe_translation must be finite and non-negative",
        ));
    }
    if !config.keyframe_rotation.is_finite() || config.keyframe_rotation < 0.0 {
        return Err(LoopClosurePipelineError::InvalidConfig(
            "keyframe_rotation must be finite and non-negative",
        ));
    }
    if !config.loop_search_radius.is_finite() || config.loop_search_radius <= 0.0 {
        return Err(LoopClosurePipelineError::InvalidConfig(
            "loop_search_radius must be finite and positive",
        ));
    }
    if !config.min_loop_separation_secs.is_finite() || config.min_loop_separation_secs < 0.0 {
        return Err(LoopClosurePipelineError::InvalidConfig(
            "min_loop_separation_secs must be finite and non-negative",
        ));
    }
    if config.min_keyframe_points < 3 {
        return Err(LoopClosurePipelineError::InvalidConfig(
            "min_keyframe_points must be at least three",
        ));
    }
    if !config.min_loop_overlap_ratio.is_finite()
        || config.min_loop_overlap_ratio <= 0.0
        || config.min_loop_overlap_ratio > 1.0
    {
        return Err(LoopClosurePipelineError::InvalidConfig(
            "min_loop_overlap_ratio must be finite and within (0, 1]",
        ));
    }
    if !config.max_loop_mse.is_finite() || config.max_loop_mse < 0.0 {
        return Err(LoopClosurePipelineError::InvalidConfig(
            "max_loop_mse must be finite and non-negative",
        ));
    }
    if config.icp.max_iterations == 0
        || !config.icp.mse_tolerance.is_finite()
        || config.icp.mse_tolerance < 0.0
        || !config.icp.max_correspondence_distance.is_finite()
        || config.icp.max_correspondence_distance <= 0.0
    {
        return Err(LoopClosurePipelineError::InvalidConfig(
            "icp configuration must have positive iterations/distance and finite tolerances",
        ));
    }
    if !config.optimizer.is_valid() {
        return Err(LoopClosurePipelineError::InvalidConfig(
            "optimizer requires positive iterations/lambda, finite tolerances, and lambda_scale > 1",
        ));
    }
    Ok(())
}

fn is_finite_pose(pose: &Isometry3<f64>) -> bool {
    pose.translation
        .vector
        .iter()
        .all(|value| value.is_finite())
        && pose.rotation.coords.iter().all(|value| value.is_finite())
}

fn is_finite_point(point: &Point3<f64>) -> bool {
    point.coords.iter().all(|value| value.is_finite())
}

fn has_valid_correction(before: &PoseGraph, after: &PoseGraph) -> bool {
    before.nodes.len() == after.nodes.len()
        && before.nodes.iter().all(|(id, old_node)| {
            after.nodes.get(id).is_some_and(|new_node| {
                is_finite_pose(&new_node.pose)
                    && if old_node.fixed {
                        new_node.pose == old_node.pose
                    } else {
                        true
                    }
            })
        })
        && before.nodes.iter().any(|(id, old_node)| {
            after.nodes.get(id).is_some_and(|new_node| {
                let delta = old_node.pose.inverse() * new_node.pose;
                delta.translation.vector.norm() > 1e-12 || delta.rotation.angle() > 1e-12
            })
        })
}
