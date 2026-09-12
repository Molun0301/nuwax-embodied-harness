//! Runtime-facing keyframe, loop-constraint, optimization, and correction pipeline.

use std::collections::BTreeMap;

use nalgebra::{Isometry3, Matrix6, Point3, Translation3};
use thiserror::Error;

use super::optimizer::{cost_strictly_improved, graph_cost, OptimizerConfig};
use super::pgo::{PoseEdge, PoseGraph};
use super::registration::evaluate_loop_registration;
use super::scan_context::ScanContext;
use super::PgoOptimizer;
use crate::relocalization::{voxel_downsample, IcpConfig, IcpResult, IcpSolver};

/// Configuration for the complete loop-closure coordinator.
#[derive(Debug, Clone, Copy)]
pub struct LoopClosurePipelineConfig {
    /// 启用与里程计距离无关的地点检索；几何门验证可单独关闭它以隔离候选来源。
    pub scan_context_enabled: bool,
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
    /// 原Dimos库内路径使用点到平面匹配；点到点供离散点集调用方明确选择。
    pub point_to_plane: bool,
    /// 匹配前的关键帧和子图体素大小。
    pub submap_resolution_m: f64,
    /// 历史子图在目标节点两侧各保留的关键帧数。
    pub loop_submap_half_range: usize,
    /// 开始回环搜索所需的最少关键帧数。
    pub min_keyframes_for_loop_search: usize,
    /// 两次接纳回环之间的最短源时间间隔。
    pub min_loop_interval_secs: f64,
    /// Pose-graph optimizer configuration.
    pub optimizer: OptimizerConfig,
}

impl Default for LoopClosurePipelineConfig {
    fn default() -> Self {
        Self {
            scan_context_enabled: true,
            keyframe_translation: 0.5,
            keyframe_rotation: 10.0_f64.to_radians(),
            loop_search_radius: 2.0,
            min_loop_separation_secs: 20.0,
            min_keyframe_points: 10,
            max_loop_mse: 0.04,
            min_loop_overlap_ratio: 0.5,
            icp: IcpConfig {
                max_correspondence_distance: 1.0,
                ..IcpConfig::default()
            },
            point_to_plane: true,
            submap_resolution_m: 0.2,
            loop_submap_half_range: 10,
            min_keyframes_for_loop_search: 10,
            min_loop_interval_secs: 5.0,
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
    context: ScanContext,
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
    /// 由独立对应支撑估计的有界信息矩阵，平移在前、旋转在后。
    pub information: Matrix6<f64>,
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
            let delta = previous_pose.inverse() * local_pose;
            // LIO没有提供关键帧增量协方差时采用随运动增长的保守噪声模型。
            // 不能把垂直/旋转固定成近乎无误差，否则回环无法纠正长期累计漂移。
            let distance = delta.translation.vector.norm();
            let translation_sigma = 0.02 + 0.02 * distance;
            let rotation_sigma =
                0.5_f64.to_radians() * (1.0 + distance) + 0.05 * delta.rotation.angle();
            let translation_information = translation_sigma.powi(-2);
            let rotation_information = rotation_sigma.powi(-2);
            self.graph
                .add_edge(PoseEdge::new(previous_id, node_id, delta).with_information(
                    Matrix6::from_diagonal(&nalgebra::Vector6::new(
                        translation_information,
                        translation_information,
                        translation_information,
                        rotation_information,
                        rotation_information,
                        rotation_information,
                    )),
                ));
        } else {
            self.graph.fix_first_node();
        }
        self.keyframes.insert(
            node_id,
            StoredKeyframe {
                timestamp,
                local_pose,
                points: voxel_downsample(points, self.config.submap_resolution_m),
                context: ScanContext::new(points, local_pose.rotation),
            },
        );

        let loop_result = self.find_and_measure_loop(node_id);
        let result = if let Some(record) = loop_result {
            let mut optimized_graph = self.graph.clone();
            optimized_graph.add_edge(
                PoseEdge::new(record.from, record.to, record.measurement)
                    .with_information(record.information),
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
        if self.keyframes.len() < self.config.min_keyframes_for_loop_search {
            return None;
        }
        if self
            .loops
            .last()
            .and_then(|record| self.keyframes.get(&record.to))
            .is_some_and(|previous| {
                current.timestamp - previous.timestamp < self.config.min_loop_interval_secs
            })
        {
            return None;
        }
        let candidates = self.loop_candidates(current_id);

        for (historic_id, historic_from_current) in candidates {
            let historic_pose = self.graph.nodes.get(&historic_id)?.pose;
            let first = historic_id.saturating_sub(self.config.loop_submap_half_range as u64);
            let last = (historic_id + self.config.loop_submap_half_range as u64)
                .min(current_id.saturating_sub(1));
            let mut target = Vec::new();
            for (id, keyframe) in self.keyframes.range(first..=last) {
                // 当前时段的邻帧不能混入历史目标，否则会用自身里程计证明回环。
                if current.timestamp - keyframe.timestamp < self.config.min_loop_separation_secs {
                    continue;
                }
                let target_from_body = historic_pose.inverse() * self.graph.nodes.get(id)?.pose;
                target.extend(keyframe.points.iter().map(|point| target_from_body * point));
            }
            let target = voxel_downsample(&target, self.config.submap_resolution_m);
            let seeded_points: Vec<_> = current
                .points
                .iter()
                .map(|point| historic_from_current * point)
                .collect();
            if seeded_points.iter().any(|point| !is_finite_point(point)) {
                continue;
            }
            let registration = if self.config.point_to_plane {
                let source: Vec<[f64; 3]> = seeded_points.iter().map(|p| p.coords.into()).collect();
                let target_array: Vec<[f64; 3]> = target.iter().map(|p| p.coords.into()).collect();
                match nuwax_gtsam::point_to_plane(
                    &source,
                    &target_array,
                    self.config.icp.max_correspondence_distance,
                    self.config.icp.max_iterations,
                ) {
                    Ok(result) => {
                        let q = result.transformation.rotation_xyzw;
                        IcpResult {
                            transformation: Isometry3::from_parts(
                                Translation3::from(nalgebra::Vector3::from(
                                    result.transformation.translation,
                                )),
                                nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
                                    q[3], q[0], q[1], q[2],
                                )),
                            ),
                            mse: result.mse,
                            iterations: result.iterations as usize,
                            converged: result.converged != 0,
                        }
                    }
                    Err(error) => {
                        tracing::debug!(%error,"回环点到平面匹配失败");
                        continue;
                    }
                }
            } else {
                IcpSolver::new(self.config.icp).align(&seeded_points, &target)
            };
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
                &target,
                &registration.transformation,
                if self.config.point_to_plane {
                    self.config.icp.max_correspondence_distance.min(0.3)
                } else {
                    self.config.icp.max_correspondence_distance
                },
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
            // 对应点相关，最多按64个独立支撑计权；保留2厘米与0.5度方差下限。
            // 用几何力臂把米平方换成弧度平方，不直接将匹配误差充作角度方差。
            let support = evidence
                .source_inliers
                .min(evidence.unique_target_inliers)
                .min(64) as f64;
            let position_variance = (evidence.mse / support).max(0.02_f64.powi(2));
            let rotation_variance = (position_variance / evidence.lever_arm_m2.max(0.25))
                .max(0.5_f64.to_radians().powi(2));
            let information = Matrix6::from_diagonal(&nalgebra::Vector6::new(
                position_variance.recip(),
                position_variance.recip(),
                position_variance.recip(),
                rotation_variance.recip(),
                rotation_variance.recip(),
                rotation_variance.recip(),
            ));
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
                information,
            });
        }
        None
    }

    /// 每个关键帧最多四次几何配准，描述子只精比六个环键近邻。
    fn loop_candidates(&self, current_id: u64) -> Vec<(u64, Isometry3<f64>)> {
        let current = &self.keyframes[&current_id];
        let current_pose = self.graph.nodes[&current_id].pose;
        let mut spatial = Vec::new();
        let mut descriptors = Vec::new();
        for (id, historic) in self.keyframes.iter().filter(|(id, historic)| {
            **id != current_id
                && (current.timestamp - historic.timestamp).abs()
                    >= self.config.min_loop_separation_secs
        }) {
            let distance = (current_pose.translation.vector
                - self.graph.nodes[id].pose.translation.vector)
                .norm();
            if distance <= self.config.loop_search_radius {
                spatial.push((distance, *id));
            }
            descriptors.push((current.context.ring_distance(&historic.context), *id));
        }
        spatial.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        descriptors.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        let mut candidates: Vec<_> = spatial
            .iter()
            .take(2)
            .map(|(_, id)| (*id, self.graph.nodes[id].pose.inverse() * current_pose))
            .collect();
        let mut matches: Vec<_> = descriptors
            .iter()
            .filter(|_| self.config.scan_context_enabled)
            .take(6)
            .filter_map(|(_, id)| {
                current
                    .context
                    .align_to(&self.keyframes[id].context)
                    .map(|(score, yaw)| (score, *id, yaw))
            })
            .collect();
        matches.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        for (_, id, yaw) in matches {
            let historic = &self.keyframes[&id];
            let rotation = historic.context.level_from_body.inverse()
                * nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, yaw)
                * current.context.level_from_body;
            let center = |points: &[Point3<f64>]| {
                points
                    .iter()
                    .fold(nalgebra::Vector3::zeros(), |sum, p| sum + p.coords)
                    / points.len() as f64
            };
            let translation = center(&historic.points) - rotation * center(&current.points);
            let seed = Isometry3::from_parts(Translation3::from(translation), rotation);
            // 距离初值可能已受漂移影响，同一历史地点允许描述子提供另一条独立初值。
            if candidates.iter().any(|(existing, pose)| {
                *existing == id
                    && (pose.inverse() * seed).translation.vector.norm() < 0.1
                    && pose.rotation.angle_to(&rotation) < 0.05
            }) {
                continue;
            }
            candidates.push((id, seed));
            if candidates.len() == 4 {
                break;
            }
        }
        for (_, id) in spatial.into_iter().skip(2) {
            if candidates.len() >= 4 {
                break;
            }
            if candidates.iter().all(|(existing, _)| *existing != id) {
                candidates.push((id, self.graph.nodes[&id].pose.inverse() * current_pose));
            }
        }
        candidates
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
    if config.min_keyframes_for_loop_search < 3
        || config.min_keyframes_for_loop_search > 2000
        || config.loop_submap_half_range > 100
        || !config.submap_resolution_m.is_finite()
        || config.submap_resolution_m <= 0.0
        || !config.min_loop_interval_secs.is_finite()
        || config.min_loop_interval_secs < 0.0
    {
        return Err(LoopClosurePipelineError::InvalidConfig(
            "invalid submap or loop search configuration",
        ));
    }
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

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::UnitQuaternion;

    #[test]
    fn distant_return_uses_descriptor_and_removes_translation_and_yaw_drift() {
        let world: Vec<_> = (0..60)
            .flat_map(|sector| {
                (0..3).map(move |k| {
                    let angle = (sector as f64 + 0.3) * std::f64::consts::TAU / 60.0;
                    let radius = 2.0 + ((sector * 7 + k * 13) % 17) as f64;
                    Point3::new(
                        radius * angle.cos(),
                        radius * angle.sin(),
                        (sector % 5) as f64 * 0.3,
                    )
                })
            })
            .collect();
        let mut coordinator = LoopClosureCoordinator::new(LoopClosurePipelineConfig {
            point_to_plane: false,
            min_loop_separation_secs: 1000.0,
            submap_resolution_m: 0.01,
            ..Default::default()
        })
        .unwrap();
        let mut last = ProcessFrameResult::InsufficientPoints;
        let mut raw = Isometry3::identity();
        let mut truth = Isometry3::identity();
        for i in 0..=30 {
            let x = if i <= 15 {
                i as f64 * 0.5
            } else {
                (30 - i) as f64 * 0.5
            };
            let yaw = if i == 30 { std::f64::consts::PI } else { 0.0 };
            truth = Isometry3::from_parts(
                Translation3::new(x, 0.0, 0.0),
                UnitQuaternion::from_euler_angles(0.0, 0.0, yaw),
            );
            raw = Isometry3::from_parts(
                Translation3::new(x + i as f64 * 0.1, 0.0, 0.0),
                UnitQuaternion::from_euler_angles(0.0, 0.0, yaw + i as f64 / 150.0),
            );
            let body: Vec<_> = world.iter().map(|p| truth.inverse() * p).collect();
            last = coordinator
                .process_frame(if i == 30 { 1000.0 } else { i as f64 }, raw, &body)
                .unwrap();
        }
        assert!(
            matches!(last, ProcessFrameResult::LoopClosed { historic_id: 0, .. }),
            "{last:?}"
        );
        let corrected = coordinator
            .corrections()
            .correct_pose(1000.0, &raw)
            .unwrap();
        let error = truth.inverse() * corrected;
        eprintln!(
            "三米/0.2弧度回程漂移修正后：平移={}米，旋转={}度",
            error.translation.vector.norm(),
            error.rotation.angle().to_degrees()
        );
        assert!(error.translation.vector.norm() < 0.10, "{error:?}");
        assert!(error.rotation.angle() < 1.0_f64.to_radians(), "{error:?}");
        assert_eq!(coordinator.graph.nodes[&0].pose, Isometry3::identity());
        let current_id = *coordinator.keyframes.last_key_value().unwrap().0;
        assert!(coordinator.loop_candidates(current_id).len() <= 4);
    }

    #[test]
    fn point_to_plane_rejects_unobservable_single_plane_loop() {
        let target: Vec<[f64; 3]> = (-20..=20)
            .flat_map(|x| (-20..=20).map(move |y| [x as f64 * 0.1, y as f64 * 0.1, 0.0]))
            .collect();
        let source: Vec<_> = target
            .iter()
            .map(|p| [p[0] + 0.1, p[1] - 0.1, p[2] + 0.05])
            .collect();
        let result = nuwax_gtsam::point_to_plane(&source, &target, 1.0, 50).unwrap();
        assert_eq!(
            result.converged, 0,
            "单平面不能证明完整六自由度回环：{result:?}"
        );
    }
}
