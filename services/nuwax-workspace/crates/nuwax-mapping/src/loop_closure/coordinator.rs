//! Runtime-facing keyframe, loop-constraint, optimization, and correction pipeline.

use std::collections::BTreeMap;

use nalgebra::{Isometry3, Matrix6, Point3, Translation3};
use thiserror::Error;

use super::optimizer::{cost_strictly_improved, graph_cost, OptimizerConfig};
use super::pgo::{PoseEdge, PoseGraph};
use super::registration::evaluate_loop_registration;
use super::scan_context::{ScanContext, ScanContextConfig};
use super::se3::{se3_adjoint, se3_log_to_vector6};
use super::PgoOptimizer;
use crate::relocalization::{voxel_downsample, IcpConfig, IcpResult, IcpSolver};

/// Configuration for the complete loop-closure coordinator.
#[derive(Debug, Clone, Copy)]
pub struct LoopClosurePipelineConfig {
    /// 启用与里程计距离无关的地点检索；几何门验证可单独关闭它以隔离候选来源。
    pub scan_context_enabled: bool,
    /// 扫描上下文的范围、相似度及航向歧义门限。
    pub scan_context: ScanContextConfig,
    /// 每个关键帧最多尝试的空间与描述子配准初值总数。
    pub loop_candidate_count: usize,
    /// 对比里程计路径及候选之间的带协方差闭合误差，数值为马氏距离平方上限。
    pub max_loop_innovation: f64,
    /// 至少由多少次分离的观测确认同一回环，单次匹配不能直接改写整图。
    pub loop_confirmation_frames: usize,
    /// 确认观测之间的最短源时间间隔。
    pub loop_confirmation_interval_secs: f64,
    /// 原生LIO重力对齐朝向的软先验，保留偏航和平移自由度；None用于非惯性数据。
    pub gravity_prior_sigma_rad: Option<f64>,
    /// 未提供增量协方差时的基础标准差，同一模型用于图边和一致性检查。
    pub odometry_translation_sigma_m: f64,
    /// 里程计增量旋转的基础标准差，单位弧度。
    pub odometry_rotation_sigma_rad: f64,
    /// 回环平移置信度下限，同时参与图权重和一致性检查。
    pub loop_translation_sigma_floor_m: f64,
    /// 回环旋转置信度下限，单位弧度。
    pub loop_rotation_sigma_floor_rad: f64,
    /// Translation required before a frame becomes a keyframe.
    pub keyframe_translation: f64,
    /// Rotation angle required before a frame becomes a keyframe.
    pub keyframe_rotation: f64,
    /// 已有回环历史时的静止观测间隔，避免首次回程扫描失败后永久停止搜索。
    pub stationary_probe_interval_secs: f64,
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
            scan_context: ScanContextConfig::default(),
            loop_candidate_count: 4,
            max_loop_innovation: 30.0,
            loop_confirmation_frames: 2,
            loop_confirmation_interval_secs: 1.0,
            gravity_prior_sigma_rad: Some(1.0_f64.to_radians()),
            odometry_translation_sigma_m: 0.02,
            odometry_rotation_sigma_rad: 0.5_f64.to_radians(),
            loop_translation_sigma_floor_m: 0.02,
            loop_rotation_sigma_floor_rad: 0.5_f64.to_radians(),
            keyframe_translation: 0.5,
            keyframe_rotation: 10.0_f64.to_radians(),
            stationary_probe_interval_secs: 2.0,
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

impl LoopClosurePipelineConfig {
    /// 已校准Point-LIO的回环参数：优先保留低漂移里程计，只接纳充分确认的局部配准。
    /// 其它前端继续使用各自配置，不能将这组实录调参当作通用噪声标定。
    #[must_use]
    pub fn for_point_lio() -> Self {
        Self {
            max_loop_mse: 0.01,
            min_loop_overlap_ratio: 0.7,
            loop_confirmation_frames: 3,
            loop_confirmation_interval_secs: 2.0,
            min_loop_separation_secs: 60.0,
            min_loop_interval_secs: 60.0,
            stationary_probe_interval_secs: 5.0,
            icp: IcpConfig {
                max_correspondence_distance: 0.3,
                ..IcpConfig::default()
            },
            ..Self::default()
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
    odometry_covariance: Matrix6<f64>,
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
    /// 独立原始里程计闭合检查的马氏距离平方。
    pub odometry_innovation: f64,
}

struct PendingLoop {
    records: Vec<LoopClosureRecord>,
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
    pending_loop: Option<PendingLoop>,
    search_diagnostics: LoopSearchDiagnostics,
    last_search_report_timestamp: Option<f64>,
}

#[derive(Default)]
struct LoopSearchDiagnostics {
    searches: usize,
    candidates: usize,
    registration_failed: usize,
    geometry_rejected: usize,
    odometry_rejected: usize,
    best_source_overlap: f64,
    best_target_overlap: f64,
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
            pending_loop: None,
            search_diagnostics: LoopSearchDiagnostics::default(),
            last_search_report_timestamp: None,
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
            let confirmation_due = self.pending_loop.is_some()
                && timestamp - last.timestamp >= self.config.loop_confirmation_interval_secs;
            let stationary_probe_due = self.keyframes.len() >= self.config.min_keyframes_for_loop_search
                && timestamp - last.timestamp >= self.config.stationary_probe_interval_secs;
            if !confirmation_due
                && !stationary_probe_due
                && delta.translation.vector.norm() < self.config.keyframe_translation
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
        let mut odometry_covariance = Matrix6::zeros();
        if let Some((previous_id, previous_pose)) = previous {
            let delta = previous_pose.inverse() * local_pose;
            // LIO没有提供关键帧增量协方差时采用随运动增长的保守噪声模型。
            // 不能把垂直/旋转固定成近乎无误差，否则回环无法纠正长期累计漂移。
            let distance = delta.translation.vector.norm();
            let translation_sigma = self.config.odometry_translation_sigma_m * (1.0 + distance);
            let rotation_sigma = self.config.odometry_rotation_sigma_rad * (1.0 + distance)
                + 0.05 * delta.rotation.angle();
            let translation_information = translation_sigma.powi(-2);
            let rotation_information = rotation_sigma.powi(-2);
            odometry_covariance = Matrix6::from_diagonal(&nalgebra::Vector6::new(
                translation_sigma.powi(2),
                translation_sigma.powi(2),
                translation_sigma.powi(2),
                rotation_sigma.powi(2),
                rotation_sigma.powi(2),
                rotation_sigma.powi(2),
            ));
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
                context: ScanContext::new(points, local_pose.rotation, self.config.scan_context),
                odometry_covariance,
            },
        );

        let loop_result = self.find_and_measure_loop(node_id);
        let confirmed = self.confirm_loop(loop_result, timestamp);
        let result = if let Some(records) = confirmed {
            let mut optimized_graph = self.graph.clone();
            for record in &records {
                optimized_graph.add_edge(
                    PoseEdge::new(record.from, record.to, record.measurement)
                        .with_information(record.information),
                );
            }
            let gravity = self.gravity_priors();
            let initial_cost =
                graph_cost(&optimized_graph) + gravity_cost(&optimized_graph, &gravity);
            let historic_id = records.last().unwrap().from;
            let final_cost = PgoOptimizer::with_config(self.config.optimizer)
                .optimize_with_gravity(&mut optimized_graph, &gravity);
            if cost_strictly_improved(initial_cost, final_cost)
                && has_valid_correction(&self.graph, &optimized_graph)
            {
                tracing::info!(event = "pgo.loop.confirmed", node_id, historic_id,
                    constraints = records.len(), initial_cost, final_cost,
                    "独立回环观测已确认，提交完整优化轨迹供规范地图校正");
                self.graph = optimized_graph;
                self.loops.extend(records);
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
        self.pending_loop = None;
        self.search_diagnostics = LoopSearchDiagnostics::default();
        self.last_search_report_timestamp = None;
    }

    fn find_and_measure_loop(&mut self, current_id: u64) -> Option<LoopClosureRecord> {
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
        self.search_diagnostics.searches += 1;
        self.search_diagnostics.candidates += candidates.len();
        let mut measurements = Vec::new();

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
                        self.search_diagnostics.registration_failed += 1;
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
                self.search_diagnostics.registration_failed += 1;
                tracing::debug!(event = "pgo.registration.rejected", current_id, historic_id,
                    source_timestamp = current.timestamp,
                    converged = registration.converged, mse = registration.mse,
                    iterations = registration.iterations, max_mse = self.config.max_loop_mse,
                    "回环求解未收敛或求解残差超限");
                continue;
            }
            // 子图用于稳定法线与求解；独立双向覆盖必须比较当前帧和候选锚帧。
            // 将多帧子图的全部区域当作单帧视野分母，会随历史增长拒绝真实重访。
            let evidence = evaluate_loop_registration(
                &seeded_points,
                &self.keyframes[&historic_id].points,
                &registration.transformation,
                if self.config.point_to_plane {
                    self.config.icp.max_correspondence_distance.min(0.3)
                } else {
                    self.config.icp.max_correspondence_distance
                },
            );
            if evidence.source_overlap.min(evidence.target_overlap)
                > self.search_diagnostics.best_source_overlap.min(self.search_diagnostics.best_target_overlap)
            {
                self.search_diagnostics.best_source_overlap = evidence.source_overlap;
                self.search_diagnostics.best_target_overlap = evidence.target_overlap;
            }
            if let Some(reason) = evidence.rejection_reason(
                self.config.min_keyframe_points,
                self.config.min_loop_overlap_ratio,
                self.config.max_loop_mse,
            ) {
                self.search_diagnostics.geometry_rejected += 1;
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
            // 对应点相关，最多按64个独立支撑计权；置信度受配置下限约束。
            // 用几何力臂把米平方换成弧度平方，不直接将匹配误差充作角度方差。
            let support = evidence
                .source_inliers
                .min(evidence.unique_target_inliers)
                .min(64) as f64;
            let position_variance = (evidence.mse / support)
                .max(self.config.loop_translation_sigma_floor_m.powi(2));
            let rotation_variance = (position_variance / evidence.lever_arm_m2.max(0.25))
                .max(self.config.loop_rotation_sigma_floor_rad.powi(2));
            let information = Matrix6::from_diagonal(&nalgebra::Vector6::new(
                position_variance.recip(),
                position_variance.recip(),
                position_variance.recip(),
                rotation_variance.recip(),
                rotation_variance.recip(),
                rotation_variance.recip(),
            ));
            let mut record = LoopClosureRecord {
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
                odometry_innovation: 0.0,
            };
            record.odometry_innovation = self.loop_innovation(&record);
            tracing::info!(
                event = "pgo.loop.measured",
                current_id,
                historic_id,
                mse = record.mse,
                source_overlap = record.source_overlap,
                target_overlap = record.target_overlap,
                measurement_translation_m = ?record.measurement.translation.vector,
                measurement_rotation_xyzw = ?record.measurement.rotation.coords,
                innovation = record.odometry_innovation,
                "记录回环配准与里程计一致性依据"
            );
            if record.odometry_innovation > self.config.max_loop_innovation {
                self.search_diagnostics.odometry_rejected += 1;
                tracing::info!(
                    event = "pgo.loop.rejected",
                    reason = "odometry_inconsistent",
                    current_id,
                    historic_id,
                    innovation = record.odometry_innovation,
                    "点云匹配与独立里程计路径不一致"
                );
                continue;
            }
            measurements.push(record);
        }
        // 扩大检索不能把重复房间里的第一个低误差解当作唯一地点。
        // 同次观测的候选也须沿原里程计协方差相互闭合；歧义时保留原图等待新观察。
        let ambiguous = measurements.iter().enumerate().any(|(index, first)| {
            measurements.iter().skip(index + 1).any(|second|
                self.pair_innovation(first, second) > self.config.max_loop_innovation)
        });
        let measured = if ambiguous {
            tracing::info!(event = "pgo.loop.rejected", reason = "ambiguous_geometry", current_id,
                candidates = measurements.len(), "多个历史地点的几何匹配相互矛盾，等待新的独立观测");
            None
        } else {
            measurements.into_iter().min_by(|a, b| a.mse.total_cmp(&b.mse))
        };
        if measured.is_some() || self.last_search_report_timestamp
            .is_none_or(|previous| current.timestamp - previous >= 5.0)
        {
            let report = std::mem::take(&mut self.search_diagnostics);
            tracing::info!(event = "pgo.loop.search_summary", current_id,
                source_timestamp = current.timestamp, source_points = current.points.len(),
                searches = report.searches, candidates = report.candidates,
                registration_failed = report.registration_failed,
                geometry_rejected = report.geometry_rejected, odometry_rejected = report.odometry_rejected,
                best_source_overlap = report.best_source_overlap, best_target_overlap = report.best_target_overlap,
                "回环搜索与几何拒绝汇总");
            self.last_search_report_timestamp = Some(current.timestamp);
        }
        measured
    }

    /// 在配置预算内合并空间和描述子初值，保留近邻及较早锚点。
    fn loop_candidates(&self, current_id: u64) -> Vec<(u64, Isometry3<f64>)> {
        let current = &self.keyframes[&current_id];
        let current_pose = self.graph.nodes[&current_id].pose;
        let mut spatial = Vec::new();
        let mut descriptors = Vec::new();
        let mut tail = (Isometry3::identity(), Matrix6::zeros());
        let mut next = current;
        // 从当前节点向前一次累积既有里程计噪声；检索和最终一致性使用同一模型。
        for (id, historic) in self.keyframes.range(..current_id).rev() {
            tail = compose_uncertainty(
                (historic.local_pose.inverse() * next.local_pose, next.odometry_covariance), tail,
            );
            next = historic;
            if current.timestamp - historic.timestamp < self.config.min_loop_separation_secs {
                continue;
            }
            let distance = (current_pose.translation.vector
                - self.graph.nodes[id].pose.translation.vector)
                .norm();
            let uncertainty_radius = (self.config.max_loop_innovation
                * tail.1.fixed_view::<3, 3>(0, 0).trace().max(0.0)).sqrt();
            if distance <= self.config.loop_search_radius.max(uncertainty_radius) {
                spatial.push((distance, *id));
            }
            descriptors.push((current.context.ring_distance(&historic.context), *id));
        }
        spatial.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        descriptors.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        let spatial_seed = |distance: f64, id: u64| {
            let mut seed = self.graph.nodes[&id].pose.inverse() * current_pose;
            if distance > self.config.loop_search_radius {
                // 累计漂移超过局部配准捕获范围时，用两帧质心提供平移初值。
                // 里程计保留朝向初值；几何、独立覆盖及一致性检验仍全部执行。
                let center = |points: &[Point3<f64>]| points.iter()
                    .fold(nalgebra::Vector3::zeros(), |sum, p| sum + p.coords) / points.len() as f64;
                seed.translation = Translation3::from(center(&self.keyframes[&id].points)
                    - seed.rotation * center(&current.points));
            }
            (id, seed)
        };
        let mut candidates: Vec<_> = spatial.first()
            .map(|(distance, id)| spatial_seed(*distance, *id)).into_iter().collect();
        // 同一地点相邻历史帧不能占满预算，保留一个更早的锚点以发现整段累计漂移。
        if let Some((distance, id)) = spatial.iter().min_by_key(|(_, id)| *id) {
            if candidates.len() < self.config.loop_candidate_count
                && candidates.iter().all(|(existing, _)| existing != id) {
                candidates.push(spatial_seed(*distance, *id));
            }
        }
        let spatial_count = candidates.len();
        let mut descriptor_checked = 0;
        let mut matches: Vec<_> = descriptors
            .iter()
            .filter(|_| self.config.scan_context_enabled)
            .take(self.config.scan_context.candidate_count)
            .filter_map(|(_, id)| {
                descriptor_checked += 1;
                let alignment = current
                    .context
                    .align_to(&self.keyframes[id].context, self.config.scan_context);
                tracing::debug!(event = "pgo.sc.alignment", current_id, historic_id = *id,
                    source_timestamp = current.timestamp,
                    historic_timestamp = self.keyframes[id].timestamp,
                    accepted = alignment.is_ok(), reason = alignment.as_ref().err().copied().unwrap_or("accepted"),
                    score = alignment.as_ref().map(|value| value.0).ok(),
                    "扫描上下文航向检索结果");
                alignment.ok().map(|(score, yaw)| (score, *id, yaw))
            })
            .collect();
        let descriptor_accepted = matches.len();
        matches.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        for (_, id, yaw) in matches {
            if candidates.len() >= self.config.loop_candidate_count {
                break;
            }
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
            if candidates.len() == self.config.loop_candidate_count {
                break;
            }
        }
        let descriptor_selected = candidates.len() - spatial_count;
        let spatial_available = spatial.len();
        for (distance, id) in spatial {
            if candidates.len() >= self.config.loop_candidate_count {
                break;
            }
            if candidates.iter().all(|(existing, _)| *existing != id) {
                candidates.push(spatial_seed(distance, id));
            }
        }
        tracing::debug!(event = "pgo.loop.retrieval", current_id,
            source_timestamp = current.timestamp, spatial_available,
            descriptor_available = descriptors.len(), descriptor_checked, descriptor_accepted,
            descriptor_selected, candidates = candidates.len(),
            "记录空间与扫描上下文检索预算及召回");
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

    fn gravity_priors(&self) -> Vec<(u64, nuwax_gtsam::GravityPrior)> {
        let Some(sigma) = self.config.gravity_prior_sigma_rad else {
            return Vec::new();
        };
        self.keyframes
            .iter()
            .map(|(&id, frame)| {
                (
                    id,
                    nuwax_gtsam::GravityPrior {
                        direction_body: (frame.local_pose.rotation.inverse()
                            * nalgebra::Vector3::z())
                        .into(),
                        sigma,
                    },
                )
            })
            .collect()
    }

    /// 右扰动协方差随原始里程计增量传播，不能用已优化路径自证回环。
    fn odometry_between(&self, from: u64, to: u64) -> Option<(Isometry3<f64>, Matrix6<f64>)> {
        if from == to {
            self.keyframes.get(&from)?;
            return Some((Isometry3::identity(), Matrix6::zeros()));
        }
        if from > to {
            let (pose, covariance) = self.odometry_between(to, from)?;
            let adjoint = se3_adjoint(&pose);
            return Some((pose.inverse(), adjoint * covariance * adjoint.transpose()));
        }
        let mut pose = Isometry3::identity();
        let mut covariance = Matrix6::zeros();
        let mut previous = self.keyframes.get(&from)?.local_pose;
        for (_, frame) in self.keyframes.range(from.saturating_add(1)..=to) {
            let delta = previous.inverse() * frame.local_pose;
            (pose, covariance) =
                compose_uncertainty((pose, covariance), (delta, frame.odometry_covariance));
            previous = frame.local_pose;
        }
        Some((pose, covariance))
    }

    fn loop_innovation(&self, record: &LoopClosureRecord) -> f64 {
        let Some((prediction, covariance)) = self.odometry_between(record.from, record.to) else {
            return f64::INFINITY;
        };
        let Some(measurement_covariance) = record.information.try_inverse() else {
            return f64::INFINITY;
        };
        innovation_squared(
            &record.measurement,
            &prediction,
            covariance + measurement_covariance,
        )
    }

    fn pair_innovation(&self, first: &LoopClosureRecord, second: &LoopClosureRecord) -> f64 {
        let Some(historic) = self.odometry_between(second.from, first.from) else {
            return f64::INFINITY;
        };
        let Some(current) = self.odometry_between(first.to, second.to) else {
            return f64::INFINITY;
        };
        let (Some(a), Some(b)) = (
            first.information.try_inverse(),
            second.information.try_inverse(),
        ) else {
            return f64::INFINITY;
        };
        let (prediction, covariance) = compose_uncertainty(
            compose_uncertainty(historic, (first.measurement, a)),
            current,
        );
        innovation_squared(&second.measurement, &prediction, covariance + b)
    }

    fn confirm_loop(
        &mut self,
        record: Option<LoopClosureRecord>,
        timestamp: f64,
    ) -> Option<Vec<LoopClosureRecord>> {
        if self.pending_loop.as_ref().is_some_and(|pending| {
            timestamp - self.keyframes[&pending.records.last().unwrap().to].timestamp > 10.0
        }) {
            self.pending_loop = None;
        }
        let record = record?;
        if self.config.loop_confirmation_frames == 1 {
            return Some(vec![record]);
        }
        let consistent = self.pending_loop.as_ref().is_some_and(|pending| {
            pending.records.iter().all(|previous| {
                self.pair_innovation(previous, &record) <= self.config.max_loop_innovation
            })
        });
        if !consistent {
            tracing::info!(
                event = "pgo.loop.awaiting_confirmation",
                current_id = record.to,
                historic_id = record.from,
                replaced = self.pending_loop.is_some(),
                "等待间隔观测确认回环"
            );
            self.pending_loop = Some(PendingLoop {
                records: vec![record],
            });
            return None;
        }
        let pending = self.pending_loop.as_mut().unwrap();
        if timestamp - self.keyframes[&pending.records.last().unwrap().to].timestamp
            < self.config.loop_confirmation_interval_secs
        {
            return None;
        }
        pending.records.push(record);
        if pending.records.len() < self.config.loop_confirmation_frames {
            return None;
        }
        Some(self.pending_loop.take().unwrap().records)
    }
}

fn compose_uncertainty(
    first: (Isometry3<f64>, Matrix6<f64>),
    second: (Isometry3<f64>, Matrix6<f64>),
) -> (Isometry3<f64>, Matrix6<f64>) {
    let adjoint = se3_adjoint(&second.0.inverse());
    (
        first.0 * second.0,
        adjoint * first.1 * adjoint.transpose() + second.1,
    )
}

fn innovation_squared(
    measured: &Isometry3<f64>,
    predicted: &Isometry3<f64>,
    covariance: Matrix6<f64>,
) -> f64 {
    let error = se3_log_to_vector6(&(measured.inverse() * predicted));
    let symmetric = (covariance + covariance.transpose()) * 0.5;
    let value = symmetric
        .cholesky()
        .map_or(f64::INFINITY, |factor| error.dot(&factor.solve(&error)));
    if value.is_finite() && value >= 0.0 {
        value
    } else {
        f64::INFINITY
    }
}

fn gravity_cost(graph: &PoseGraph, priors: &[(u64, nuwax_gtsam::GravityPrior)]) -> f64 {
    priors
        .iter()
        .map(|(id, prior)| {
            let error = graph.nodes[id].pose.rotation
                * nalgebra::Vector3::from(prior.direction_body)
                - nalgebra::Vector3::z();
            0.5 * error.norm_squared() / prior.sigma.powi(2)
        })
        .sum()
}

fn validate_config(config: LoopClosurePipelineConfig) -> Result<(), LoopClosurePipelineError> {
    if !config.scan_context.is_valid() || !(1..=64).contains(&config.loop_candidate_count) {
        return Err(LoopClosurePipelineError::InvalidConfig("扫描上下文或配准候选预算无效"));
    }
    if config.loop_confirmation_frames == 0
        || config.loop_confirmation_frames > 8
        || !config.max_loop_innovation.is_finite()
        || config.max_loop_innovation <= 0.0
        || !config.loop_confirmation_interval_secs.is_finite()
        || config.loop_confirmation_interval_secs <= 0.0
        || config
            .gravity_prior_sigma_rad
            .is_some_and(|sigma| !sigma.is_finite() || sigma <= 0.0)
        || !config.odometry_translation_sigma_m.is_finite()
        || config.odometry_translation_sigma_m <= 0.0
        || !config.odometry_rotation_sigma_rad.is_finite()
        || config.odometry_rotation_sigma_rad <= 0.0
        || !config.loop_translation_sigma_floor_m.is_finite()
        || config.loop_translation_sigma_floor_m <= 0.0
        || !config.loop_rotation_sigma_floor_rad.is_finite()
        || config.loop_rotation_sigma_floor_rad <= 0.0
    {
        return Err(LoopClosurePipelineError::InvalidConfig(
            "invalid loop consistency or gravity configuration",
        ));
    }
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
    if !config.stationary_probe_interval_secs.is_finite() || config.stationary_probe_interval_secs <= 0.0 {
        return Err(LoopClosurePipelineError::InvalidConfig("静止回环观测间隔必须为正有限数"));
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
    fn point_lio_profile_preserves_accurate_room_and_corrects_supported_drift() {
        // 三个不同法向的合成墙面提供完整几何约束；分别验证准确前端与已知缓慢漂移。
        let world: Vec<_> = (-20..=20)
            .flat_map(|a| {
                (-10..=20).flat_map(move |b| {
                    let u = a as f64 * 0.1;
                    let v = b as f64 * 0.1;
                    [
                        Point3::new(4.0, u, v),
                        Point3::new(u + 1.0, 3.0, v),
                        Point3::new(u + 1.0, v, -1.0),
                    ]
                })
            })
            .collect();
        for drift in [0.0, 0.2] {
            let mut owner =
                LoopClosureCoordinator::new(LoopClosurePipelineConfig::for_point_lio()).unwrap();
            let mut raw = Isometry3::identity();
            for step in 0..=24 {
                let x = if step <= 10 {
                    step as f64 * 0.5
                } else {
                    (20 - step.min(20)) as f64 * 0.5
                };
                let truth = Isometry3::translation(x, 0.0, 0.0);
                raw = Isometry3::translation(x + drift * step as f64 / 24.0, 0.0, 0.0);
                let points: Vec<_> = world.iter().map(|p| truth.inverse() * p).collect();
                owner
                    .process_frame(step as f64 * 10.0, raw, &points)
                    .unwrap();
            }
            let corrected = owner.corrections().correct_pose(240.0, &raw).unwrap();
            // 准确输入保持厘米内，已知漂移至少减半；末段新增漂移仍可能等待下一轮回环。
            let error_limit = if drift == 0.0 { 0.01 } else { drift * 0.5 };
            assert!(
                corrected.translation.vector.norm() < error_limit,
                "漂移={drift}，校正={corrected:?}"
            );
            if drift > 0.0 {
                assert!(owner.loops().len() >= 3, "真实配准必须形成足够的回环约束");
            }
        }
    }

    fn consistency_fixture() -> LoopClosureCoordinator {
        let mut owner = LoopClosureCoordinator::new(LoopClosurePipelineConfig {
            min_loop_separation_secs: 10_000.0,
            keyframe_translation: 0.1,
            ..Default::default()
        })
        .unwrap();
        let points: Vec<_> = (0..20)
            .map(|i| Point3::new(i as f64 * 0.1, (i % 3) as f64 * 0.2, (i % 5) as f64 * 0.2))
            .collect();
        for i in 0..=11 {
            owner
                .process_frame(
                    i as f64,
                    Isometry3::translation(i as f64 * 0.5, 0., 0.),
                    &points,
                )
                .unwrap();
        }
        owner
    }

    fn measured_loop(to: u64, x: f64, yaw: f64) -> LoopClosureRecord {
        LoopClosureRecord {
            from: 0,
            to,
            measurement: Isometry3::from_parts(
                Translation3::new(x, 0., 0.),
                UnitQuaternion::from_euler_angles(0., 0., yaw),
            ),
            mse: 0.022755389991151544,
            source_inliers: 102,
            target_inliers: 117,
            source_overlap: 0.593,
            target_overlap: 0.672,
            information: Matrix6::from_diagonal(&nalgebra::Vector6::new(
                2500., 2500., 2500., 10000., 10000., 10000.,
            )),
            odometry_innovation: 0.,
        }
    }

    #[test]
    fn short_odometry_path_rejects_large_angle_even_with_low_registration_error() {
        let owner = consistency_fixture();
        let bad = measured_loop(10, 5., 41.1338_f64.to_radians());
        assert!(owner.loop_innovation(&bad) > owner.config.max_loop_innovation);
        let good = measured_loop(10, 4.9, 0.);
        assert!(owner.loop_innovation(&good) < owner.config.max_loop_innovation);
        assert_eq!(owner.odometry_between(0, 0).unwrap().1, Matrix6::zeros());
    }

    #[test]
    fn single_or_inconsistent_loop_never_rewrites_graph_and_reset_discards_votes() {
        let mut owner = consistency_fixture();
        let edges = owner.graph.num_edges();
        assert!(owner
            .confirm_loop(Some(measured_loop(9, 4.4, 0.)), 9.)
            .is_none());
        assert!(owner.loops.is_empty());
        assert_eq!(owner.graph.num_edges(), edges);
        assert!(owner
            .confirm_loop(Some(measured_loop(10, 5., 0.5)), 10.)
            .is_none());
        assert_eq!(owner.pending_loop.as_ref().unwrap().records.len(), 1);
        assert!(owner
            .confirm_loop(Some(measured_loop(10, 4.9, 0.)), 10.)
            .is_none());
        let confirmed = owner
            .confirm_loop(Some(measured_loop(11, 5.4, 0.)), 11.)
            .unwrap();
        assert_eq!(confirmed.len(), 2);
        owner.confirm_loop(Some(measured_loop(10, 4.9, 0.)), 10.);
        owner.reset();
        assert!(owner.pending_loop.is_none());
    }

    #[test]
    fn uncertainty_composition_matches_finite_difference_in_rotated_translated_frame() {
        use super::super::se3::se3_exp_from_vector6;
        let a = Isometry3::from_parts(
            Translation3::new(2., -3., 1.),
            UnitQuaternion::from_euler_angles(0.2, -0.3, 0.6),
        );
        let b = Isometry3::from_parts(
            Translation3::new(-1., 2., 0.5),
            UnitQuaternion::from_euler_angles(-0.1, 0.4, -0.2),
        );
        let qa = Matrix6::from_diagonal(&nalgebra::Vector6::new(
            0.01, 0.02, 0.03, 0.001, 0.002, 0.003,
        ));
        let qb = Matrix6::identity() * 0.001;
        let (_, actual) = compose_uncertainty((a, qa), (b, qb));
        let mut jacobian = Matrix6::zeros();
        for i in 0..6 {
            let mut dx = nalgebra::Vector6::zeros();
            dx[i] = 1e-6;
            let perturbed = a * se3_exp_from_vector6(&dx) * b;
            jacobian.set_column(
                i,
                &(se3_log_to_vector6(&((a * b).inverse() * perturbed)) / 1e-6),
            );
        }
        assert!((actual - (jacobian * qa * jacobian.transpose() + qb)).norm() < 1e-7);
    }

    #[test]
    fn gravity_prior_resists_map_tilt_while_allowing_yaw_correction() {
        let mut graph = PoseGraph::new();
        for x in [0., 1., 0.2] {
            graph.add_node(0, Isometry3::translation(x, 0., 0.));
        }
        graph.fix_first_node();
        graph.add_edge(
            PoseEdge::new(0, 1, Isometry3::translation(1., 0., 0.))
                .with_information(Matrix6::identity() * 1000.),
        );
        graph.add_edge(
            PoseEdge::new(1, 2, Isometry3::translation(-0.8, 0., 0.))
                .with_information(Matrix6::identity() * 1000.),
        );
        graph.add_edge(
            PoseEdge::new(
                0,
                2,
                Isometry3::from_parts(
                    Translation3::new(0., 0., 0.),
                    UnitQuaternion::from_euler_angles(0.1, 0., 0.2),
                ),
            )
            .with_information(Matrix6::identity() * 100.),
        );
        let mut unguarded = graph.clone();
        let _ = PgoOptimizer::default().optimize(&mut unguarded);
        let priors: Vec<_> = (0..3)
            .map(|id| {
                (
                    id,
                    nuwax_gtsam::GravityPrior {
                        direction_body: [0., 0., 1.],
                        sigma: 0.005,
                    },
                )
            })
            .collect();
        let cost = PgoOptimizer::default().optimize_with_gravity(&mut graph, &priors);
        let (roll, _, yaw) = graph.nodes[&2].pose.rotation.euler_angles();
        assert!(roll.abs() < 0.002 && yaw.abs() > 0.01, "{roll} {yaw}");
        assert!(unguarded.nodes[&2].pose.rotation.euler_angles().0.abs() > roll.abs() * 5.);
        assert!((cost - graph_cost(&graph) - gravity_cost(&graph, &priors)).abs() < 1e-7);
        assert_eq!(graph.nodes[&0].pose, Isometry3::identity());
    }

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
            // 本例明确注入强噪声里程计，图边和一致性检查共同使用该不确定性。
            odometry_translation_sigma_m: 0.2,
            submap_resolution_m: 0.01,
            ..Default::default()
        })
        .unwrap();
        let mut last = ProcessFrameResult::InsufficientPoints;
        let mut raw = Isometry3::identity();
        let mut truth = Isometry3::identity();
        for step in 0..=31 {
            let i = step.min(30);
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
                .process_frame(
                    if step >= 30 {
                        1000.0 + (step - 30) as f64 * 1.1
                    } else {
                        i as f64
                    },
                    raw,
                    &body,
                )
                .unwrap();
        }
        assert!(
            matches!(last, ProcessFrameResult::LoopClosed { historic_id: 0, .. }),
            "{last:?}"
        );
        let corrected = coordinator
            .corrections()
            .correct_pose(1001.1, &raw)
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
        for budget in [1, 2, 8] {
            coordinator.config.loop_candidate_count = budget;
            let candidates = coordinator.loop_candidates(current_id);
            assert!(!candidates.is_empty());
            assert!(candidates.len() <= budget);
        }
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
