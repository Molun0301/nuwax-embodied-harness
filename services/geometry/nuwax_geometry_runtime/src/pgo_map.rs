//! PGO扫描归档与私有地图重建；同一地图拥有方负责归档、提交和后续有效变换。

use crate::{
    lio_ingress::RegistrationBinding,
    mapping_backend::{
        self, MappingBackendComputeConfig, MappingBackendInput, MappingBackendKind,
        MappingBackendSpec, MappingComputeToken,
    },
    persistent_occupancy::PersistentOccupancyGrid,
    pointcloud::{CloudMapShmWriter, CommittedCloudMapSnapshot},
};
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion};
use nuwax_codec::lcm::native_pgo::{validate_optimized_keyframes, OptimizedKeyframe};
use nuwax_mapping::{
    loop_closure::{CorrectionKeyframe, CorrectionTimeline},
    OccupancyGrid,
};
use std::{
    collections::BTreeMap,
    sync::Arc,
    time::{Duration, Instant},
};

const MAX_ARCHIVED_POINTS: usize = 8_000_000;
pub(crate) const REBUILD_BUDGET: Duration = Duration::from_secs(3);
// 后台准备按小段让出计算资源；累计历史耗时不再成为丢弃回环的条件。
pub(crate) const REPLAY_SLICE: Duration = Duration::from_millis(8);
pub(crate) const REPLAY_YIELD: Duration = Duration::from_millis(2);
// 时间片仍是首要边界，轻帧不再因16帧上限反复休眠；每帧继续检查取消。
const REPLAY_SLICE_MAX_FRAMES: usize = 128;

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub(crate) enum RebuildError {
    #[error("PGO地图重建已取消")]
    Cancelled,
    #[error("PGO地图重建超过计算预算")]
    BudgetExceeded,
    #[error("PGO缺少完整原始扫描历史，不能替换当前地图")]
    HistoryUnavailable,
    #[error("{0}")]
    Invalid(String),
}

impl From<String> for RebuildError {
    fn from(error: String) -> Self {
        Self::Invalid(error)
    }
}

impl From<&str> for RebuildError {
    fn from(error: &str) -> Self {
        Self::Invalid(error.into())
    }
}

#[derive(Clone)]
pub(crate) struct ArchivedScan {
    pub source_timestamp_ns: u64,
    pub received_at_ns: u64,
    pub odom_from_body: Isometry3<f64>,
    pub sensor_origin_body: Point3<f64>,
    pub body_points: Arc<[Point3<f64>]>,
    envelope_sha256: [u8; 32],
}

#[derive(Default)]
pub(crate) struct ScanHistory {
    scans: BTreeMap<u64, ArchivedScan>,
    point_count: usize,
    incomplete: bool,
    dynamic_scene: crate::dynamic_scene::DynamicScene,
    latest_static_scan: Option<(u64, [u8; 32], usize, Arc<[u32]>)>,
}

impl ScanHistory {
    pub fn clear(&mut self) {
        self.scans.clear();
        self.point_count = 0;
        self.incomplete = false;
        self.dynamic_scene = Default::default();
        self.latest_static_scan = None;
    }

    pub fn record(
        &mut self,
        source_timestamp_ns: u64,
        received_at_ns: u64,
        binding: &RegistrationBinding,
        points: &[Point3<f64>],
    ) -> Result<(), String> {
        if let Some(existing) = self.scans.get(&source_timestamp_ns) {
            return if existing.envelope_sha256 == binding.envelope_sha256 { Ok(()) }
                else { Err("PGO历史扫描来源冲突".into()) };
        }
        if let Some((stamp, digest, _, _)) = &self.latest_static_scan {
            if source_timestamp_ns <= *stamp {
                return if source_timestamp_ns == *stamp && *digest == binding.envelope_sha256 {
                    Ok(())
                } else {
                    Err("PGO扫描来源倒序或冲突".into())
                };
            }
        }
        let body_from_map =
            (binding.registration_map_from_odom * binding.frozen_odom_from_body).inverse();
        let body_points: Vec<_> = points.iter().map(|point| body_from_map * point).collect();
        let sensor_origin = binding.sensor_origin_map.map(|p| body_from_map * Point3::from(p));
        self.dynamic_scene.clear_frame();
        if let Some(origin) = sensor_origin {
            // 用未受回环跳变影响的里程计坐标积累运动证据；每个已接纳扫描都参与。
            let odom_points = body_points.iter().map(|point| binding.frozen_odom_from_body * point).collect::<Vec<_>>();
            let odom_origin = binding.frozen_odom_from_body * origin;
            self.dynamic_scene.observe(&odom_points, odom_origin.coords.into(), source_timestamp_ns);
        }
        let static_indices = body_points.iter().enumerate().filter_map(|(index, point)| {
            let p = binding.frozen_odom_from_body * point;
            (!self.dynamic_scene.is_dynamic([p.x, p.y, p.z])).then_some(index as u32)
        }).collect::<Vec<_>>();
        self.latest_static_scan = Some((source_timestamp_ns, binding.envelope_sha256, points.len(), static_indices.into()));
        if self.incomplete { return Ok(()); }
        if self.point_count.saturating_add(points.len()) > MAX_ARCHIVED_POINTS {
            tracing::warn!(event = "pgo.history.exhausted", archived_points = self.point_count,
                incoming_points = points.len(), "PGO归档容量耗尽，本会话停止完整历史校正，正常建图继续");
            self.scans.clear();
            self.point_count = 0;
            self.incomplete = true;
            return Ok(());
        }
        let sensor_origin_body = sensor_origin.unwrap_or_else(Point3::origin);
        self.point_count += body_points.len();
        self.scans.insert(
            source_timestamp_ns,
            ArchivedScan {
                source_timestamp_ns,
                received_at_ns,
                odom_from_body: binding.frozen_odom_from_body,
                sensor_origin_body,
                body_points: body_points.into(),
                envelope_sha256: binding.envelope_sha256,
            },
        );
        Ok(())
    }

    pub(crate) fn static_point_indices(&self, timestamp_ns: u64, digest: [u8; 32], count: usize) -> Result<Arc<[u32]>, String> {
        let (stamp, source_digest, source_count, indices) = self.latest_static_scan.as_ref()
            .ok_or("PGO静态扫描尚未准备")?;
        if (*stamp, *source_digest, *source_count) != (timestamp_ns, digest, count) {
            return Err("PGO静态扫描与当前来源不一致".into());
        }
        Ok(Arc::clone(indices))
    }

    pub fn latest_timestamp_ns(&self) -> Option<u64> {
        self.scans.last_key_value().map(|(stamp, _)| *stamp)
    }

    pub fn snapshot_after(&self, after: u64) -> Result<Vec<ArchivedScan>, RebuildError> {
        if self.incomplete {
            return Err(RebuildError::HistoryUnavailable);
        }
        use std::ops::Bound::{Excluded, Unbounded};
        Ok(self
            .scans
            .range((Excluded(after), Unbounded))
            .map(|(_, scan)| scan.clone())
            .collect())
    }

    pub fn snapshot(&self) -> Result<Vec<ArchivedScan>, RebuildError> {
        if self.incomplete {
            return Err(RebuildError::HistoryUnavailable);
        }
        Ok(self.scans.values().cloned().collect())
    }
}

pub(crate) struct RebuiltMap {
    pub occupancy: OccupancyGrid,
    pub cloud: CommittedCloudMapSnapshot,
    pub backend_checkpoint: Vec<u8>,
    pub accumulator: crate::pointcloud::PreparedCloudAccumulator,
    pub source_timestamp_ns: u64,
    pub prepared_backend: Option<mapping_backend::PreparedMappingBackendRestore>,
}

pub(crate) fn optimized_pose(frame: &OptimizedKeyframe) -> Isometry3<f64> {
    let q = frame.rotation_xyzw;
    Isometry3::from_parts(
        Translation3::from(nalgebra::Vector3::from(frame.translation_m)),
        UnitQuaternion::new_normalize(nalgebra::Quaternion::new(q[3], q[0], q[1], q[2])),
    )
}

/// 私有重放拥有方保留内核和动态反证；追赶只追加新扫描，不反复重算历史前缀。
pub(crate) struct ReplayBuilder {
    kernel: mapping_backend::MappingKernel,
    spec: MappingBackendSpec,
    persistent: Option<PersistentOccupancyGrid>,
    cloud: Option<CloudMapShmWriter>,
    cloud_worker: Option<CloudStageWorker>,
    timeline: CorrectionTimeline,
    anchor: Isometry3<f64>,
    first_timestamp_ns: i64,
    frame_id: String,
    config: MappingBackendComputeConfig,
    last_input: Option<Arc<MappingBackendInput>>,
    processed: usize,
    started: Instant,
    budget: Duration,
    background: bool,
    background_parallel_available: bool,
    #[cfg(test)]
    force_parallel_lanes: bool,
    kernel_time: Duration,
    cloud_time: Duration,
}

pub(crate) struct PreparedReplay {
    occupancy: OccupancyGrid,
    cloud: CommittedCloudMapSnapshot,
    backend_checkpoint: Vec<u8>,
    pub source_timestamp_ns: u64,
    prepared_backend: Option<mapping_backend::PreparedMappingBackendRestore>,
}

impl ReplayBuilder {
    pub(crate) fn new(
        scans: &[ArchivedScan],
        trajectory: Vec<OptimizedKeyframe>,
        anchor: Isometry3<f64>,
        spec: MappingBackendSpec,
        config: MappingBackendComputeConfig,
        frame_id: String,
        budget: Duration,
        cancelled: &impl Fn() -> bool,
    ) -> Result<Self, RebuildError> {
        let started = Instant::now();
        check_budget(started, budget, cancelled)?;
        validate_optimized_keyframes(&trajectory)?;
        let first = trajectory
            .first()
            .ok_or("PGO结果缺少完整优化轨迹")?
            .source_timestamp_ns;
        if !scans
            .windows(2)
            .all(|pair| pair[0].source_timestamp_ns < pair[1].source_timestamp_ns)
        {
            return Err("PGO归档扫描必须按真实源时间严格递增".into());
        }
        let mut keyframes = Vec::with_capacity(trajectory.len());
        for node in &trajectory {
            check_budget(started, budget, cancelled)?;
            let frame = scans
                .binary_search_by_key(&(node.source_timestamp_ns as u64), |s| {
                    s.source_timestamp_ns
                })
                .ok()
                .map(|index| &scans[index])
                .ok_or("PGO轨迹引用了未保存的原始扫描")?;
            keyframes.push(CorrectionKeyframe {
                timestamp: (node.source_timestamp_ns - first) as f64 / 1e9,
                local_pose: frame.odom_from_body,
                optimized_pose: optimized_pose(node),
            });
        }
        let first_delta = keyframes[0].local_pose.inverse() * keyframes[0].optimized_pose;
        if first_delta.translation.vector.norm() > 1e-6 || first_delta.rotation.angle() > 1e-6 {
            return Err("PGO优化改变了固定首节点".into());
        }
        let timeline = CorrectionTimeline::new(keyframes).map_err(|e| e.to_string())?;
        let cloud = CloudMapShmWriter::new_private().map_err(|e| e.to_string())?;
        Ok(Self {
            kernel: spec.into_kernel(config.window_capacity),
            spec,
            persistent: None,
            cloud: Some(cloud),
            cloud_worker: None,
            timeline,
            anchor,
            first_timestamp_ns: first,
            frame_id,
            config,
            last_input: None,
            processed: 0,
            started,
            budget,
            background: false,
            background_parallel_available: false,
            #[cfg(test)]
            force_parallel_lanes: false,
            kernel_time: Duration::ZERO,
            cloud_time: Duration::ZERO,
        })
    }

    pub(crate) fn check(&self, cancelled: &impl Fn() -> bool) -> Result<(), RebuildError> {
        if self.background {
            if cancelled() {
                Err(RebuildError::Cancelled)
            } else {
                Ok(())
            }
        } else {
            check_budget(self.started, self.budget, cancelled)
        }
    }

    pub(crate) fn into_background(mut self) -> Self {
        self.background = true;
        self.background_parallel_available =
            std::thread::available_parallelism().is_ok_and(|count| count.get() >= 4);
        self
    }

    /// 每个切片至少推进一帧；单次内核调用不可抢占，片间由异步拥有方让出资源。
    pub(crate) fn append_slice(
        mut self,
        scans: &[ArchivedScan],
        cancelled: &impl Fn() -> bool,
    ) -> Result<(Self, usize), RebuildError> {
        let started = Instant::now();
        let mut processed = 0;
        for scan in scans.iter().take(REPLAY_SLICE_MAX_FRAMES) {
            self.append(std::slice::from_ref(scan), cancelled)?;
            processed += 1;
            if started.elapsed() >= REPLAY_SLICE {
                break;
            }
        }
        // 回收在阻塞工作内完成，异步拥有方跨片只保留本地点云，不承担线程等待。
        self.recover_cloud_worker()?;
        Ok((self, processed))
    }

    pub(crate) fn latest_timestamp_ns(&self) -> u64 {
        self.last_input
            .as_ref()
            .map_or(0, |input| input.source_timestamp_ns)
    }

    pub(crate) fn advance(
        mut self,
        scans: &[ArchivedScan],
        cancelled: &impl Fn() -> bool,
    ) -> Result<(Self, PreparedReplay), RebuildError> {
        let result = self
            .append(scans, cancelled)
            .and_then(|()| self.prepare_inner(cancelled, true));
        match result {
            Ok(prepared) => Ok((self, prepared)),
            Err(error) => {
                if let Some(cloud) = self.cloud.as_mut() {
                    cloud
                        .cleanup_product_owned_file()
                        .map_err(|e| format!("PGO临时点云清理失败：{e}"))?;
                }
                Err(error)
            }
        }
    }

    pub(crate) fn append(
        &mut self,
        scans: &[ArchivedScan],
        cancelled: &impl Fn() -> bool,
    ) -> Result<(), RebuildError> {
        for scan in scans {
            self.check(cancelled)?;
            if self
                .last_input
                .as_ref()
                .is_some_and(|input| scan.source_timestamp_ns <= input.source_timestamp_ns)
            {
                return Err("PGO追赶扫描与已处理前缀重叠或回退".into());
            }
            let time =
                (scan.source_timestamp_ns as i128 - self.first_timestamp_ns as i128) as f64 / 1e9;
            // 轨迹尾部之后复用最后一个已求解校正，原始新里程计继续按真实时间推进。
            let map_from_body = self.anchor
                * self
                    .timeline
                    .correct_pose(time, &scan.odom_from_body)
                    .map_err(|e| e.to_string())?;
            let points: Vec<_> = scan
                .body_points
                .iter()
                .map(|point| map_from_body * point)
                .collect();
            let origin = map_from_body * scan.sensor_origin_body;
            let input = Arc::new(MappingBackendInput {
                session_id: "pgo-private-rebuild".into(),
                session_generation: 1,
                backend_generation: 1,
                frame_sequence: self.processed as u64 + 1,
                source_timestamp_ns: scan.source_timestamp_ns,
                received_at_ns: scan.received_at_ns,
                registered_frame_id: self.frame_id.clone(),
                sensor_origin_map: origin.coords.into(),
                registered_points_map: points,
            });
            let validation_started = Instant::now();
            mapping_backend::validate_backend_input(&self.kernel, &input)?;
            self.kernel_time += validation_started.elapsed();
            let use_parallel = self.cloud_worker.is_some()
                || (self.background
                    && self.background_parallel_available
                    && self.processed >= 8
                    && self.kernel_time.as_nanos() / self.processed as u128 >= 250_000
                    && self.cloud_time.as_nanos() / self.processed as u128 >= 250_000);
            #[cfg(test)]
            let use_parallel = use_parallel || (self.background && self.force_parallel_lanes);
            if !use_parallel || !self.append_parallel_stages(&input, cancelled)? {
                let phase = Instant::now();
                append_kernel_stage(
                    &mut self.kernel,
                    &mut self.persistent,
                    &input,
                    self.config,
                    self.background,
                )?;
                self.kernel_time += phase.elapsed();
                self.check(cancelled)?;
                let phase = Instant::now();
                append_cloud_stage(self.cloud.as_mut().expect("本地点云已保留"), &input)?;
                self.cloud_time += phase.elapsed();
            }
            self.processed += 1;
            self.last_input = Some(input);
        }
        self.check(cancelled)
    }

    /// 复用本次重建的点云工作线程，同一帧只有二维和三维两路在途计算。
    fn append_parallel_stages(
        &mut self,
        input: &Arc<MappingBackendInput>,
        cancelled: &impl Fn() -> bool,
    ) -> Result<bool, RebuildError> {
        if self.cloud_worker.is_none() {
            let cloud = self.cloud.take().expect("启动前点云由本地拥有");
            match CloudStageWorker::start(cloud) {
                Ok(worker) => self.cloud_worker = Some(worker),
                Err(cloud) => {
                    self.cloud = Some(cloud);
                    self.background_parallel_available = false;
                    return Ok(false);
                }
            }
        }
        self.cloud_worker.as_mut().unwrap().submit(input.clone())?;
        let started = Instant::now();
        // 禁止两阶段并行中再启动高度投影线程，最多两路计算。
        let kernel_result = append_kernel_stage(
            &mut self.kernel,
            &mut self.persistent,
            input,
            self.config,
            false,
        );
        self.kernel_time += started.elapsed();
        let was_cancelled = cancelled();
        let reply = self.cloud_worker.as_mut().unwrap().receive()?;
        self.cloud_time += reply.elapsed;
        if reply.source_timestamp_ns != input.source_timestamp_ns
            || reply.frame_sequence != input.frame_sequence
        {
            return Err("PGO点云阶段回复与当前扫描不匹配".into());
        }
        kernel_result?;
        if was_cancelled {
            return Err(RebuildError::Cancelled);
        }
        reply.result?;
        self.check(cancelled)?;
        Ok(true)
    }

    fn recover_cloud_worker(&mut self) -> Result<(), RebuildError> {
        if let Some(worker) = self.cloud_worker.take() {
            self.cloud = Some(worker.finish()?);
        }
        Ok(())
    }

    /// 全量物化与后端编码仍在后台；只有已经物化的相同尾部才能进入最终提交。
    pub(crate) fn prepare(
        &mut self,
        cancelled: &impl Fn() -> bool,
    ) -> Result<PreparedReplay, RebuildError> {
        self.prepare_inner(cancelled, false)
    }

    fn prepare_inner(
        &mut self,
        cancelled: &impl Fn() -> bool,
        typed_restore: bool,
    ) -> Result<PreparedReplay, RebuildError> {
        self.check(cancelled)?;
        self.recover_cloud_worker()?;
        let input = self.last_input.as_ref().ok_or("PGO重建没有扫描")?;
        let token = MappingComputeToken {
            token_id: 1,
            session_id: input.session_id.clone(),
            session_generation: 1,
            backend_generation: 1,
            source_frame_sequence: input.frame_sequence,
            source_timestamp_ns: input.source_timestamp_ns,
            prepared_at_ns: input.received_at_ns,
            base_occupancy_revision: 0,
        };
        // 物化候选不能改变继续追赶的节拍或持久栅格；否则分批边界会改变同输入结果。
        let mut output_kernel = self.kernel.clone();
        let occupancy = if let Some(candidate) =
            mapping_backend::flush_candidate(&mut output_kernel, input, &token, self.config)?
        {
            match candidate.backend_kind {
                MappingBackendKind::SlamToolbox => {
                    return Err("二维算法不得进入三维回环重建".into())
                }
                MappingBackendKind::Voxel => PersistentOccupancyGrid::stage_voxel(
                    self.persistent.as_ref(),
                    &candidate.live_window_occupancy,
                    &candidate.latest_frame_occupancy,
                )?,
                MappingBackendKind::Raytracing => {
                    PersistentOccupancyGrid::stage_authoritative(&candidate.live_window_occupancy)?
                }
            }
            .grid
        } else {
            self.persistent
                .as_ref()
                .ok_or("PGO重建没有占据地图")?
                .grid
                .clone()
        };
        self.check(cancelled)?;
        self.cloud
            .as_mut()
            .expect("物化前点云线程已回收")
            .commit_accumulated()
            .ok_or("PGO重建点云无法完成快照")?;
        let (backend_checkpoint, prepared_backend) = if typed_restore {
            (
                Vec::new(),
                Some(mapping_backend::prepare_computed_mapping_backend_restore(
                    self.spec,
                    self.config,
                    output_kernel,
                )?),
            )
        } else {
            (
                mapping_backend::export_mapping_kernel_checkpoint(&output_kernel)?,
                None,
            )
        };
        let prepared = PreparedReplay {
            occupancy,
            cloud: self
                .cloud
                .as_ref()
                .expect("物化前点云线程已回收")
                .committed_snapshot()
                .ok_or("PGO重建点云快照缺失")?,
            backend_checkpoint,
            source_timestamp_ns: input.source_timestamp_ns,
            prepared_backend,
        };
        self.check(cancelled)?;
        Ok(prepared)
    }

    pub(crate) fn finish(
        mut self,
        prepared: PreparedReplay,
        cancelled: &impl Fn() -> bool,
    ) -> Result<RebuiltMap, RebuildError> {
        self.check(cancelled)?;
        if prepared.source_timestamp_ns != self.latest_timestamp_ns() {
            return Err("PGO准备快照落后于私有重放尾部".into());
        }
        if self.cloud_worker.is_some() {
            return Err("PGO点云线程尚未在后台回收".into());
        }
        let result = RebuiltMap {
            occupancy: prepared.occupancy,
            cloud: prepared.cloud,
            backend_checkpoint: prepared.backend_checkpoint,
            accumulator: self
                .cloud
                .as_mut()
                .expect("提交前点云已回收")
                .take_prepared_accumulator(),
            source_timestamp_ns: prepared.source_timestamp_ns,
            prepared_backend: prepared.prepared_backend,
        };
        self.cloud
            .as_mut()
            .expect("提交前点云已回收")
            .cleanup_product_owned_file()
            .map_err(|e| format!("PGO临时点云清理失败：{e}"))?;
        self.check(cancelled)?;
        tracing::info!(
            event = "pgo.map.background_ready",
            frames = self.processed,
            kernel_ms = self.kernel_time.as_millis() as u64,
            cloud_ms = self.cloud_time.as_millis() as u64,
            elapsed_ms = self.started.elapsed().as_millis() as u64,
            background = self.background,
            "PGO私有地图已完整准备"
        );
        Ok(result)
    }
}

struct CloudStageReply {
    source_timestamp_ns: u64,
    frame_sequence: u64,
    result: Result<(), RebuildError>,
    elapsed: Duration,
}

/// 每次重建独立拥有；关闭回复端后再等待，避免未领取回执阻塞退出。
struct CloudStageWorker {
    input: Option<std::sync::mpsc::SyncSender<Arc<MappingBackendInput>>>,
    replies: Option<std::sync::mpsc::Receiver<CloudStageReply>>,
    join: Option<std::thread::JoinHandle<Result<CloudMapShmWriter, RebuildError>>>,
}

impl CloudStageWorker {
    fn start(cloud: CloudMapShmWriter) -> Result<Self, CloudMapShmWriter> {
        let (initialize, initialized) = std::sync::mpsc::sync_channel::<CloudMapShmWriter>(1);
        let (input, requests) = std::sync::mpsc::sync_channel::<Arc<MappingBackendInput>>(1);
        let (replies, receiver) = std::sync::mpsc::sync_channel(1);
        let join = std::thread::Builder::new()
            .name("pgo-cloud".into())
            .spawn(move || {
                let mut cloud = initialized
                    .recv()
                    .map_err(|_| RebuildError::Invalid("PGO点云初始化通道关闭".into()))?;
                while let Ok(input) = requests.recv() {
                    let started = Instant::now();
                    let result = append_cloud_stage(&mut cloud, &input);
                    let reply = CloudStageReply {
                        source_timestamp_ns: input.source_timestamp_ns,
                        frame_sequence: input.frame_sequence,
                        result,
                        elapsed: started.elapsed(),
                    };
                    if replies.send(reply).is_err() {
                        break;
                    }
                }
                Ok(cloud)
            });
        let Ok(join) = join else {
            return Err(cloud);
        };
        if let Err(error) = initialize.send(cloud) {
            drop(input);
            drop(receiver);
            let _ = join.join();
            return Err(error.0);
        }
        Ok(Self {
            input: Some(input),
            replies: Some(receiver),
            join: Some(join),
        })
    }

    fn submit(&mut self, input: Arc<MappingBackendInput>) -> Result<(), RebuildError> {
        self.input
            .as_ref()
            .ok_or("PGO点云工作已关闭")?
            .send(input)
            .map_err(|_| RebuildError::Invalid("PGO点云工作线程已退出".into()))
    }

    fn receive(&mut self) -> Result<CloudStageReply, RebuildError> {
        self.replies
            .as_ref()
            .ok_or("PGO点云回复已关闭")?
            .recv()
            .map_err(|_| RebuildError::Invalid("PGO点云工作线程未交付结果".into()))
    }

    fn finish(mut self) -> Result<CloudMapShmWriter, RebuildError> {
        drop(self.input.take());
        drop(self.replies.take());
        self.join
            .take()
            .expect("点云线程由本对象拥有")
            .join()
            .map_err(|_| RebuildError::Invalid("PGO点云工作线程异常退出".into()))?
    }
}

impl Drop for CloudStageWorker {
    fn drop(&mut self) {
        drop(self.input.take());
        drop(self.replies.take());
        if let Some(join) = self.join.take() {
            if let Ok(Ok(mut cloud)) = join.join() {
                let _ = cloud.cleanup_product_owned_file();
            }
        }
    }
}

fn append_kernel_stage(
    kernel: &mut mapping_backend::MappingKernel,
    persistent: &mut Option<PersistentOccupancyGrid>,
    input: &MappingBackendInput,
    config: MappingBackendComputeConfig,
    parallel_projection: bool,
) -> Result<(), RebuildError> {
    if let Some(candidate) = mapping_backend::compute_candidate_in_place_with_interval(
        kernel,
        input,
        config,
        mapping_backend::REPLAY_CANDIDATE_INTERVAL_NS,
        parallel_projection,
    )? {
        match candidate.backend_kind {
            MappingBackendKind::SlamToolbox => return Err("二维算法不得进入三维回环重建".into()),
            MappingBackendKind::Voxel => {
                if persistent.is_none() {
                    *persistent = Some(PersistentOccupancyGrid::from_grid(
                        candidate.live_window_occupancy.clone(),
                    )?);
                }
                persistent.as_mut().unwrap().merge(
                    &candidate.live_window_occupancy,
                    &candidate.latest_frame_occupancy,
                )?;
            }
            MappingBackendKind::Raytracing => {
                *persistent = Some(PersistentOccupancyGrid::stage_authoritative(
                    &candidate.live_window_occupancy,
                )?);
            }
        }
    }
    Ok(())
}

fn append_cloud_stage(
    cloud: &mut CloudMapShmWriter,
    input: &MappingBackendInput,
) -> Result<(), RebuildError> {
    if !cloud.accumulate_mapping_scan(
        1,
        &input.registered_points_map,
        Some(input.sensor_origin_map),
        input.source_timestamp_ns,
    ) {
        return Err("PGO重建点云无法接纳原始扫描".into());
    }
    Ok(())
}

fn check_budget(
    started: Instant,
    budget: Duration,
    cancelled: &impl Fn() -> bool,
) -> Result<(), RebuildError> {
    if cancelled() {
        Err(RebuildError::Cancelled)
    } else if started.elapsed() >= budget {
        Err(RebuildError::BudgetExceeded)
    } else {
        Ok(())
    }
}

/// 同步受管调用仍要求冻结扫描尾部；后台追赶复用同一个私有重放实现。
pub(crate) fn rebuild(
    scans: Vec<ArchivedScan>,
    trajectory: Vec<OptimizedKeyframe>,
    anchor: Isometry3<f64>,
    spec: MappingBackendSpec,
    config: MappingBackendComputeConfig,
    frame_id: String,
    budget: Duration,
    cancelled: impl Fn() -> bool,
) -> Result<RebuiltMap, RebuildError> {
    let started = Instant::now();
    let frames = scans.len();
    let mut processed = 0;
    let mut kernel_time = Duration::ZERO;
    let mut cloud_time = Duration::ZERO;
    let result = (|| {
        check_budget(started, budget, &cancelled)?;
        if scans.last().map(|scan| scan.source_timestamp_ns)
            != trajectory
                .last()
                .map(|node| node.source_timestamp_ns as u64)
        {
            return Err("PGO轨迹与已接纳扫描尾部不一致".into());
        }
        let mut builder = ReplayBuilder::new(
            &scans, trajectory, anchor, spec, config, frame_id, budget, &cancelled,
        )?;
        let appended = builder.append(&scans, &cancelled);
        processed = builder.processed;
        kernel_time = builder.kernel_time;
        cloud_time = builder.cloud_time;
        match appended.and_then(|()| builder.prepare(&cancelled)) {
            Ok(prepared) => builder.finish(prepared, &cancelled),
            Err(error) => {
                if let Some(cloud) = builder.cloud.as_mut() {
                    cloud
                        .cleanup_product_owned_file()
                        .map_err(|e| format!("PGO临时点云清理失败：{e}"))?;
                }
                Err(error)
            }
        }
    })();
    tracing::info!(event = "pgo.map.rebuilt", frames, processed,
        kernel_ms = kernel_time.as_millis() as u64, cloud_ms = cloud_time.as_millis() as u64,
        elapsed_ms = started.elapsed().as_millis() as u64, success = result.is_ok(),
        error = ?result.as_ref().err(), budget_ms = budget.as_millis() as u64,
        "PGO私有地图重建结束");
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn archive_capacity_disables_only_complete_history_and_resets_with_session() {
        let binding = RegistrationBinding {
            runtime_generation: 1,
            cloud_source_sequence: 1,
            correlated_odometry: crate::lio_ingress::CorrelatedOdometry {
                source_sequence: 1,
                source_timestamp_ns: 1,
            },
            mapping_session_id: "bounded-history".into(),
            mapping_session_generation: 1,
            map_frame_generation: 1,
            envelope_sha256: [1; 32],
            frozen_odom_from_body: Isometry3::identity(),
            base_anchor_map_from_odom: Isometry3::identity(),
            registration_map_from_odom: Isometry3::identity(),
            registration_correction_revision: 0,
            sensor_origin_map: Some([0.0; 3]),
            frame_capture_relocalization_epoch: None,
        };
        let mut history = ScanHistory::default();
        let points = [Point3::new(1.0, 2.0, 3.0)];
        history.record(1, 1, &binding, &points).unwrap();
        assert_eq!(history.snapshot().unwrap().len(), 1);
        // 只注入容量计数，避免为边界回归分配数百兆字节数据。
        history.point_count = MAX_ARCHIVED_POINTS;
        history.record(2, 2, &binding, &points).unwrap();
        assert!(matches!(
            history.snapshot(),
            Err(RebuildError::HistoryUnavailable)
        ));
        assert_eq!(history.point_count, 0);
        assert!(history.scans.is_empty());
        history.record(3, 3, &binding, &points).unwrap();
        assert!(matches!(
            history.snapshot(),
            Err(RebuildError::HistoryUnavailable)
        ));
        history.clear();
        history.record(4, 4, &binding, &points).unwrap();
        assert_eq!(history.snapshot().unwrap()[0].source_timestamp_ns, 4);
    }

    #[test]
    fn cloud_stage_stop_joins_even_with_unclaimed_reply() {
        let cloud = CloudMapShmWriter::new_private().unwrap();
        let mut worker =
            CloudStageWorker::start(cloud).unwrap_or_else(|_| panic!("无法创建测试点云线程"));
        for sequence in 1..=2 {
            worker
                .submit(Arc::new(MappingBackendInput {
                    session_id: "private-test".into(),
                    session_generation: 1,
                    backend_generation: 1,
                    frame_sequence: sequence,
                    source_timestamp_ns: sequence,
                    received_at_ns: sequence,
                    registered_frame_id: "map".into(),
                    sensor_origin_map: [0.0; 3],
                    registered_points_map: vec![Point3::new(sequence as f64, 0.0, 0.0)],
                }))
                .unwrap();
        }
        // 回复端未领取也必须能关闭，不能在等待线程时被已满回复通道卡住。
        let mut cloud = worker.finish().unwrap();
        assert!(cloud.commit_accumulated().unwrap().1 >= 1);
    }

    fn scale_input(frame_count: usize) -> (Vec<ArchivedScan>, Vec<OptimizedKeyframe>) {
        let world: Vec<_> = (0..41)
            .flat_map(|x| {
                (0..41)
                    .map(move |y| Point3::new(-4.0 + x as f64 * 0.2, -4.0 + y as f64 * 0.2, -1.0))
            })
            .chain((0..41).flat_map(|y| {
                (0..16).map(move |z| Point3::new(4.0, -4.0 + y as f64 * 0.2, -1.0 + z as f64 * 0.2))
            }))
            .chain((0..41).flat_map(|x| {
                (0..16).map(move |z| Point3::new(-4.0 + x as f64 * 0.2, 4.0, -1.0 + z as f64 * 0.2))
            }))
            .collect();
        let scans = (0..frame_count as u64)
            .map(|i| {
                let pose = Isometry3::translation(i as f64 / 150.0, 0.0, 0.0);
                ArchivedScan {
                    source_timestamp_ns: 1_000_000_000 + i * 100_000_000,
                    received_at_ns: 1 + i * 150_000_000,
                    odom_from_body: pose,
                    sensor_origin_body: Point3::origin(),
                    body_points: world
                        .iter()
                        .map(|p| pose.inverse() * p)
                        .collect::<Vec<_>>()
                        .into(),
                    envelope_sha256: [1; 32],
                }
            })
            .collect::<Vec<_>>();
        let trajectory = [0, frame_count - 1]
            .map(|i| OptimizedKeyframe {
                source_timestamp_ns: scans[i].source_timestamp_ns as i64,
                translation_m: scans[i].odom_from_body.translation.vector.into(),
                rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
            })
            .to_vec();
        (scans, trajectory)
    }

    #[test]
    fn background_catchup_preserves_replay_cadence_maps_and_checkpoint() {
        let (scans, mut trajectory) = scale_input(31);
        let cutoff = 12;
        trajectory[1].source_timestamp_ns = scans[cutoff].source_timestamp_ns as i64;
        trajectory[1].translation_m = (scans[cutoff].odom_from_body.translation.vector
            + nalgebra::Vector3::new(-0.04, 0.03, 0.0))
        .into();
        let mut complete_trajectory = trajectory.clone();
        complete_trajectory.push(OptimizedKeyframe {
            source_timestamp_ns: scans.last().unwrap().source_timestamp_ns as i64,
            translation_m: (scans.last().unwrap().odom_from_body.translation.vector
                + nalgebra::Vector3::new(-0.04, 0.03, 0.0))
            .into(),
            rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
        });
        let config = MappingBackendComputeConfig {
            resolution: 0.05,
            occupancy_algo: nuwax_mapping::cost_mapper::CostAlgoSelector::HeightCost,
            initial_safe_radius_meters: 0.0,
            window_capacity: 10_000,
        };
        let anchor = Isometry3::translation(0.73, -1.15, 0.04);
        for spec in [
            MappingBackendSpec::Voxel,
            MappingBackendSpec::raytracing_defaults(),
        ] {
            let expected = rebuild(
                scans.clone(),
                complete_trajectory.clone(),
                anchor,
                spec,
                config,
                "map".into(),
                Duration::from_secs(10),
                || false,
            )
            .unwrap();
            let mut builder = ReplayBuilder::new(
                &scans[..=cutoff],
                trajectory.clone(),
                anchor,
                spec,
                config,
                "map".into(),
                Duration::from_secs(10),
                &|| false,
            )
            .unwrap();
            let mut start = 0;
            let mut prepared = None;
            for end in [13, 14, 19, 26, 31] {
                builder.append(&scans[start..end], &|| false).unwrap();
                prepared = Some(builder.prepare(&|| false).unwrap());
                start = end;
            }
            let actual = builder.finish(prepared.unwrap(), &|| false).unwrap();
            assert_eq!(
                actual.source_timestamp_ns,
                scans.last().unwrap().source_timestamp_ns
            );
            let a = &actual.occupancy;
            let b = &expected.occupancy;
            assert_eq!(
                (
                    a.width,
                    a.height,
                    a.resolution,
                    a.origin,
                    a.origin_orientation,
                    &a.frame_id,
                    &a.data
                ),
                (
                    b.width,
                    b.height,
                    b.resolution,
                    b.origin,
                    b.origin_orientation,
                    &b.frame_id,
                    &b.data
                ),
                "分批追赶不能改变占据结果"
            );
            assert_eq!(
                actual.cloud.xyz_f32_le, expected.cloud.xyz_f32_le,
                "动态清理与点云一致"
            );
            assert_eq!(
                actual.backend_checkpoint, expected.backend_checkpoint,
                "继续建图的内核状态一致"
            );
        }
    }

    fn rebuild_scale(
        frames: usize,
        budget: Duration,
        cancelled: impl Fn() -> bool,
    ) -> Result<RebuiltMap, RebuildError> {
        let (scans, trajectory) = scale_input(frames);
        rebuild(
            scans,
            trajectory,
            Isometry3::identity(),
            MappingBackendSpec::Voxel,
            MappingBackendComputeConfig {
                resolution: 0.05,
                occupancy_algo: nuwax_mapping::cost_mapper::CostAlgoSelector::HeightCost,
                initial_safe_radius_meters: 0.0,
                window_capacity: 100_000,
            },
            "map".into(),
            budget,
            cancelled,
        )
    }

    #[test]
    fn background_slices_preserve_progress_after_old_deadline_and_cancel() {
        let (scans, trajectory) = scale_input(48);
        let config = MappingBackendComputeConfig {
            resolution: 0.05,
            occupancy_algo: nuwax_mapping::cost_mapper::CostAlgoSelector::HeightCost,
            initial_safe_radius_meters: 0.0,
            window_capacity: 10_000,
        };
        for spec in [
            MappingBackendSpec::Voxel,
            MappingBackendSpec::raytracing_defaults(),
        ] {
            let expected = rebuild(
                scans.clone(),
                trajectory.clone(),
                Isometry3::identity(),
                spec,
                config,
                "map".into(),
                Duration::from_secs(30),
                || false,
            )
            .unwrap();
            let mut builder = ReplayBuilder::new(
                &scans,
                trajectory.clone(),
                Isometry3::identity(),
                spec,
                config,
                "map".into(),
                REBUILD_BUDGET,
                &|| false,
            )
            .unwrap()
            .into_background();
            builder.force_parallel_lanes = true;
            // 注入已经超过旧截止时间的历史年龄，核对继续推进而非清空后从头再来。
            builder.started = Instant::now() - Duration::from_secs(4);
            let mut cursor = 0;
            let mut slices = 0;
            while cursor < scans.len() {
                // 强制分批交付归档，验证进度跨批保留，不依赖机器速度或生产帧数上限。
                let end = (cursor + 16).min(scans.len());
                let (next, count) = builder
                    .append_slice(&scans[cursor..end], &|| false)
                    .unwrap();
                assert!((1..=REPLAY_SLICE_MAX_FRAMES).contains(&count));
                builder = next;
                assert!(builder.cloud_worker.is_none(), "工作线程不能越过阻塞计算片");
                assert!(builder.cloud.is_some());
                cursor += count;
                slices += 1;
                assert_eq!(builder.processed, cursor);
            }
            assert!(slices >= 3);
            let (builder, prepared) = builder.advance(&[], &|| false).unwrap();
            let actual = builder.finish(prepared, &|| false).unwrap();
            assert_eq!(actual.occupancy.data, expected.occupancy.data);
            assert_eq!(actual.cloud.xyz_f32_le, expected.cloud.xyz_f32_le);
            assert_eq!(actual.source_timestamp_ns, expected.source_timestamp_ns);
            assert!(actual.prepared_backend.is_some());
            let builder = ReplayBuilder::new(
                &scans,
                trajectory.clone(),
                Isometry3::identity(),
                spec,
                config,
                "map".into(),
                REBUILD_BUDGET,
                &|| false,
            )
            .unwrap()
            .into_background();
            assert!(matches!(
                builder.append_slice(&scans, &|| true),
                Err(RebuildError::Cancelled)
            ));
        }
    }

    #[tokio::test]
    #[ignore = "固定扰动扫描的串行/双阶段性能与完整地图等价对照"]
    async fn pgo_fixed_dynamic_stage_benchmark() {
        let (mut scans, trajectory) = scale_input(320);
        for (frame, scan) in scans.iter_mut().enumerate() {
            for (index, point) in Arc::make_mut(&mut scan.body_points).iter_mut().enumerate() {
                point.x += 0.08 * (frame as f64 * 0.19 + index as f64 * 0.03).sin();
                point.z += 0.08 * (frame as f64 * 0.17 + index as f64 * 0.02).cos();
            }
        }
        let scans: Arc<[ArchivedScan]> = scans.into();
        let config = MappingBackendComputeConfig {
            resolution: 0.05,
            occupancy_algo: nuwax_mapping::cost_mapper::CostAlgoSelector::HeightCost,
            initial_safe_radius_meters: 0.0,
            window_capacity: 1_000_000,
        };
        let mut reference: Option<RebuiltMap> = None;
        for parallel in [false, true] {
            let mut builder = ReplayBuilder::new(
                &scans,
                trajectory.clone(),
                Isometry3::identity(),
                MappingBackendSpec::Voxel,
                config,
                "map".into(),
                Duration::MAX,
                &|| false,
            )
            .unwrap()
            .into_background();
            builder.background_parallel_available = false;
            builder.force_parallel_lanes = parallel;
            let started = Instant::now();
            let mut cursor = 0;
            while cursor < scans.len() {
                let input = scans.clone();
                let (next, count) = tokio::task::spawn_blocking(move || {
                    builder.append_slice(&input[cursor..], &|| false)
                })
                .await
                .unwrap()
                .unwrap();
                builder = next;
                cursor += count;
                assert!(builder.cloud_worker.is_none());
                tokio::time::sleep(REPLAY_YIELD).await;
            }
            let kernel_ms = builder.kernel_time.as_secs_f64() * 1000.0;
            let cloud_ms = builder.cloud_time.as_secs_f64() * 1000.0;
            let output = tokio::task::spawn_blocking(move || {
                let (builder, prepared) = builder.advance(&[], &|| false).unwrap();
                builder.finish(prepared, &|| false).unwrap()
            })
            .await
            .unwrap();
            eprintln!("固定扰动 parallel={parallel} elapsed_ms={:.3} kernel_ms={kernel_ms:.3} cloud_ms={cloud_ms:.3} points={}",
                started.elapsed().as_secs_f64()*1000.0, output.cloud.point_count);
            if let Some(reference) = &reference {
                assert_eq!(output.occupancy.data, reference.occupancy.data);
                assert_eq!(output.occupancy.origin, reference.occupancy.origin);
                assert_eq!(
                    (output.occupancy.width, output.occupancy.height),
                    (reference.occupancy.width, reference.occupancy.height)
                );
                assert_eq!(output.cloud.xyz_f32_le, reference.cloud.xyz_f32_le);
                assert_eq!(output.source_timestamp_ns, reference.source_timestamp_ns);
            } else {
                reference = Some(output);
            }
        }
    }

    #[tokio::test]
    #[ignore = "长历史后台进度与片间调度基准，使用release构建独立执行"]
    async fn pgo_background_long_history_finishes_without_starving_executor() {
        let (scans, trajectory) = scale_input(2400);
        let scans: Arc<[ArchivedScan]> = scans.into();
        let started = Instant::now();
        let config = MappingBackendComputeConfig {
            resolution: 0.05,
            occupancy_algo: nuwax_mapping::cost_mapper::CostAlgoSelector::HeightCost,
            initial_safe_radius_meters: 0.0,
            window_capacity: 100_000,
        };
        let mut builder = ReplayBuilder::new(
            &scans,
            trajectory,
            Isometry3::identity(),
            MappingBackendSpec::Voxel,
            config,
            "map".into(),
            Duration::MAX,
            &|| false,
        )
        .unwrap()
        .into_background();
        let mut cursor = 0;
        let mut slices = 0;
        let stopped = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let tick_stop = stopped.clone();
        let ticker = tokio::spawn(async move {
            let mut previous = Instant::now();
            let mut max_gap = Duration::ZERO;
            let mut ticks = 0;
            while !tick_stop.load(std::sync::atomic::Ordering::Acquire) {
                tokio::time::sleep(Duration::from_millis(5)).await;
                max_gap = max_gap.max(previous.elapsed());
                previous = Instant::now();
                ticks += 1;
            }
            (max_gap, ticks)
        });
        while cursor < scans.len() {
            let input = scans.clone();
            let (next, count) = tokio::task::spawn_blocking(move || {
                builder.append_slice(&input[cursor..], &|| false)
            })
            .await
            .unwrap()
            .unwrap();
            builder = next;
            cursor += count;
            slices += 1;
            tokio::time::sleep(REPLAY_YIELD).await;
        }
        let kernel_ms = builder.kernel_time.as_secs_f64() * 1000.0;
        let cloud_ms = builder.cloud_time.as_secs_f64() * 1000.0;
        let materialize_started = Instant::now();
        let output = tokio::task::spawn_blocking(move || {
            let (builder, prepared) = builder.advance(&[], &|| false).unwrap();
            builder.finish(prepared, &|| false).unwrap()
        })
        .await
        .unwrap();
        let materialize_ms = materialize_started.elapsed().as_secs_f64() * 1000.0;
        eprintln!("后台阶段 kernel_ms={kernel_ms:.3} cloud_ms={cloud_ms:.3} materialize_ms={materialize_ms:.3}");
        assert_eq!(
            output.source_timestamp_ns,
            scans.last().unwrap().source_timestamp_ns
        );
        stopped.store(true, std::sync::atomic::Ordering::Release);
        let (max_gap, ticks) = ticker.await.unwrap();
        assert!(ticks > 10);
        assert!(
            max_gap < Duration::from_millis(100),
            "异步执行器被阻塞：{max_gap:?}"
        );
        eprintln!(
            "后台2400帧完成：elapsed_ms={} slices={} points={} tick_max_gap_ms={}",
            started.elapsed().as_millis(),
            slices,
            output.cloud.point_count,
            max_gap.as_secs_f64() * 1000.0
        );
    }

    #[test]
    #[ignore = "三百八十五帧规模基准，使用release构建独立执行"]
    fn pgo_rebuild_recorded_scale_budget() {
        rebuild_scale(385, REBUILD_BUDGET, || false).unwrap();
    }

    #[test]
    #[ignore = "接近归档容量的长历史规模基准，使用release构建独立执行"]
    fn pgo_rebuild_long_history_budget() {
        for frames in [770, 1540, 2400] {
            match rebuild_scale(frames, REBUILD_BUDGET, || false) {
                Ok(_) => eprintln!("长历史 frames={frames} 在正式预算内完成"),
                Err(RebuildError::BudgetExceeded) => {
                    eprintln!("长历史 frames={frames} 正式预算耗尽，完整结果未交付");
                    // 仅基准允许延长计算来测量全量成本；生产拥有方仍固定使用3秒预算。
                    rebuild_scale(frames, Duration::from_secs(30), || false).unwrap();
                }
                Err(error) => panic!("长历史重建出现非预算错误：{error}"),
            }
        }
    }

    #[test]
    fn pgo_rebuild_distinguishes_budget_and_mid_replay_cancellation() {
        assert!(matches!(
            rebuild_scale(8, Duration::ZERO, || false),
            Err(RebuildError::BudgetExceeded)
        ));
        let checks = std::cell::Cell::new(0);
        let cancelled = rebuild_scale(8, Duration::from_secs(30), || {
            checks.set(checks.get() + 1);
            checks.get() == 8
        });
        assert!(matches!(cancelled, Err(RebuildError::Cancelled)));
        // 已取消的私有重建不污染下一次重建的内核或临时点云。
        rebuild_scale(8, Duration::from_secs(30), || false).unwrap();
    }
}
