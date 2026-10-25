//! PGO扫描归档与私有地图重建；同一地图拥有方负责归档、提交和后续有效变换。

use crate::{
    lio_ingress::AdmittedRegisteredScan,
    mapping_backend::{
        self, MappingBackendComputeConfig, MappingBackendInput,
        MappingBackendSpec, MappingComputeToken,
    },
    persistent_occupancy::PersistentOccupancyGrid,
    pointcloud::{CloudMapShmWriter, CommittedCloudMapSnapshot},
};
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion};
use nuwax_codec::lcm::native_pgo::{validate_optimized_keyframes, OptimizedKeyframe};
use nuwax_mapping::{
    loop_closure::{CorrectionKeyframe, CorrectionTimeline},
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
    pub body_points: crate::correction_points::CorrectionPoints,
    source: ArchivedSource,
}

use crate::admitted_cloud::AdmittedCloudSource as ArchivedSource;

impl ArchivedSource {
    fn key(&self) -> ArchiveSourceKey {
        match self {
            Self::Lidar(_) => ArchiveSourceKey::Lidar,
            Self::Auxiliary(source) => { let (sensor, clock, sequence) = source.key();
                ArchiveSourceKey::Auxiliary(sensor, clock, sequence) }
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum ArchiveSourceKey {
    Lidar,
    Auxiliary(String, String, u64),
}

type ArchiveKey = (u64, ArchiveSourceKey);

impl ArchivedScan {
    fn key(&self) -> ArchiveKey { (self.source_timestamp_ns, self.source.key()) }
}

#[derive(Default)]
pub(crate) struct ScanHistory {
    scans: BTreeMap<ArchiveKey, ArchivedScan>,
    point_count: usize,
    incomplete: bool,
    dynamic_scene: crate::dynamic_scene::DynamicScene,
    latest_static_scan: Option<(u64, usize, crate::lio_ingress::WeakAdmittedRegisteredScan, Arc<[u32]>)>,
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
        source: &AdmittedRegisteredScan,
        received_at_ns: u64,
    ) -> Result<(), String> {
        let source_timestamp_ns = source.source_timestamp_ns();
        let binding = source.binding();
        if let Some(existing) = self.scans.get(&(source_timestamp_ns, ArchiveSourceKey::Lidar)) {
            return if existing.source.lidar() == Some(source) { Ok(()) }
                else { Err("PGO历史扫描来源冲突".into()) };
        }
        if let Some((stamp, count, latest, _)) = &self.latest_static_scan {
            if source_timestamp_ns <= *stamp {
                return if source_timestamp_ns == *stamp && source.body_points().len() == *count && latest.matches(source) { Ok(()) }
                    else { Err("PGO扫描来源倒序或冲突".into()) };
            }
        }
        let body_points = source.body_points();
        let body_from_map =
            (binding.registration_map_from_odom * binding.frozen_odom_from_body).inverse();
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
        self.latest_static_scan = Some((source_timestamp_ns, body_points.len(), source.downgrade(), static_indices.into()));
        self.archive(ArchivedScan {
            source_timestamp_ns, received_at_ns, odom_from_body: binding.frozen_odom_from_body,
            sensor_origin_body: sensor_origin.unwrap_or_else(Point3::origin),
            body_points: body_points.clone(), source: ArchivedSource::Lidar(source.clone()),
        })
    }

    pub(crate) fn record_auxiliary(&mut self,
        source: crate::lcm_runtime::mapping_observation::RegisteredObservation,
    ) -> Result<(), String> {
        self.archive(ArchivedScan {
            source_timestamp_ns: source.timestamp_ns(), received_at_ns: source.received_at_ns(),
            odom_from_body: source.odom_from_body(), sensor_origin_body: source.origin_body(),
            body_points: source.body_points().clone(), source: ArchivedSource::Auxiliary(source),
        })
    }

    fn archive(&mut self, scan: ArchivedScan) -> Result<(), String> {
        let key = scan.key();
        if let Some(existing) = self.scans.get(&key) {
            return if existing.source == scan.source { Ok(()) } else { Err("PGO归档来源冲突".into()) };
        }
        if self.incomplete { return Ok(()); }
        if self.point_count.saturating_add(scan.body_points.len()) > MAX_ARCHIVED_POINTS {
            tracing::warn!(event = "pgo.history.exhausted", archived_points = self.point_count,
                incoming_points = scan.body_points.len(), "PGO归档容量耗尽，本会话停止完整历史校正，正常建图继续");
            self.scans.clear(); self.point_count = 0; self.incomplete = true;
            return Ok(());
        }
        self.point_count += scan.body_points.len();
        self.scans.insert(key, scan);
        Ok(())
    }

    pub(crate) fn static_point_indices(&self, source: &AdmittedRegisteredScan) -> Result<Arc<[u32]>, String> {
        let (stamp, count, latest, indices) = self.latest_static_scan.as_ref()
            .ok_or("PGO静态扫描尚未准备")?;
        if source.source_timestamp_ns() != *stamp || source.body_points().len() != *count || !latest.matches(source) {
            return Err("PGO静态扫描与当前来源不一致".into());
        }
        Ok(Arc::clone(indices))
    }

    pub fn latest_timestamp_ns(&self) -> Option<u64> {
        self.scans.last_key_value().map(|((stamp, _), _)| *stamp)
    }

    pub fn snapshot_after(&self, after: u64) -> Result<Vec<ArchivedScan>, RebuildError> {
        if self.incomplete {
            return Err(RebuildError::HistoryUnavailable);
        }
        use std::ops::Bound::{Excluded, Unbounded};
        Ok(self
            .scans
            .range((Excluded((after, ArchiveSourceKey::Lidar)), Unbounded))
            .filter(|((stamp, _), _)| *stamp > after)
            .map(|(_, scan)| scan.clone())
            .collect())
    }

    #[cfg(test)]
    pub fn snapshot(&self) -> Result<Vec<ArchivedScan>, RebuildError> {
        if self.incomplete {
            return Err(RebuildError::HistoryUnavailable);
        }
        Ok(self.scans.values().cloned().collect())
    }
}

pub(crate) struct RebuiltMap {
    pub persistent: PersistentOccupancyGrid,
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
    last_archive_key: Option<ArchiveKey>,
    processed: usize,
    started: Instant,
    budget: Duration,
    background: bool,
    background_parallel_available: bool,
    #[cfg(test)]
    force_parallel_lanes: bool,
    #[cfg(test)]
    skip_ground_capture_for_cost_comparison: bool,
    kernel_time: Duration,
    cloud_time: Duration,
}

pub(crate) struct PreparedReplay {
    persistent: PersistentOccupancyGrid,
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
            .all(|pair| pair[0].key() < pair[1].key())
        {
            return Err("PGO归档扫描必须按真实源时间和独立来源排序".into());
        }
        let mut keyframes = Vec::with_capacity(trajectory.len());
        for node in &trajectory {
            check_budget(started, budget, cancelled)?;
            let stamp = node.source_timestamp_ns as u64;
            let start = scans.partition_point(|scan| scan.source_timestamp_ns < stamp);
            let frame = scans[start..].iter().take_while(|scan| scan.source_timestamp_ns == stamp)
                .find(|scan| scan.source.lidar().is_some())
                .ok_or("PGO轨迹引用了未保存的原始LiDAR扫描")?;
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
            last_archive_key: None,
            processed: 0,
            started,
            budget,
            background: false,
            background_parallel_available: false,
            #[cfg(test)]
            force_parallel_lanes: false,
            #[cfg(test)]
            skip_ground_capture_for_cost_comparison: false,
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
            if self.last_archive_key.as_ref().is_some_and(|key| scan.key() <= *key)
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
            let ground_source = scan.source.clone();
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
            if !use_parallel || !self.append_parallel_stages(&input, &ground_source, cancelled)? {
                let phase = Instant::now();
                append_kernel_stage(
                    &mut self.kernel,
                    &mut self.persistent,
                    &input,
                    self.config,
                )?;
                self.kernel_time += phase.elapsed();
                self.check(cancelled)?;
                let phase = Instant::now();
                let capture_source = Some(&ground_source);
                #[cfg(test)]
                let capture_source = capture_source.filter(|_| !self.skip_ground_capture_for_cost_comparison);
                append_cloud_stage(self.cloud.as_mut().expect("本地点云已保留"), &input, capture_source)?;
                self.cloud_time += phase.elapsed();
            }
            self.processed += 1;
            self.last_input = Some(input);
            self.last_archive_key = Some(scan.key());
        }
        self.check(cancelled)
    }

    /// 复用本次重建的点云工作线程，同一帧只有二维和三维两路在途计算。
    fn append_parallel_stages(
        &mut self,
        input: &Arc<MappingBackendInput>,
        ground_source: &ReplayedGroundSource,
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
        self.cloud_worker.as_mut().unwrap().submit(input.clone(), Some(ground_source.clone()))?;
        let started = Instant::now();
        // 地图与点云各自推进，最多两路计算。
        let kernel_result = append_kernel_stage(
            &mut self.kernel,
            &mut self.persistent,
            input,
            self.config,
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
        let persistent = if let Some(candidate) =
            mapping_backend::flush_candidate(&mut output_kernel, input, &token, self.config)?
        {
            PersistentOccupancyGrid::stage_observation(self.persistent.as_ref(), &candidate.latest_frame_occupancy)?
        } else {
            self.persistent
                .as_ref()
                .ok_or("PGO重建没有占据地图")?
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
            persistent,
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
            persistent: prepared.persistent,
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

type ReplayedGroundSource = ArchivedSource;

struct CloudStageReply {
    source_timestamp_ns: u64,
    frame_sequence: u64,
    result: Result<(), RebuildError>,
    elapsed: Duration,
}

/// 每次重建独立拥有；关闭回复端后再等待，避免未领取回执阻塞退出。
struct CloudStageWorker {
    input: Option<std::sync::mpsc::SyncSender<(Arc<MappingBackendInput>, Option<ReplayedGroundSource>)>>,
    replies: Option<std::sync::mpsc::Receiver<CloudStageReply>>,
    join: Option<std::thread::JoinHandle<Result<CloudMapShmWriter, RebuildError>>>,
}

impl CloudStageWorker {
    fn start(cloud: CloudMapShmWriter) -> Result<Self, CloudMapShmWriter> {
        let (initialize, initialized) = std::sync::mpsc::sync_channel::<CloudMapShmWriter>(1);
        let (input, requests) = std::sync::mpsc::sync_channel::<(Arc<MappingBackendInput>, Option<ReplayedGroundSource>)>(1);
        let (replies, receiver) = std::sync::mpsc::sync_channel(1);
        let join = std::thread::Builder::new()
            .name("pgo-cloud".into())
            .spawn(move || {
                let mut cloud = initialized
                    .recv()
                    .map_err(|_| RebuildError::Invalid("PGO点云初始化通道关闭".into()))?;
                while let Ok((input, ground_source)) = requests.recv() {
                    let started = Instant::now();
                    let result = append_cloud_stage(&mut cloud, &input, ground_source.as_ref());
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

    fn submit(&mut self, input: Arc<MappingBackendInput>, ground_source: Option<ReplayedGroundSource>) -> Result<(), RebuildError> {
        self.input
            .as_ref()
            .ok_or("PGO点云工作已关闭")?
            .send((input, ground_source))
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
) -> Result<(), RebuildError> {
    if let Some(candidate) = mapping_backend::compute_candidate_in_place_with_interval(
        kernel,
        input,
        config,
        0,
    )? {
        if let Some(grid) = persistent.as_mut() {
            grid.merge_observation(&candidate.latest_frame_occupancy)?;
        } else {
            *persistent = Some(PersistentOccupancyGrid::stage_observation(None, &candidate.latest_frame_occupancy)?);
        }
    }
    Ok(())
}

fn append_cloud_stage(
    cloud: &mut CloudMapShmWriter,
    input: &MappingBackendInput,
    ground_source: Option<&ReplayedGroundSource>,
) -> Result<(), RebuildError> {
    if ground_source.is_some_and(|s| s.timestamp_ns() != input.source_timestamp_ns) { return Err("PGO ground/source timestamp mismatch".into()); }
    let accepted = match ground_source {
        Some(source) => cloud.accumulate_mapping_replayed_scan(1, &input.registered_points_map,
            Some(input.sensor_origin_map), source),
        None => cloud.accumulate_mapping_scan(1, &input.registered_points_map,
            Some(input.sensor_origin_map), input.source_timestamp_ns),
    };
    if !accepted { return Err("PGO重建点云无法接纳原始扫描".into()); }
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
    use crate::lio_ingress::RegistrationBinding;

    #[test]
    fn planar_scan_replay_observes_each_frame_and_materialization_does_not_vote_again() {
        let config = MappingBackendComputeConfig {
            resolution:0.05,
            initial_safe_radius_meters:0.0, window_capacity:1000,
        };
        for spec in [MappingBackendSpec::Voxel, MappingBackendSpec::raytracing_defaults()] {
            let mut kernel = spec.into_kernel(config.window_capacity);
            let mut persistent = None;
            let mut input = MappingBackendInput {
                session_id:"planar-replay".into(),session_generation:1,backend_generation:1,
                frame_sequence:1,source_timestamp_ns:1_000_000_000,received_at_ns:1_000_000_000,
                registered_frame_id:"map".into(),sensor_origin_map:[0.025,0.025,0.0],
                registered_points_map:vec![Point3::new(2.025,0.025,0.0)],
            };
            append_kernel_stage(&mut kernel,&mut persistent,&input,config).unwrap();
            assert!(!persistent.as_ref().unwrap().grid.data.contains(&100));
            input.frame_sequence += 1;
            input.source_timestamp_ns += 50_000_000;
            input.received_at_ns += 50_000_000;
            append_kernel_stage(&mut kernel,&mut persistent,&input,config).unwrap();
            let grid = &persistent.as_ref().unwrap().grid;
            assert!(grid.data.contains(&100), "重放必须累积小于250毫秒间隔的真实扫描");
            let again = PersistentOccupancyGrid::stage_observation(persistent.as_ref(),grid).unwrap();
            assert_eq!(again.grid.data,grid.data);
        }
    }

    fn test_binding() -> RegistrationBinding {
        RegistrationBinding {
            runtime_generation: 1,
            cloud_source_sequence: 1,
            correlated_odometry: crate::lio_ingress::CorrelatedOdometry {
                source_sequence: 1,
                source_timestamp_ns: 1,
            },
            mapping_session_id: "bounded-history".into(),
            mapping_session_generation: 1,
            map_frame_generation: 1,
            frozen_odom_from_body: Isometry3::identity(),
            base_anchor_map_from_odom: Isometry3::identity(),
            registration_map_from_odom: Isometry3::identity(),
            registration_correction_revision: 0,
            sensor_origin_map: Some([0.0; 3]),
            frame_capture_relocalization_epoch: None,
        }
    }

    #[test]
    fn archive_capacity_disables_only_complete_history_and_resets_with_session() {
        let binding = test_binding();
        let mut history = ScanHistory::default();
        let points = [Point3::new(1.0, 2.0, 3.0)];
        history.record(&AdmittedRegisteredScan::from_map_points_for_test(1, binding.clone(), &points), 1).unwrap();
        assert_eq!(history.snapshot().unwrap().len(), 1);
        // 只注入容量计数，避免为边界回归分配数百兆字节数据。
        history.point_count = MAX_ARCHIVED_POINTS;
        history.record(&AdmittedRegisteredScan::from_map_points_for_test(2, binding.clone(), &points), 2).unwrap();
        assert!(matches!(
            history.snapshot(),
            Err(RebuildError::HistoryUnavailable)
        ));
        assert_eq!(history.point_count, 0);
        assert!(history.scans.is_empty());
        history.record(&AdmittedRegisteredScan::from_map_points_for_test(3, binding.clone(), &points), 3).unwrap();
        assert!(matches!(
            history.snapshot(),
            Err(RebuildError::HistoryUnavailable)
        ));
        history.clear();
        history.record(&AdmittedRegisteredScan::from_map_points_for_test(4, binding.clone(), &points), 4).unwrap();
        assert_eq!(history.snapshot().unwrap()[0].source_timestamp_ns, 4);
    }

    #[test]
    fn archive_and_correction_share_owner_points_until_their_last_retirement() {
        let source = AdmittedRegisteredScan::from_map_points_for_test(10, test_binding(), &[Point3::new(1.0, 2.0, 3.0)]);
        let retained = source.retention_probe_for_test();
        let points = source.body_points().clone();
        let allocation = points.as_ptr();
        let mut history = ScanHistory::default();
        history.record(&source, 11).unwrap();
        let replay = history.snapshot().unwrap();
        assert_eq!(replay[0].body_points.as_ptr(), allocation);
        assert_eq!(replay[0].source.lidar().unwrap().body_points().as_ptr(), allocation);
        let correction_points = source.body_points().clone();
        assert_eq!(correction_points.as_ptr(), allocation);
        drop(source);
        history.clear();
        assert!(retained(), "an active replay still owns the admitted source");
        drop(replay);
        assert!(!retained(), "cleared history and finished replay release source metadata");
        assert_eq!(correction_points.as_ptr(), allocation);
        assert_eq!(correction_points[0], Point3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn archive_duplicate_and_static_selection_require_the_complete_admitted_owner() {
        let binding = test_binding();
        let points = [Point3::new(1.0, 2.0, 3.0)];
        let source = AdmittedRegisteredScan::from_map_points_for_test(10, binding.clone(), &points);
        let independent = AdmittedRegisteredScan::from_map_points_for_test(10, binding.clone(), &points);
        let mut history = ScanHistory::default();
        history.record(&source, 11).unwrap();
        let indices = history.static_point_indices(&source).unwrap();
        history.record(&independent, 12).unwrap();
        assert_eq!(history.point_count, 1);
        assert_eq!(history.snapshot().unwrap()[0].received_at_ns, 11);
        assert!(Arc::ptr_eq(&indices, &history.static_point_indices(&independent).unwrap()));
        let mut conflicts = Vec::new();
        let mut changed = binding.clone(); changed.cloud_source_sequence += 1;
        conflicts.push(AdmittedRegisteredScan::from_map_points_for_test(10, changed, &points));
        let mut changed = binding.clone(); changed.base_anchor_map_from_odom.translation.vector.x = -0.0;
        conflicts.push(AdmittedRegisteredScan::from_map_points_for_test(10, changed, &points));
        conflicts.push(AdmittedRegisteredScan::from_map_points_for_test(10, binding.clone(), &[Point3::new(2.0, 2.0, 3.0)]));
        for conflict in conflicts {
            assert!(history.record(&conflict, 12).is_err());
            assert!(history.static_point_indices(&conflict).is_err());
            assert_eq!(history.point_count, 1);
            assert!(Arc::ptr_eq(&indices, &history.static_point_indices(&source).unwrap()));
        }
        let newer = AdmittedRegisteredScan::from_map_points_for_test(20, binding.clone(), &points);
        history.record(&newer, 21).unwrap();
        history.record(&source, 22).unwrap(); // archived exact duplicate remains idempotent
        assert!(history.static_point_indices(&source).is_err());
        assert_eq!(&*indices, &[0]); // already issued static selections remain immutable
        let stale = AdmittedRegisteredScan::from_map_points_for_test(9, binding, &points);
        assert!(history.record(&stale, 22).is_err());
    }

    #[test]
    fn exhausted_history_retires_sources_without_static_selection_pinning_payload() {
        let points = [Point3::new(1.0, 2.0, 3.0)];
        let source = AdmittedRegisteredScan::from_map_points_for_test(10, test_binding(), &points);
        let source_retained = source.retention_probe_for_test();
        let mut history = ScanHistory::default();
        history.record(&source, 11).unwrap();
        drop(source);
        history.point_count = MAX_ARCHIVED_POINTS;
        let next = AdmittedRegisteredScan::from_map_points_for_test(20, test_binding(), &points);
        let next_retained = next.retention_probe_for_test();
        history.record(&next, 21).unwrap();
        assert!(!source_retained());
        assert!(history.snapshot().is_err());
        history.record(&next, 22).unwrap();
        assert_eq!(&*history.static_point_indices(&next).unwrap(), &[0]);
        let indices = history.static_point_indices(&next).unwrap();
        drop(next);
        assert!(!next_retained());
        let expired = AdmittedRegisteredScan::from_map_points_for_test(20, test_binding(), &points);
        assert!(history.record(&expired, 22).is_err());
        assert!(history.static_point_indices(&expired).is_err());
        assert_eq!(&*indices, &[0]);
        let latest = AdmittedRegisteredScan::from_map_points_for_test(30, test_binding(), &points);
        let latest_retained = latest.retention_probe_for_test();
        history.record(&latest, 31).unwrap();
        assert!(!next_retained());
        drop(latest);
        assert!(!latest_retained());
        history.clear();
        assert!(!latest_retained());
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
                }), None)
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
                let source_timestamp_ns = 1_000_000_000 + i * 100_000_000;
                let mut binding = test_binding();
                binding.frozen_odom_from_body = pose;
                let source = AdmittedRegisteredScan::from_map_points_for_test(source_timestamp_ns, binding, &world);
                ArchivedScan {
                    source_timestamp_ns,
                    received_at_ns: 1 + i * 150_000_000,
                    odom_from_body: pose,
                    sensor_origin_body: Point3::origin(),
                    body_points: source.body_points().clone(),
                    source: ArchivedSource::Lidar(source),
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
    #[ignore = "same-fixture capture-only A/B diagnostic; original 10s budget, not a git baseline"]
    fn pgo_capture_cost_same_fixture_ab() {
        let (scans, mut trajectory) = scale_input(31);
        let cutoff = 12;
        trajectory[1].source_timestamp_ns = scans[cutoff].source_timestamp_ns as i64;
        trajectory[1].translation_m = (scans[cutoff].odom_from_body.translation.vector
            + nalgebra::Vector3::new(-0.04, 0.03, 0.0)).into();
        trajectory.push(OptimizedKeyframe {
            source_timestamp_ns: scans.last().unwrap().source_timestamp_ns as i64,
            translation_m: (scans.last().unwrap().odom_from_body.translation.vector
                + nalgebra::Vector3::new(-0.04, 0.03, 0.0)).into(),
            rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
        });
        let config = MappingBackendComputeConfig { resolution: 0.05, initial_safe_radius_meters: 0.0, window_capacity: 10_000 };
        // Opposite orders reduce warm-cache/order bias. All runs use fresh private
        // owners, the same input, profile and exact existing test budget.
        for order in [[false, true], [true, false]] {
            for spec in [MappingBackendSpec::Voxel, MappingBackendSpec::raytracing_defaults()] {
                for skip in order {
                    let mut builder = ReplayBuilder::new(&scans, trajectory.clone(),
                        Isometry3::translation(0.73, -1.15, 0.04), spec, config, "map".into(),
                        Duration::from_secs(10), &|| false).unwrap();
                    builder.skip_ground_capture_for_cost_comparison = skip;
                    let result = builder.append(&scans, &|| false).and_then(|()|builder.prepare(&|| false));
                    eprintln!("CAPTURE_AB backend={spec:?} capture={} frames={} points_per_frame={} processed={} kernel_ms={} cloud_ms={} wall_ms={} result={:?}",
                        !skip, scans.len(), scans[0].body_points.len(), builder.processed,
                        builder.kernel_time.as_millis(), builder.cloud_time.as_millis(), builder.started.elapsed().as_millis(), result.as_ref().err());
                    if let Some(cloud) = builder.cloud.as_mut() { cloud.cleanup_product_owned_file().unwrap(); }
                }
            }
        }
    }

    /// 本进程累计处理器时间（用户态加内核态，毫秒），用于和墙钟对照。
    fn process_cpu_ms() -> f64 {
        let mut usage: libc::rusage = unsafe { std::mem::zeroed() };
        assert_eq!(unsafe { libc::getrusage(libc::RUSAGE_SELF, &mut usage) }, 0);
        let ms = |t: libc::timeval| t.tv_sec as f64 * 1e3 + t.tv_usec as f64 / 1e3;
        ms(usage.ru_utime) + ms(usage.ru_stime)
    }

    /// 读取实录“原生配对扫描”：每帧68字节头（源时间、里程计位姿xyz+四元数xyzw、点数）后接机体系f32点。
    fn recorded_input(path: &str, frames: usize, origin_body: [f64; 3]) -> (Vec<ArchivedScan>, Vec<OptimizedKeyframe>) {
        use std::io::Read;
        let mut file = std::io::BufReader::new(std::fs::File::open(path).unwrap());
        let mut scans = Vec::new();
        while scans.len() < frames {
            let mut header = [0; 68];
            match file.read_exact(&mut header) {
                Ok(()) => {}
                Err(error) if error.kind() == std::io::ErrorKind::UnexpectedEof => break,
                Err(error) => panic!("读取实录扫描失败：{error}"),
            }
            let stamp = u64::from_le_bytes(header[..8].try_into().unwrap());
            let pose: [f64; 7] = std::array::from_fn(|i| f64::from_le_bytes(header[8 + i * 8..16 + i * 8].try_into().unwrap()));
            let size = u32::from_le_bytes(header[64..68].try_into().unwrap()) as usize;
            let mut bytes = vec![0; size * 12];
            file.read_exact(&mut bytes).unwrap();
            let odom_from_body = Isometry3::from_parts(Translation3::new(pose[0], pose[1], pose[2]),
                UnitQuaternion::new_normalize(nalgebra::Quaternion::new(pose[6], pose[3], pose[4], pose[5])));
            let map_points: Vec<_> = bytes.chunks_exact(12).map(|b| odom_from_body * Point3::new(
                f32::from_le_bytes(b[..4].try_into().unwrap()) as f64,
                f32::from_le_bytes(b[4..8].try_into().unwrap()) as f64,
                f32::from_le_bytes(b[8..12].try_into().unwrap()) as f64)).collect();
            let mut binding = test_binding();
            binding.cloud_source_sequence = scans.len() as i64 + 1;
            binding.frozen_odom_from_body = odom_from_body;
            binding.sensor_origin_map = Some((odom_from_body * Point3::from(origin_body)).coords.into());
            let source = AdmittedRegisteredScan::from_map_points_for_test(stamp, binding, &map_points);
            scans.push(ArchivedScan {
                source_timestamp_ns: stamp,
                received_at_ns: stamp,
                odom_from_body,
                sensor_origin_body: Point3::from(origin_body),
                body_points: source.body_points().clone(),
                source: ArchivedSource::Lidar(source),
            });
        }
        // 首节点固定；中段和尾部各加一个小的回环校正，覆盖校正插值与尾部复用。
        let keyframe = |scan: &ArchivedScan, offset: [f64; 3]| {
            let pose = scan.odom_from_body;
            let q = pose.rotation.coords;
            OptimizedKeyframe {
                source_timestamp_ns: scan.source_timestamp_ns as i64,
                translation_m: (pose.translation.vector + nalgebra::Vector3::from(offset)).into(),
                rotation_xyzw: [q.x, q.y, q.z, q.w],
            }
        };
        let trajectory = vec![
            keyframe(&scans[0], [0.0; 3]),
            keyframe(&scans[scans.len() / 2], [-0.04, 0.03, 0.0]),
            keyframe(scans.last().unwrap(), [-0.08, 0.05, 0.01]),
        ];
        (scans, trajectory)
    }

    /// 真实规模基准：NUWAX_PGO_RECORDED_SCANS 指向实录配对扫描，NUWAX_PGO_RECORDED_FRAMES 为帧数，
    /// NUWAX_PGO_SENSOR_ORIGIN_BODY 为雷达在机体系的原点（JSON数组）。分别测开启与关闭地面捕获。
    #[test]
    #[ignore = "需要显式提供实录配对扫描，release构建独立执行"]
    fn pgo_recorded_real_scale_rebuild() {
        let path = std::env::var("NUWAX_PGO_RECORDED_SCANS").expect("指定实录配对扫描文件");
        let frames = std::env::var("NUWAX_PGO_RECORDED_FRAMES").ok().and_then(|v| v.parse().ok()).unwrap_or(400);
        let origin: [f64; 3] = serde_json::from_str(&std::env::var("NUWAX_PGO_SENSOR_ORIGIN_BODY")
            .expect("指定雷达机体原点")).unwrap();
        let captures: Vec<bool> = std::env::var("NUWAX_PGO_CAPTURE_MODES").ok()
            .map(|v| v.split(',').map(|m| m == "on").collect()).unwrap_or_else(|| vec![true, false]);
        let (scans, trajectory) = recorded_input(&path, frames, origin);
        let points: usize = scans.iter().map(|scan| scan.body_points.len()).sum();
        let config = MappingBackendComputeConfig { resolution: 0.05, initial_safe_radius_meters: 0.0,
            window_capacity: nuwax_mapping::mapping_session::DEFAULT_BOUNDED_CAPACITY };
        // NUWAX_PGO_BACKENDS 为 voxel、raytracing 的逗号列表，缺省只测 voxel。
        let backends: Vec<MappingBackendSpec> = std::env::var("NUWAX_PGO_BACKENDS").unwrap_or_else(|_| "voxel".into())
            .split(',').map(|name| match name {
                "voxel" => MappingBackendSpec::Voxel,
                "raytracing" => MappingBackendSpec::raytracing_defaults(),
                other => panic!("未知后端 {other}"),
            }).collect();
        for spec in backends {
            let backend = if matches!(spec, MappingBackendSpec::Voxel) { "voxel" } else { "raytracing" };
            for &capture in &captures {
                let cpu = process_cpu_ms();
                let mut builder = ReplayBuilder::new(&scans, trajectory.clone(), Isometry3::identity(),
                    spec, config, "map".into(), Duration::MAX, &|| false).unwrap();
                builder.skip_ground_capture_for_cost_comparison = !capture;
                // 每 100 帧输出一段内核与点云的每帧平均，观察是否随地图增长。
                let mut result = Ok(());
                for chunk in scans.chunks(100) {
                    let (kernel, cloud) = (builder.kernel_time, builder.cloud_time);
                    result = builder.append(chunk, &|| false);
                    if result.is_err() { break; }
                    let count = chunk.len() as f64;
                    eprintln!("REAL_SEGMENT backend={backend} capture={capture} frames={} kernel_ms={:.2} cloud_ms={:.2}", builder.processed,
                        (builder.kernel_time - kernel).as_secs_f64() * 1e3 / count, (builder.cloud_time - cloud).as_secs_f64() * 1e3 / count);
                }
                let result = result.and_then(|()| builder.prepare(&|| false));
                eprintln!("REAL_SCALE backend={backend} capture={capture} frames={} points={points} processed={} kernel_ms={} cloud_ms={} wall_ms={} cpu_ms={:.0} cloud_points={:?} result={:?}",
                    scans.len(), builder.processed, builder.kernel_time.as_millis(), builder.cloud_time.as_millis(),
                    builder.started.elapsed().as_millis(), process_cpu_ms() - cpu, result.as_ref().ok().map(|p| p.cloud.point_count), result.as_ref().err());
                if let Some(cloud) = builder.cloud.as_mut() { cloud.cleanup_product_owned_file().unwrap(); }
            }
        }
    }

    /// 在线逐帧交接基准：实录按原顺序走在线建图点云入口（规范来源，含地面捕获），
    /// 每 100 帧输出平均与最大耗时，观察是否随累积地图增长。环境变量同实录重建基准，
    /// NUWAX_PGO_RECORDED_FRAMES 为帧数。
    #[test]
    #[ignore = "需要显式提供实录配对扫描，release构建独立执行"]
    fn online_recorded_frame_handoff_cost() {
        use crate::admitted_cloud::AcceptedMappingCloud;
        use crate::lio_ingress::AcceptedRegisteredCloud;
        let path = std::env::var("NUWAX_PGO_RECORDED_SCANS").expect("指定实录配对扫描文件");
        let frames = std::env::var("NUWAX_PGO_RECORDED_FRAMES").ok().and_then(|v| v.parse().ok()).unwrap_or(1500);
        let origin: [f64; 3] = serde_json::from_str(&std::env::var("NUWAX_PGO_SENSOR_ORIGIN_BODY")
            .expect("指定雷达机体原点")).unwrap();
        let captures: Vec<bool> = std::env::var("NUWAX_PGO_CAPTURE_MODES").ok()
            .map(|v| v.split(',').map(|m| m == "on").collect()).unwrap_or_else(|| vec![true]);
        let (scans, _) = recorded_input(&path, frames, origin);
        for capture in captures {
            let mut cloud = CloudMapShmWriter::new_private().unwrap();
            let (mut segment, mut worst, mut total) = (Duration::ZERO, Duration::ZERO, Duration::ZERO);
            let mut stages = [0.0; 3];
            let mut segment_cpu = process_cpu_ms();
            for (index, scan) in scans.iter().enumerate() {
                let map_points: Vec<_> = scan.body_points.iter().map(|p| scan.odom_from_body * p).collect();
                let sensor_origin = (scan.odom_from_body * scan.sensor_origin_body).coords.into();
                let mut binding = test_binding();
                binding.cloud_source_sequence = index as i64 + 1;
                binding.frozen_odom_from_body = scan.odom_from_body;
                binding.sensor_origin_map = Some(sensor_origin);
                let accepted = AcceptedMappingCloud::from(AcceptedRegisteredCloud::from_map_points_for_test(
                    map_points.clone(), scan.source_timestamp_ns, scan.source_timestamp_ns, "map", "body", binding));
                let started = Instant::now();
                assert!(cloud.accumulate_mapping_scan_with_source(1, &map_points, Some(sensor_origin),
                    scan.source_timestamp_ns, capture.then_some(&accepted)));
                let elapsed = started.elapsed();
                for (sum, stage) in stages.iter_mut().zip(cloud.last_stage_ms()) { *sum += stage; }
                segment += elapsed;
                total += elapsed;
                worst = worst.max(elapsed);
                if (index + 1) % 100 == 0 || index + 1 == scans.len() {
                    let count = (index % 100 + 1) as f64;
                    let cpu = process_cpu_ms();
                    eprintln!("ONLINE capture={capture} frames={} segment_mean_ms={:.2} segment_cpu_ms={:.2} segment_max_ms={:.2} total_mean_ms={:.2} fine/dense/voxel_ms={:.2}/{:.2}/{:.2}",
                        index + 1, segment.as_secs_f64() * 1e3 / count, (cpu - segment_cpu) / count, worst.as_secs_f64() * 1e3,
                        total.as_secs_f64() * 1e3 / (index + 1) as f64,
                        stages[0] / count, stages[1] / count, stages[2] / count);
                    segment_cpu = cpu;
                    stages = [0.0; 3];
                    segment = Duration::ZERO;
                    worst = Duration::ZERO;
                }
            }
            cloud.cleanup_product_owned_file().unwrap();
        }
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
            initial_safe_radius_meters: 0.0,
            window_capacity: 10_000,
        };
        let anchor = Isometry3::translation(0.73, -1.15, 0.04);
        // 本用例只证明分批追赶与一次重建结果相同，预算不是被测性质；预算超限与
        // 中途取消由 pgo_rebuild_distinguishes_budget_and_mid_replay_cancellation 覆盖。
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
                Duration::MAX,
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
                Duration::MAX,
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
            let a = &actual.persistent.grid;
            let b = &expected.persistent.grid;
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
            assert_eq!(actual.persistent.grid.data, expected.persistent.grid.data);
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
            scan.body_points = scan.body_points.edited_for_test(|points| {
                for (index, point) in points.iter_mut().enumerate() {
                    point.x += 0.08 * (frame as f64 * 0.19 + index as f64 * 0.03).sin();
                    point.z += 0.08 * (frame as f64 * 0.17 + index as f64 * 0.02).cos();
                }
            });
        }
        let scans: Arc<[ArchivedScan]> = scans.into();
        let config = MappingBackendComputeConfig {
            resolution: 0.05,
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
            eprintln!("固定扰动 parallel={parallel} elapsed_ms={:0.3} kernel_ms={kernel_ms:0.3} cloud_ms={cloud_ms:0.3} points={}",
                started.elapsed().as_secs_f64()*1000.0, output.cloud.point_count);
            if let Some(reference) = &reference {
                assert_eq!(output.persistent.grid.data, reference.persistent.grid.data);
                assert_eq!(output.persistent.grid.origin, reference.persistent.grid.origin);
                assert_eq!(
                    (output.persistent.grid.width, output.persistent.grid.height),
                    (reference.persistent.grid.width, reference.persistent.grid.height)
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
        eprintln!("后台阶段 kernel_ms={kernel_ms:0.3} cloud_ms={cloud_ms:0.3} materialize_ms={materialize_ms:0.3}");
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
    #[test]
    fn auxiliary_same_time_keeps_lidar_owner_and_its_distinct_source() {
        let (original,trajectory)=scale_input(2);
        let mut history=ScanHistory::default();
        let lidar=original[0].source.lidar().unwrap();
        history.record(lidar,original[0].received_at_ns).unwrap();
        let indices=history.static_point_indices(lidar).unwrap();
        let auxiliary=crate::lcm_runtime::mapping_observation::RegisteredObservation::for_test(
            original[0].source_timestamp_ns,"depth-front",vec![Point3::new(0.7,0.3,-0.8)],
            original[0].odom_from_body,[0.12,0.,0.3]);
        history.record_auxiliary(auxiliary.clone()).unwrap();
        history.record_auxiliary(auxiliary).unwrap();
        assert_eq!(history.static_point_indices(lidar).unwrap(),indices);
        assert_eq!(history.snapshot().unwrap().len(),2);
        assert!(history.snapshot_after(original[0].source_timestamp_ns).unwrap().is_empty());
        history.record(original[1].source.lidar().unwrap(),original[1].received_at_ns).unwrap();
        let scans=history.snapshot().unwrap();
        assert_eq!(scans.len(),3);
        assert!(scans[0].source.lidar().is_some());
        assert!(scans[1].source.lidar().is_none());
        let config=MappingBackendComputeConfig {resolution:0.1,initial_safe_radius_meters:0.,window_capacity:10_000};
        let mut builder=ReplayBuilder::new(&scans,trajectory,Isometry3::identity(),MappingBackendSpec::Voxel,
            config,"map".into(),Duration::from_secs(10),&||false).unwrap();
        builder.append(&scans,&||false).unwrap();
        assert_eq!(builder.processed,3);
        assert!(matches!(builder.append(&scans[2..],&||false),Err(RebuildError::Invalid(_))));
        builder.cloud.as_mut().unwrap().cleanup_product_owned_file().unwrap();
        let conflicting=crate::lcm_runtime::mapping_observation::RegisteredObservation::for_test(
            scans[0].source_timestamp_ns,"depth-front",vec![Point3::new(9.,9.,9.)],Isometry3::identity(),[0.12,0.,0.3]);
        assert!(history.record_auxiliary(conflicting).unwrap_err().contains("来源冲突"));
    }

    #[test]
    fn auxiliary_heightfield_replays_corrected_originals_in_serial_and_worker_lanes() {
        use crate::lcm_runtime::mapping_observation::RegisteredObservation;
        use nuwax_map_chunks::LayerSource;
        let wall: Vec<_> = (0..8).flat_map(|y| (0..8).map(move |z|
            Point3::new(0.28, 0.008 + y as f64 * 0.02, 0.21 + z as f64 * 0.02))).collect();
        let floor: Vec<_> = (0..8).flat_map(|x| (0..8).map(move |y|
            Point3::new(0.008 + x as f64 * 0.004, 0.008 + y as f64 * 0.004, 0.01))).collect();
        let mut history = ScanHistory::default();
        for (sequence, stamp) in [(1, 1_000_000_000), (2, 1_100_000_000)] {
            let mut binding = test_binding();
            binding.cloud_source_sequence = sequence;
            history.record(&AdmittedRegisteredScan::from_map_points_for_test(stamp, binding, &wall), stamp + 1).unwrap();
        }
        for sensor in ["depth-front", "depth-left"] {
            let mut points = floor.clone();
            points.extend_from_slice(&wall);
            let indices = (0..points.len()).map(|index| index as u32 * 2 + 3).collect();
            let source = RegisteredObservation::for_test_source(1_100_000_000, 12,
                LayerSource::Auxiliary { sensor_id: sensor.into(), clock_id: "clock".into(),
                    runtime_generation: 7, session_generation: 1 }, points, indices, Isometry3::identity());
            history.record_auxiliary(source.clone()).unwrap();
            history.record_auxiliary(source).unwrap();
        }
        let scans = history.snapshot().unwrap();
        assert_eq!(scans.len(), 4, "source replay is idempotent before geometry replay");
        let trajectory = vec![
            OptimizedKeyframe { source_timestamp_ns: 1_000_000_000,
                translation_m: [0.; 3], rotation_xyzw: [0., 0., 0., 1.] },
            OptimizedKeyframe { source_timestamp_ns: 1_100_000_000,
                translation_m: [0.1, 0., 0.2], rotation_xyzw: [0., 0., 0., 1.] },
        ];
        let anchor = Isometry3::translation(0.2, 0., 0.1);
        let expected_pose = anchor * Isometry3::translation(0.1, 0., 0.2);
        let config = MappingBackendComputeConfig { resolution: 0.05, initial_safe_radius_meters: 0., window_capacity: 1000 };
        let mut reference_layers = None;
        for parallel in [false, true] {
            let mut builder = ReplayBuilder::new(&scans, trajectory.clone(), anchor, MappingBackendSpec::Voxel,
                config, "map".into(), Duration::from_secs(10), &|| false).unwrap();
            if parallel {
                builder = builder.into_background();
                builder.force_parallel_lanes = true;
            }
            builder.append(&scans, &|| false).unwrap();
            builder.recover_cloud_worker().unwrap();
            let cloud = builder.cloud.as_mut().unwrap();
            cloud.bind_fine_map_owner(1, 1).unwrap();
            let receipt = crate::mapping_commit::OccupancyCommitReceipt {
                commit_id: crate::mapping_commit::MappingCommitId { boot_nonce: 1, sequence: 1 },
                session_id: "pgo-depth".into(), session_generation: 1, occupancy_revision: 1,
                source_timestamp_ns: 1_100_000_000, backend_kind: mapping_backend::MappingBackendKind::Voxel,
                backend_generation: 1,
            };
            cloud.commit_accumulated_snapshot(&receipt).unwrap().unwrap();
            let root = cloud.committed_fine_cloud().unwrap();
            let mut measured = BTreeMap::new();
            for block in &root.manifest().blocks {
                let bytes = root.repository.read_object(block.object, nuwax_map_chunks::MAX_BLOCK_BYTES).unwrap();
                let records = nuwax_map_chunks::decode_ground_records(&bytes).unwrap();
                for (key, record) in records.iter() {
                    let nuwax_map_chunks::GroundRecordRef::HeightfieldV1(field) = record else {
                        panic!("new real frames must use the heightfield representation");
                    };
                    measured.insert((block.key, key), field.clone());
                }
            }
            assert!(!measured.is_empty());
            let mut sensors = std::collections::BTreeSet::new();
            for sample in measured.values().flat_map(|layer| &layer.samples) {
                // The unchanged short vertical returns span 14 cm, below
                // the fixed >4q upright rule. Classification is not collision
                // permission; this control proves corrected source identity.
                match sample.source.as_ref() {
                    LayerSource::Auxiliary { sensor_id, clock_id, runtime_generation, session_generation } => {
                        sensors.insert(sensor_id.clone());
                        assert_eq!((clock_id.as_str(), *runtime_generation, *session_generation), ("clock", 7, 1));
                        assert_eq!((sample.source_sequence, sample.source_timestamp_ns), (12, 1_100_000_000));
                        assert!(sample.point_index >= 3 && (sample.point_index - 3) % 2 == 0);
                        let original = ((sample.point_index - 3) / 2) as usize;
                        let point = floor.iter().chain(&wall).nth(original).expect("original auxiliary index");
                        assert_eq!(sample.point, <[f64; 3]>::from((expected_pose * point).coords));
                    },
                    LayerSource::Lidar { .. } => {
                        let pose = match (sample.source_sequence, sample.source_timestamp_ns) {
                            (1, 1_000_000_000) => anchor,
                            (2, 1_100_000_000) => expected_pose,
                            other => panic!("unexpected LiDAR source {other:?}"),
                        };
                        let point = wall.get(sample.point_index as usize).expect("original LiDAR index");
                        assert_eq!(sample.point, <[f64; 3]>::from((pose * point).coords));
                    },
                    other => panic!("unexpected source {other:?}"),
                }
            }
            assert_eq!(sensors.len(), 2, "same-time independent sensors both keep evidence");
            if let Some(expected) = &reference_layers { assert_eq!(&measured, expected); }
            else { reference_layers = Some(measured); }
            let (file, _) = nuwax_map_chunks::archive_file(root.manifest(), &root.repository).unwrap();
            let prepared = crate::pointcloud::prepare_fine_cloud_archive(file, "pgo-depth", 1,
                root.manifest().revision, 1_100_000_000).unwrap();
            assert_eq!(prepared.voxel_size(), root.manifest().voxel_size_m);
            cloud.cleanup_product_owned_file().unwrap();
        }
    }

}

#[cfg(test)]
#[path = "pgo_ray_count_equivalence_tests.rs"]
mod ray_count_equivalence_tests;
