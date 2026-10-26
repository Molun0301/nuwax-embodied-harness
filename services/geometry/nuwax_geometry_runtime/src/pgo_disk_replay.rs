//! 完整日志前缀逐记录读取；私有重建只持当前帧、末帧和既有地图状态。
use super::*;
use nuwax_cloud_cleanup::scan_log::{LoggedScan, ReplaySource, ScanReader};

fn log_error(error: String) -> RebuildError {
    tracing::warn!(event = "pgo.log.incomplete", %error, "扫描日志不可用于完整重放，保留当前地图");
    RebuildError::HistoryUnavailable
}

#[derive(Clone)]
pub(crate) struct DiskReplayInput {
    pub source: ReplaySource,
    pub latest: ArchivedScan,
    pub locals: Vec<Isometry3<f64>>,
    pub first_receipt: (u64, u64),
}

pub(crate) struct DiskReplayCursor {
    source: ReplaySource,
    reader: Option<ScanReader>,
    pending: Option<LoggedScan>,
    layer_source: Arc<nuwax_map_chunks::LayerSource>,
    last_logged_ns: u64,
    first_receipt: (u64, u64),
}

impl DiskReplayInput {
    pub(crate) fn start(
        self,
        trajectory: Vec<OptimizedKeyframe>,
        anchor: Isometry3<f64>,
        spec: MappingBackendSpec,
        config: MappingBackendComputeConfig,
        frame_id: String,
        budget: Duration,
        terminal: crate::dynamic_backend::OnlineDufo,
        cancelled: &impl Fn() -> bool,
    ) -> Result<(ReplayBuilder, DiskReplayCursor), RebuildError> {
        let started = Instant::now();
        check_budget(started, budget, cancelled)?;
        validate_optimized_keyframes(&trajectory)?;
        if trajectory.is_empty() || self.locals.len() != trajectory.len() {
            return Err("PGO缺少精确原始关键帧位姿".into());
        }
        let first = trajectory[0].source_timestamp_ns;
        let keyframes: Vec<_> = trajectory
            .iter()
            .zip(self.locals)
            .map(|(node, local_pose)| CorrectionKeyframe {
                timestamp: (node.source_timestamp_ns - first) as f64 / 1e9,
                local_pose,
                optimized_pose: optimized_pose(node),
            })
            .collect();
        let first_delta = keyframes[0].local_pose.inverse() * keyframes[0].optimized_pose;
        if first_delta.translation.vector.norm() > 1e-6 || first_delta.rotation.angle() > 1e-6 {
            return Err("PGO优化改变了固定首节点".into());
        }
        let timeline = CorrectionTimeline::new(keyframes).map_err(|e| e.to_string())?;
        let mut builder = ReplayBuilder::with_timeline(
            timeline, first, anchor, spec, config, frame_id, budget, started,
        )?;
        builder.terminal_dufo = Some(terminal);
        builder.processed_evidence =
            Some(crate::dynamic_backend::free_evidence::ProcessedEvidence::default());
        let root = self
            .source
            .path()
            .parent()
            .ok_or("PGO日志目录缺失")?
            .to_path_buf();
        builder.pose_spool =
            Some(nuwax_cloud_cleanup::scan_log::PoseSpool::new(root).map_err(log_error)?);
        let cursor = DiskReplayCursor {
            source: self.source,
            reader: None,
            pending: None,
            layer_source: self.latest.source.layer_source(),
            last_logged_ns: 0,
            first_receipt: self.first_receipt,
        };
        Ok((builder, cursor))
    }
}

impl DiskReplayCursor {
    /// 每次只解码一条记录；长静止段的进度记录同样受取消与时间片约束。
    pub(crate) fn append_slice(
        mut self,
        mut builder: ReplayBuilder,
        latest: &ArchivedScan,
        cancelled: &impl Fn() -> bool,
    ) -> Result<(Self, ReplayBuilder, bool), RebuildError> {
        builder.check(cancelled)?;
        let prefix = self.source.snapshot().map_err(log_error)?;
        let Some(prefix) = prefix else {
            return Ok((self, builder, false));
        };
        match &mut self.reader {
            Some(reader) => reader.extend_prefix(&prefix).map_err(log_error)?,
            slot @ None => *slot = Some(ScanReader::open_prefix(&prefix).map_err(log_error)?),
        }
        let started = Instant::now();
        let mut records = 0;
        let mut complete = false;
        loop {
            builder.check(cancelled)?;
            let next = if let Some(scan) = self.pending.take() {
                Some(scan)
            } else {
                use nuwax_cloud_cleanup::scan_log::ReplayRecord;
                match self
                    .reader
                    .as_mut()
                    .unwrap()
                    .next_record()
                    .map_err(log_error)?
                {
                    Some(ReplayRecord::Scan(scan)) => Some(scan),
                    Some(ReplayRecord::Progress) => None,
                    None => {
                        complete = true;
                        break;
                    }
                }
            };
            if let Some(scan) = next {
                if scan.metadata.source_timestamp_ns > latest.source_timestamp_ns {
                    self.pending = Some(scan);
                    complete = true;
                    break;
                }
                if scan.metadata.source_timestamp_ns <= self.last_logged_ns {
                    return Err(log_error("PGO日志扫描源时间未前进".into()));
                }
                self.last_logged_ns = scan.metadata.source_timestamp_ns;
                if scan.metadata.source_timestamp_ns > builder.latest_timestamp_ns() {
                    let archived = self
                        .decode_scan(scan, latest)
                        .map_err(|error| log_error(error.to_string()))?;
                    builder.append(std::slice::from_ref(&archived), cancelled)?;
                }
            }
            records += 1;
            if records >= REPLAY_SLICE_MAX_FRAMES || started.elapsed() >= REPLAY_SLICE {
                break;
            }
        }
        let ready = complete && prefix.observed_until_ns() >= latest.source_timestamp_ns;
        if ready && latest.source_timestamp_ns > builder.latest_timestamp_ns() {
            // 默认抽帧允许末帧未入日志；仅在日志进度已覆盖它时衔接这一真实末帧。
            builder.append(std::slice::from_ref(latest), cancelled)?;
        }
        builder.recover_cloud_worker()?;
        Ok((self, builder, ready))
    }

    fn decode_scan(
        &self,
        scan: LoggedScan,
        latest: &ArchivedScan,
    ) -> Result<ArchivedScan, RebuildError> {
        let metadata = scan.metadata;
        if metadata.source_timestamp_ns == 0
            || metadata.source_sequence == 0
            || metadata.odom_from_body.iter().any(|v| !v.is_finite())
        {
            return Err("PGO日志扫描身份或位姿无效".into());
        }
        let matrix = nalgebra::Matrix4::from_column_slice(&metadata.odom_from_body);
        let rotation = matrix.fixed_view::<3, 3>(0, 0).into_owned();
        if (rotation.transpose() * rotation - nalgebra::Matrix3::identity()).norm() > 1e-6
            || (rotation.determinant() - 1.).abs() > 1e-6
            || (0..4).any(|i| (matrix[(3, i)] - if i == 3 { 1. } else { 0. }).abs() > 1e-9)
        {
            return Err("PGO日志扫描位姿不是刚体变换".into());
        }
        let odom_from_body = Isometry3::from_parts(
            Translation3::new(matrix[(0, 3)], matrix[(1, 3)], matrix[(2, 3)]),
            UnitQuaternion::from_matrix(&rotation),
        );
        let body_points: crate::correction_points::CorrectionPoints = scan
            .points
            .into_iter()
            .map(|p| Point3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2])))
            .collect();
        let nuwax_map_chunks::LayerSource::Lidar { backend, .. } = self.layer_source.as_ref()
        else {
            return Err("PGO日志扫描不是LiDAR来源".into());
        };
        let source = ArchivedSource::Logged(Arc::new(crate::admitted_cloud::LoggedCloudSource {
            timestamp_ns: metadata.source_timestamp_ns,
            sequence: metadata.source_sequence,
            layer_source: Arc::new(nuwax_map_chunks::LayerSource::Lidar {
                backend: *backend,
                runtime_generation: i64::try_from(metadata.source_generation)
                    .map_err(|_| "PGO日志来源代次无效")?,
                sensor_frame_id: metadata.sensor,
            }),
            body_points: body_points.clone(),
        }));
        // v2只保存源时钟。历史构图强制逐帧推进，接收时钟以真实首/末收据重标定；
        // 末帧回到在线接收时钟，避免恢复内核令后续真实输入长期等不到构图间隔。
        let (first_source, first_received) = self.first_receipt;
        let first_received = first_received.min(latest.received_at_ns);
        let source_span = latest.source_timestamp_ns.saturating_sub(first_source);
        let received_at_ns = if source_span == 0 {
            latest.received_at_ns
        } else {
            let elapsed = metadata
                .source_timestamp_ns
                .saturating_sub(first_source)
                .min(source_span);
            first_received
                + ((u128::from(latest.received_at_ns - first_received) * u128::from(elapsed))
                    / u128::from(source_span)) as u64
        };
        Ok(ArchivedScan {
            source_timestamp_ns: metadata.source_timestamp_ns,
            received_at_ns,
            odom_from_body,
            sensor_origin_body: metadata
                .sensor_origin_body
                .map(|p| Point3::from(p.map(f64::from))),
            body_points,
            source,
        })
    }
}

pub(crate) fn rebuild_disk(
    input: DiskReplayInput,
    trajectory: Vec<OptimizedKeyframe>,
    anchor: Isometry3<f64>,
    spec: MappingBackendSpec,
    config: MappingBackendComputeConfig,
    frame_id: String,
    budget: Duration,
    terminal: crate::dynamic_backend::OnlineDufo,
    cancelled: impl Fn() -> bool,
) -> Result<RebuiltMap, RebuildError> {
    let latest = input.latest.clone();
    if trajectory
        .last()
        .map(|node| node.source_timestamp_ns as u64)
        != Some(latest.source_timestamp_ns)
    {
        return Err("PGO轨迹与已接纳扫描尾部不一致".into());
    }
    let (mut builder, mut cursor) = input.start(
        trajectory, anchor, spec, config, frame_id, budget, terminal, &cancelled,
    )?;
    loop {
        let (advanced, next_builder, ready) = cursor.append_slice(builder, &latest, &cancelled)?;
        cursor = advanced;
        builder = next_builder;
        if ready {
            break;
        }
        std::thread::sleep(REPLAY_YIELD);
    }
    let prepared = builder.prepare(&cancelled)?;
    builder.finish(prepared, &cancelled)
}
