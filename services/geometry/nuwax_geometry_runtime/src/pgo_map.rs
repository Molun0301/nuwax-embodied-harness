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
const REBUILD_BUDGET: Duration = Duration::from_secs(3);

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
}

impl ScanHistory {
    pub fn clear(&mut self) {
        self.scans.clear();
        self.point_count = 0;
    }

    pub fn record(
        &mut self,
        source_timestamp_ns: u64,
        received_at_ns: u64,
        binding: &RegistrationBinding,
        points: &[Point3<f64>],
    ) -> Result<(), String> {
        if let Some(existing) = self.scans.get(&source_timestamp_ns) {
            return if existing.envelope_sha256 == binding.envelope_sha256 {
                Ok(())
            } else {
                Err("PGO历史扫描来源冲突".into())
            };
        }
        if self.point_count.saturating_add(points.len()) > MAX_ARCHIVED_POINTS {
            return Err("PGO历史扫描超过800万点上限，不能以不完整历史发布地图校正".into());
        }
        let body_from_map =
            (binding.registration_map_from_odom * binding.frozen_odom_from_body).inverse();
        let body_points: Vec<_> = points.iter().map(|point| body_from_map * point).collect();
        let sensor_origin_body = binding
            .sensor_origin_map
            .map(|p| body_from_map * Point3::from(p))
            .unwrap_or_else(Point3::origin);
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

    pub fn snapshot(&self) -> Vec<ArchivedScan> {
        self.scans.values().cloned().collect()
    }
}

pub(crate) struct RebuiltMap {
    pub occupancy: OccupancyGrid,
    pub cloud: CommittedCloudMapSnapshot,
    pub backend_checkpoint: Vec<u8>,
    pub accumulator: crate::pointcloud::PreparedCloudAccumulator,
}

pub(crate) fn optimized_pose(frame: &OptimizedKeyframe) -> Isometry3<f64> {
    let q = frame.rotation_xyzw;
    Isometry3::from_parts(
        Translation3::from(nalgebra::Vector3::from(frame.translation_m)),
        UnitQuaternion::new_normalize(nalgebra::Quaternion::new(q[3], q[0], q[1], q[2])),
    )
}

// 即使后台重建发生异常展开，也必须释放本次独占的临时共享内存文件。
struct TemporaryCloud(CloudMapShmWriter);
impl std::ops::Deref for TemporaryCloud {
    type Target = CloudMapShmWriter;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl std::ops::DerefMut for TemporaryCloud {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
impl Drop for TemporaryCloud {
    fn drop(&mut self) {
        let _ = self.0.cleanup_product_owned_file();
    }
}

/// 只使用已接纳扫描和原建图内核，不构造新的传感器观测或并行公共地图写入端。
pub(crate) fn rebuild(
    scans: Vec<ArchivedScan>,
    trajectory: Vec<OptimizedKeyframe>,
    anchor: Isometry3<f64>,
    spec: MappingBackendSpec,
    config: MappingBackendComputeConfig,
    frame_id: String,
    cancelled: impl Fn() -> bool,
) -> Result<RebuiltMap, String> {
    validate_optimized_keyframes(&trajectory)?;
    let first = trajectory
        .first()
        .ok_or("PGO结果缺少完整优化轨迹")?
        .source_timestamp_ns;
    let last = trajectory.last().unwrap().source_timestamp_ns as u64;
    if scans.last().map(|s| s.source_timestamp_ns) != Some(last) {
        return Err("PGO轨迹与已接纳扫描尾部不一致".into());
    }
    let mut keyframes = Vec::with_capacity(trajectory.len());
    for node in &trajectory {
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
    // 图的第一个节点必须保持原锚定；不能把全图平移伪装成回环优化。
    let first_delta = keyframes[0].local_pose.inverse() * keyframes[0].optimized_pose;
    if first_delta.translation.vector.norm() > 1e-6 || first_delta.rotation.angle() > 1e-6 {
        return Err("PGO优化改变了固定首节点".into());
    }
    let timeline = CorrectionTimeline::new(keyframes).map_err(|e| e.to_string())?;
    let path =
        std::env::temp_dir().join(format!("nuwax-pgo-rebuild-{}.mmap", uuid::Uuid::new_v4()));
    let mut cloud = TemporaryCloud(CloudMapShmWriter::new(&path).map_err(|e| e.to_string())?);
    let start = Instant::now();
    let mut kernel_time = Duration::ZERO;
    let mut cloud_time = Duration::ZERO;
    let mut processed = 0;
    let result = (|| {
        let mut kernel = spec.into_kernel(config.window_capacity);
        let mut persistent: Option<PersistentOccupancyGrid> = None;
        let mut last_input = None;
        for (index, scan) in scans.iter().enumerate() {
            if cancelled() || start.elapsed() > REBUILD_BUDGET {
                return Err("PGO地图重建已取消或超过计算预算".into());
            }
            let time = (scan.source_timestamp_ns as i128 - first as i128) as f64 / 1e9;
            let map_from_body = anchor
                * timeline
                    .correct_pose(time, &scan.odom_from_body)
                    .map_err(|e| e.to_string())?;
            let points: Vec<_> = scan
                .body_points
                .iter()
                .map(|point| map_from_body * point)
                .collect();
            let origin = map_from_body * scan.sensor_origin_body;
            let input = MappingBackendInput {
                session_id: "pgo-private-rebuild".into(),
                session_generation: 1,
                backend_generation: 1,
                frame_sequence: index as u64 + 1,
                source_timestamp_ns: scan.source_timestamp_ns,
                received_at_ns: scan.received_at_ns,
                registered_frame_id: frame_id.clone(),
                sensor_origin_map: origin.coords.into(),
                registered_points_map: points,
            };
            let phase = Instant::now();
            mapping_backend::validate_backend_input(&kernel, &input)?;
            if let Some(candidate) =
                mapping_backend::compute_candidate_in_place(&mut kernel, &input, config)?
            {
                persistent = Some(match candidate.backend_kind {
                    MappingBackendKind::Voxel => PersistentOccupancyGrid::stage_voxel(
                        persistent.as_ref(),
                        &candidate.live_window_occupancy,
                        &candidate.latest_frame_occupancy,
                    )?,
                    MappingBackendKind::Raytracing => PersistentOccupancyGrid::stage_authoritative(
                        &candidate.live_window_occupancy,
                    )?,
                });
            }
            kernel_time += phase.elapsed();
            let phase = Instant::now();
            if !cloud.accumulate_mapping_scan(
                1,
                &input.registered_points_map,
                Some(input.sensor_origin_map),
                scan.source_timestamp_ns,
            ) {
                return Err("PGO重建点云无法接纳原始扫描".into());
            }
            cloud_time += phase.elapsed();
            processed += 1;
            last_input = Some(input);
        }
        let input = last_input.ok_or("PGO重建没有扫描")?;
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
        if let Some(candidate) =
            mapping_backend::flush_candidate(&mut kernel, &input, &token, config)?
        {
            persistent = Some(match candidate.backend_kind {
                MappingBackendKind::Voxel => PersistentOccupancyGrid::stage_voxel(
                    persistent.as_ref(),
                    &candidate.live_window_occupancy,
                    &candidate.latest_frame_occupancy,
                )?,
                MappingBackendKind::Raytracing => {
                    PersistentOccupancyGrid::stage_authoritative(&candidate.live_window_occupancy)?
                }
            });
        }
        if cancelled() || start.elapsed() > REBUILD_BUDGET {
            return Err("PGO地图重建已取消或超过计算预算".into());
        }
        cloud
            .commit_accumulated()
            .ok_or("PGO重建点云无法完成快照")?;
        Ok(RebuiltMap {
            occupancy: persistent.ok_or("PGO重建没有占据地图")?.grid,
            cloud: cloud.committed_snapshot().ok_or("PGO重建点云快照缺失")?,
            backend_checkpoint: mapping_backend::export_mapping_kernel_checkpoint(&kernel)?,
            accumulator: cloud.take_prepared_accumulator(),
        })
    })();
    let cleanup = cloud
        .cleanup_product_owned_file()
        .map_err(|e| format!("PGO临时点云清理失败：{e}"));
    let result = result.and_then(|value| cleanup.map(|()| value));
    tracing::info!(
        event = "pgo.map.rebuilt",
        frames = scans.len(),
        processed,
        kernel_ms = kernel_time.as_millis() as u64,
        cloud_ms = cloud_time.as_millis() as u64,
        elapsed_ms = start.elapsed().as_millis() as u64,
        success = result.is_ok(),
        "PGO私有地图重建结束"
    );
    #[cfg(test)]
    eprintln!(
        "重建计时 frames={processed} kernel={kernel_time:?} cloud={cloud_time:?} total={:?}",
        start.elapsed()
    );
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    #[ignore = "三百八十五帧规模基准，使用release构建独立执行"]
    fn pgo_rebuild_recorded_scale_budget() {
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
        let scans = (0..385)
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
        let trajectory = [0, 384]
            .map(|i| OptimizedKeyframe {
                source_timestamp_ns: scans[i].source_timestamp_ns as i64,
                translation_m: scans[i].odom_from_body.translation.vector.into(),
                rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
            })
            .to_vec();
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
            || false,
        )
        .unwrap();
    }
}
