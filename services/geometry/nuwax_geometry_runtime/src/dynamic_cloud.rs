//! 用后续实测射线的空闲证据清除三维地图残影；不按时间淘汰静态结构。

use ahash::AHashSet as HashSet;

use nalgebra::Point3;

use super::voxel_index::VoxelIndex;
use super::{voxel_center_f64, voxel_key, VoxelKey};

#[derive(Default)]
pub(super) struct DynamicCloudFilter {
    scene: crate::dynamic_scene::DynamicScene,
    shared: Option<crate::dynamic_scene::MappedFrameDecision>,
    source_domain: Option<(String, u64)>,
    raw_log: Option<nuwax_cloud_cleanup::scan_log::ScanLog>,
    raw_log_failed: bool,
    removed_keys: Vec<VoxelKey>,
}

impl DynamicCloudFilter {
    pub(crate) fn has_source_domain(&self) -> bool {
        self.source_domain.is_some()
    }
    pub(super) fn clear_frame(&mut self) {
        self.removed_keys.clear();
        self.shared = None;
        self.scene.clear_frame();
    }

    pub(super) fn is_dynamic(&self, point: [f64; 3]) -> bool {
        self.shared
            .as_ref()
            .map_or_else(|| self.scene.is_dynamic(point), |f| f.is_dynamic(point))
    }
    /// 当前扫描实际移除的体素，供确定性快照同步撤销；下一次扫描清空。
    pub(super) fn removed_keys(&self) -> &[VoxelKey] {
        &self.removed_keys
    }

    /// 只在原会话接纳成功后调用；同一帧的多束射线只能形成一次反证。
    #[cfg(test)]
    pub(super) fn clear_observed_free_space(
        &mut self,
        voxels: &mut VoxelIndex,
        voxel_size: f32,
        origin: [f64; 3],
        timestamp_ns: u64,
        points: &[Point3<f64>],
    ) -> usize {
        let hits = points
            .iter()
            .filter_map(|p| voxel_key([p.x, p.y, p.z], voxel_size))
            .collect();
        self.clear_observed_free_space_with_hits(
            voxels,
            voxel_size,
            origin,
            timestamp_ns,
            points,
            &hits,
        )
    }

    pub(super) fn observe_with_ground(
        &mut self,
        voxels: &mut VoxelIndex,
        size: f32,
        origin: [f64; 3],
        stamp: u64,
        points: &[Point3<f64>],
        ground: &[u32],
    ) -> usize {
        self.removed_keys.clear();
        self.scene
            .observe_with_ground(points, origin, stamp, ground);
        if self.scene.has_clears() {
            self.removed_keys = voxels
                .iter()
                .copied()
                .filter(|key| self.clears(voxel_center_f64(*key, size)))
                .collect();
            for key in &self.removed_keys {
                voxels.remove(key);
            }
        }
        self.removed_keys.len()
    }
    pub(super) fn clear_observed_free_space_with_hits(
        &mut self,
        voxels: &mut VoxelIndex,
        size: f32,
        origin: [f64; 3],
        stamp: u64,
        points: &[Point3<f64>],
        _hits: &HashSet<VoxelKey>,
    ) -> usize {
        self.observe_with_ground(voxels, size, origin, stamp, points, &[])
    }
    pub(super) fn clears(&self, p: [f64; 3]) -> bool {
        self.shared
            .as_ref()
            .map_or_else(|| self.scene.clears(p), |f| f.clears(p))
    }
    pub(super) fn adopt(
        &mut self,
        frame: crate::dynamic_scene::MappedFrameDecision,
        voxels: &mut VoxelIndex,
        size: f32,
    ) {
        self.removed_keys.clear();
        self.shared = Some(frame);
        self.removed_keys = voxels
            .iter()
            .copied()
            .filter(|key| self.clears(voxel_center_f64(*key, size)))
            .collect();
        for key in &self.removed_keys {
            voxels.remove(key);
        }
    }
    pub(super) fn classify(
        &mut self,
        source: &crate::lio_ingress::AdmittedRegisteredScan,
        ground: &[u32],
        known: &[u32],
    ) -> Result<std::sync::Arc<crate::dynamic_scene::FrameDecision>, String> {
        if let Some(frame) = source.dynamic_frame() {
            return Ok(frame);
        }
        #[cfg(test)]
        let diagnostic = std::env::var_os("NUWAX_DYNAMIC_PHASE_DIAGNOSTIC").is_some();
        #[cfg(test)]
        let phase_started = std::time::Instant::now();
        let binding = source.binding();
        let domain = (
            binding.mapping_session_id.clone(),
            binding.mapping_session_generation,
        );
        if self.source_domain.as_ref() != Some(&domain) {
            self.scene = Default::default();
            self.source_domain = Some(domain);
            self.raw_log = None;
            self.raw_log_failed = false;
        }
        let points = source
            .body_points()
            .iter()
            .map(|p| binding.frozen_odom_from_body * p)
            .collect::<Vec<_>>();
        let odom_from_map = binding.registration_map_from_odom.inverse();
        let origin = binding
            .sensor_origin_map
            .map(|p| odom_from_map * Point3::from(p));
        #[cfg(test)]
        let prepare_ms = phase_started.elapsed().as_secs_f64() * 1e3;
        if let Some(origin) = origin {
            self.scene.observe_with_known_ground(
                &points,
                origin.coords.into(),
                source.source_timestamp_ns(),
                ground,
                known,
            );
        } else {
            self.scene.clear_frame();
        }
        #[cfg(test)]
        let observed_ms = phase_started.elapsed().as_secs_f64() * 1e3;
        let mut frame = self.scene.decision();
        if frame.labels.labels.is_empty() {
            std::sync::Arc::make_mut(&mut frame).labels = crate::human_point_labels::FrameLabels {
                source_timestamp_ns: source.source_timestamp_ns(),
                labels: vec![crate::human_point_labels::PointLabel::PendingNew; points.len()],
            };
        }
        #[cfg(test)]
        let decision_ms = phase_started.elapsed().as_secs_f64() * 1e3;
        use nuwax_cloud_cleanup::scan_log::{ScanLog, ScanMetadata, Session};
        if self.raw_log.is_none() && !self.raw_log_failed {
            let root = nuwax_cloud_cleanup::scan_log::scan_directory();
            match ScanLog::open(&root, Session {
                session_id: binding.mapping_session_id.clone(),
                generation: binding.mapping_session_generation,
            }) {
                Ok(log) => self.raw_log = Some(log),
                Err(error) => {
                    self.raw_log_failed = true;
                    tracing::warn!(event = "mapping.raw_scan_incomplete", %error,
                        "原始扫描记录不可用，继续建图；保存时将明确标记离线重判输入不完整");
                }
            }
        }
        let body_from_map =
            (binding.registration_map_from_odom * binding.frozen_odom_from_body).inverse();
        let origin_body = binding
            .sensor_origin_map
            .map(|p| body_from_map * Point3::from(p));
        let metadata = ScanMetadata {
            source_timestamp_ns: source.source_timestamp_ns(),
            source_sequence: source.source_identity().source_sequence as u64,
            sensor: source.sensor_frame_id().into(),
            source_generation: binding.runtime_generation as u64,
            odom_from_body: binding
                .frozen_odom_from_body
                .to_homogeneous()
                .as_slice()
                .try_into()
                .unwrap(),
            map_from_odom: binding
                .registration_map_from_odom
                .to_homogeneous()
                .as_slice()
                .try_into()
                .unwrap(),
            observation_minimum_range_m: Some(self.scene.minimum_range_m()),
            sensor_origin_body: origin_body.map(|p| p.coords.into()),
            point_count: source.body_points().len(),
        };
        if let Some(log) = self.raw_log.as_mut() {
            if let Err(error) = log.append(&metadata,
                source.body_points().iter().map(|p| [p.x, p.y, p.z]), &frame.labels.labels) {
                let _ = std::fs::File::create(log.path.with_extension("incomplete"));
                self.raw_log = None;
                self.raw_log_failed = true;
                tracing::warn!(event = "mapping.raw_scan_incomplete", %error,
                    "原始扫描记录已中断，继续建图；保存时将明确标记离线重判输入不完整");
            }
        }
        #[cfg(test)]
        let scanlog_ms = phase_started.elapsed().as_secs_f64() * 1e3;
        source.record_decision(frame.clone())?;
        #[cfg(test)]
        if diagnostic {
            let elapsed = phase_started.elapsed().as_secs_f64() * 1e3;
            eprintln!("DYNAMIC_CLASSIFY timestamp={} points={} prepare_ms={:.3} observe_ms={:.3} decision_ms={:.3} scanlog_ms={:.3} record_ms={:.3} total_ms={:.3}",
                source.source_timestamp_ns(), source.body_points().len(), prepare_ms,
                observed_ms - prepare_ms, decision_ms - observed_ms,
                scanlog_ms - decision_ms, elapsed - scanlog_ms, elapsed);
        }
        Ok(frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn an_isolated_ray_does_not_invent_observed_neighbours() {
        let mut filter = DynamicCloudFilter::default();
        let mut map = VoxelIndex::from([(40, 0, 0), (120, 0, 0), (40, 40, 0)]);
        let original = map.clone();
        let points = vec![Point3::new(4.025, 0.025, 0.025); 20];
        for stamp in [1, 1, 0, 2, 3, 4, 5] {
            filter.clear_observed_free_space(&mut map, 0.05, [0.025; 3], stamp, &points);
        }
        assert_eq!(
            map, original,
            "no 27-cell observed neighbourhood, no invented void"
        );
    }
    #[test]
    fn accumulated_static_room_keeps_actual_observed_geometry() {
        use crate::dynamic_scene::tests::{corridor, ORIGIN};
        let mut filter = DynamicCloudFilter::default();
        let mut map = VoxelIndex::new();
        let mut truth = HashSet::new();
        for frame in 1..=30 {
            let (points, _) = corridor(None, (frame % 3) as f64);
            filter.clear_observed_free_space(&mut map, 0.05, ORIGIN, frame * 100_000_000, &points);
            let keys = points
                .iter()
                .map(|p| voxel_key([p.x, p.y, p.z], 0.05).unwrap())
                .collect::<HashSet<_>>();
            truth.extend(keys.iter().copied());
            map.extend(keys);
        }
        assert!(truth.iter().all(|k| map.contains(k)));
    }
}
