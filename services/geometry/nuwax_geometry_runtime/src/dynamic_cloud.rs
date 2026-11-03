//! 用后续实测射线的空闲证据清除三维地图残影；不按时间淘汰静态结构。

use ahash::AHashSet as HashSet;

use nalgebra::Point3;

use super::voxel_index::VoxelIndex;
use super::{voxel_center_f64, VoxelKey};
#[cfg(test)]
use super::voxel_key;

#[derive(Default)]
pub(super) struct DynamicCloudFilter {
    dufo: crate::dynamic_backend::OnlineDufo,
    current: Option<std::sync::Arc<crate::dynamic_backend::MappedFrame>>,
    shared: Option<crate::dynamic_backend::MappedFrame>,
    source_domain: Option<(String, u64)>,
    raw_log: Option<nuwax_cloud_cleanup::scan_log::Recorder>,
    raw_log_failed: bool,
    removed_keys: Vec<VoxelKey>,
}

impl DynamicCloudFilter {
    pub(crate) fn replay_source(&self) -> Option<nuwax_cloud_cleanup::scan_log::ReplaySource> {
        self.raw_log.as_ref().map(|recorder| recorder.replay_source())
    }
    pub(crate) fn terminal_dufo(&self) -> crate::dynamic_backend::OnlineDufo { self.dufo.clone() }
    pub(crate) fn terminal_dufo_since(&self, revision: u64) -> (crate::dynamic_backend::OnlineDufo, Option<Vec<nuwax_dufomap_native::FreeCell>>) {
        self.dufo.snapshot_since(revision)
    }
    pub(crate) fn free_revision(&self) -> u64 { self.dufo.free_revision() }
    pub(crate) fn has_source_domain(&self) -> bool {
        self.source_domain.is_some()
    }
    pub(super) fn clear_frame(&mut self) {
        self.removed_keys.clear();
        self.shared = None;
    }

    pub(super) fn is_dynamic(&self, point: [f64; 3]) -> bool {
        self.shared
            .as_ref()
            .is_some_and(|f| f.is_dynamic(point))
    }
    /// 当前扫描实际移除的体素，供确定性快照同步撤销；下一次扫描清空。
    pub(super) fn observe_dufo(&mut self, points: &[Point3<f64>], origin: [f64; 3], stamp: u64)
        -> Result<crate::dynamic_backend::MappedFrame, String> {
        self.dufo.observe(points, origin, stamp, nalgebra::Isometry3::identity()).map(|v| v.0)
    }
    pub(super) fn shared_frame(&self) -> Option<&crate::dynamic_backend::MappedFrame> { self.shared.as_ref() }
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
            .filter_map(|p| super::voxel_key([p.x, p.y, p.z], voxel_size))
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
    ) -> usize {
        self.removed_keys.clear();
        self.shared = Some(self.dufo.observe(points, origin, stamp, nalgebra::Isometry3::identity())
            .expect("已校验的动态点云积分失败").0);
        if self.shared.as_ref().is_some_and(|f| f.has_clears()) {
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
        self.observe_with_ground(voxels, size, origin, stamp, points)
    }
    pub(super) fn clears(&self, p: [f64; 3]) -> bool {
        self.shared
            .as_ref()
            .is_some_and(|f| f.clears(p))
    }
    pub(super) fn adopt(
        &mut self,
        frame: crate::dynamic_backend::MappedFrame,
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
    fn record_raw(&mut self, source: &crate::lio_ingress::AdmittedRegisteredScan) {
        let binding = source.binding();
        use nuwax_cloud_cleanup::scan_log::{Recorder, ScanMetadata, Session};
        if self.raw_log.is_none() && !self.raw_log_failed {
            let root = nuwax_cloud_cleanup::scan_log::scan_directory();
            match Recorder::new(root, Session {
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
            observation_minimum_range_m: Some(crate::dynamic_backend::online_params().min_range as f64),
            sensor_origin_body: origin_body.map(|p| [p.x as f32, p.y as f32, p.z as f32]),
            point_count: source.body_points().len(),
        };
        if let Some(log) = self.raw_log.as_mut() {
            if let Err(error) = log.observe(metadata,
                source.body_points().iter().map(|p| [p.x as f32, p.y as f32, p.z as f32])) {
                self.raw_log = None;
                self.raw_log_failed = true;
                tracing::warn!(event = "mapping.raw_scan_incomplete", %error,
                    "原始扫描记录已中断，继续建图；保存时将明确标记离线重判输入不完整");
            }
        }
    }
    pub(super) fn classify_online(&mut self, source: &crate::lio_ingress::AcceptedRegisteredCloud) -> Result<crate::dynamic_backend::MappedFrame, String> {
        let scan = source.scan();
        if let Some(frame) = scan.online_frame() { return Ok((*frame).clone()); }
        let binding = scan.binding();
        let domain = (binding.mapping_session_id.clone(), binding.mapping_session_generation);
        if self.source_domain.as_ref() != Some(&domain) {
            self.dufo = Default::default();
            self.current = None;
            self.source_domain = Some(domain);
            self.raw_log = None;
            self.raw_log_failed = false;
        }
        self.record_raw(scan);
        let points: Vec<_> = scan.body_points().iter().map(|p| binding.frozen_odom_from_body * p).collect();
        let origin = binding.sensor_origin_map.map(|p| binding.registration_map_from_odom.inverse() * Point3::from(p));
        let (mut frame, indices) = if let Some(origin) = origin {
            self.dufo.observe(&points, origin.coords.into(), scan.source_timestamp_ns(), binding.registration_map_from_odom)?
        } else {
            // 缺少真实原点时保留扫描，不猜测射线，也不产生自由证据。
            (crate::dynamic_backend::MappedFrame::unfiltered(), (0..points.len() as u32).collect())
        };
        frame.bind_points(source.map_points(), &indices);
        let frame = std::sync::Arc::new(frame);
        scan.record_static_indices(indices, &frame)?;
        self.current = Some(frame.clone());
        Ok((*frame).clone())
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
