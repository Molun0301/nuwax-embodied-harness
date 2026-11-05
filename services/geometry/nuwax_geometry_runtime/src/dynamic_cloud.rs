//! 用后续实测射线的空闲证据清除三维地图残影；不按时间淘汰静态结构。

use ahash::AHashSet as HashSet;

use nalgebra::Point3;

use super::voxel_index::VoxelIndex;
#[cfg(test)]
use super::voxel_key;
use super::{voxel_center_f64, VoxelKey};

#[derive(Default)]
pub(super) struct DynamicCloudFilter {
    dufo: crate::dynamic_backend::OnlineDufo,
    shared: Option<crate::dynamic_backend::MappedFrame>,
    removed_keys: Vec<VoxelKey>,
}

impl DynamicCloudFilter {
    pub(super) fn clear_frame(&mut self) {
        self.removed_keys.clear();
        self.shared = None;
    }

    pub(super) fn is_dynamic(&self, point: [f64; 3]) -> bool {
        self.shared.as_ref().is_some_and(|f| f.is_dynamic(point))
    }
    /// 当前扫描实际移除的体素，供确定性快照同步撤销；下一次扫描清空。
    pub(super) fn observe_dufo(
        &mut self,
        points: &[Point3<f64>],
        origin: [f64; 3],
        stamp: u64,
    ) -> Result<crate::dynamic_backend::MappedFrame, String> {
        self.dufo
            .observe(points, origin, stamp, nalgebra::Isometry3::identity())
            .map(|v| v.0)
    }
    pub(super) fn shared_frame(&self) -> Option<&crate::dynamic_backend::MappedFrame> {
        self.shared.as_ref()
    }
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
        self.shared = Some(
            self.dufo
                .observe(points, origin, stamp, nalgebra::Isometry3::identity())
                .expect("已校验的动态点云积分失败")
                .0,
        );
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
        self.shared.as_ref().is_some_and(|f| f.clears(p))
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
}
#[cfg(test)]
#[path = "dynamic_cloud_tests.rs"]
mod tests;
