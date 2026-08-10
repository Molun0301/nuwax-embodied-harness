//! 离线重放专用的版本化 canonical 点云投影。

use std::collections::HashSet;

use nalgebra::Point3;

pub const OFFLINE_CANONICAL_CLOUD_PAYLOAD_SCHEMA_VERSION: u32 = 1;
pub const OFFLINE_CANONICAL_CLOUD_INITIAL_VOXEL_SIZE: f32 = 0.05;
const POINT_SIZE_BYTES: usize = 12;
const MAX_PAYLOAD_BYTES: usize = 16 * 1024 * 1024;
const MAX_POINTS: usize = MAX_PAYLOAD_BYTES / POINT_SIZE_BYTES;
const MAX_VOXEL_SIZE: f32 = 1_048_576.0;

type VoxelKey = (i32, i32, i32);

#[derive(Clone, Debug)]
pub(crate) struct OfflineCanonicalCloud {
    voxels: HashSet<VoxelKey>,
    voxel_size: f32,
    revision: u64,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct OfflineCanonicalCloudSnapshot {
    pub(crate) revision: u64,
    pub(crate) point_count: u32,
    pub(crate) voxel_size: f32,
    pub(crate) xyz_f32_le: Vec<u8>,
}

impl OfflineCanonicalCloud {
    pub(crate) fn new() -> Self {
        Self {
            voxels: HashSet::new(),
            voxel_size: OFFLINE_CANONICAL_CLOUD_INITIAL_VOXEL_SIZE,
            revision: 0,
        }
    }

    pub(crate) fn stage_frame(&self, points: &[Point3<f64>]) -> Result<Self, String> {
        if points.is_empty() {
            return Err("offline canonical cloud frame must not be empty".to_owned());
        }
        let mut staged = self.clone();
        for point in points {
            let key = voxel_key([point.x, point.y, point.z], staged.voxel_size)
                .ok_or_else(|| "offline canonical cloud contains an invalid point".to_owned())?;
            staged.voxels.insert(key);
        }
        staged.coarsen_to_fit()?;
        if staged.voxels.is_empty() {
            return Err("offline canonical cloud produced no voxels".to_owned());
        }
        Ok(staged)
    }

    pub(crate) fn commit_snapshot(&mut self) -> Result<OfflineCanonicalCloudSnapshot, String> {
        if self.voxels.is_empty() {
            return Err("offline canonical cloud cannot commit an empty snapshot".to_owned());
        }
        let revision = self
            .revision
            .checked_add(1)
            .ok_or_else(|| "offline canonical cloud revision exhausted".to_owned())?;
        let point_count = u32::try_from(self.voxels.len())
            .map_err(|_| "offline canonical cloud point count exceeds u32".to_owned())?;
        let capacity = self
            .voxels
            .len()
            .checked_mul(POINT_SIZE_BYTES)
            .ok_or_else(|| "offline canonical cloud payload size overflow".to_owned())?;
        if capacity > MAX_PAYLOAD_BYTES {
            return Err("offline canonical cloud payload exceeds capacity".to_owned());
        }
        let mut keys = self.voxels.iter().copied().collect::<Vec<_>>();
        keys.sort_unstable();
        let mut xyz_f32_le = Vec::with_capacity(capacity);
        for key in keys {
            for value in voxel_center_f32(key, self.voxel_size) {
                xyz_f32_le.extend_from_slice(&value.to_le_bytes());
            }
        }
        self.revision = revision;
        Ok(OfflineCanonicalCloudSnapshot {
            revision,
            point_count,
            voxel_size: self.voxel_size,
            xyz_f32_le,
        })
    }

    fn coarsen_to_fit(&mut self) -> Result<(), String> {
        while self.voxels.len() > MAX_POINTS {
            let next_voxel_size = self.voxel_size * 2.0;
            if !next_voxel_size.is_finite() || next_voxel_size > MAX_VOXEL_SIZE {
                return Err("offline canonical cloud cannot fit payload capacity".to_owned());
            }
            let previous_voxel_size = self.voxel_size;
            let previous = std::mem::take(&mut self.voxels);
            let mut coarsened = HashSet::with_capacity(previous.len().min(MAX_POINTS));
            for previous_key in previous {
                let point = voxel_center_f64(previous_key, previous_voxel_size);
                let key = voxel_key(point, next_voxel_size).ok_or_else(|| {
                    "offline canonical cloud coarsening produced an invalid point".to_owned()
                })?;
                coarsened.insert(key);
            }
            self.voxels = coarsened;
            self.voxel_size = next_voxel_size;
        }
        Ok(())
    }
}

fn voxel_center_f64(key: VoxelKey, voxel_size: f32) -> [f64; 3] {
    let voxel_size = f64::from(voxel_size);
    [
        (f64::from(key.0) + 0.5) * voxel_size,
        (f64::from(key.1) + 0.5) * voxel_size,
        (f64::from(key.2) + 0.5) * voxel_size,
    ]
}

fn voxel_center_f32(key: VoxelKey, voxel_size: f32) -> [f32; 3] {
    let point = voxel_center_f64(key, voxel_size);
    [point[0] as f32, point[1] as f32, point[2] as f32]
}

fn voxel_key(point: [f64; 3], voxel_size: f32) -> Option<VoxelKey> {
    if !voxel_size.is_finite() || voxel_size <= 0.0 || point.iter().any(|value| !value.is_finite())
    {
        return None;
    }
    let inv = 1.0 / f64::from(voxel_size);
    let mut key = [0_i32; 3];
    for (index, value) in point.into_iter().enumerate() {
        let scaled = (value * inv).floor();
        if scaled < f64::from(i32::MIN) || scaled > f64::from(i32::MAX) {
            return None;
        }
        key[index] = scaled as i32;
    }
    Some((key[0], key[1], key[2]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn snapshot_is_sorted_deduplicated_and_little_endian() {
        let cloud = OfflineCanonicalCloud::new();
        let mut staged = cloud
            .stage_frame(&[
                Point3::new(0.051, 0.0, 0.0),
                Point3::new(-0.001, 0.0, 0.0),
                Point3::new(0.052, 0.0, 0.0),
            ])
            .expect("stage");
        let snapshot = staged.commit_snapshot().expect("snapshot");
        assert_eq!(snapshot.revision, 1);
        assert_eq!(snapshot.point_count, 2);
        assert_eq!(snapshot.voxel_size, 0.05);
        assert_eq!(snapshot.xyz_f32_le.len(), 24);
        let values = snapshot
            .xyz_f32_le
            .chunks_exact(4)
            .map(|chunk| f32::from_le_bytes(chunk.try_into().unwrap()))
            .collect::<Vec<_>>();
        assert_eq!(values, vec![-0.025, 0.025, 0.025, 0.075, 0.025, 0.025]);
    }

    #[test]
    fn invalid_frame_does_not_mutate_source_state() {
        let cloud = OfflineCanonicalCloud::new();
        assert!(cloud
            .stage_frame(&[Point3::new(f64::NAN, 0.0, 0.0)])
            .is_err());
        assert_eq!(cloud.revision, 0);
        assert!(cloud.voxels.is_empty());
    }
}
