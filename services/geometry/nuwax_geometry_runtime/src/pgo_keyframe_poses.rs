//! 仅保留实际接纳关键帧的原始位姿，供磁盘扫描重建使用。

use nalgebra::Isometry3;
use nuwax_codec::lcm::native_pgo::{
    validate_optimized_keyframes, OptimizedKeyframe, MAX_NATIVE_PGO_GRAPH_NODES,
};
use parking_lot::Mutex;
use std::{collections::BTreeMap, sync::Arc};

#[derive(Clone, Debug, Default)]
pub(crate) struct KeyframePoseStore(Arc<Mutex<BTreeMap<u64, Isometry3<f64>>>>);

impl KeyframePoseStore {
    pub(crate) fn new() -> Self {
        Self::default()
    }

    pub(crate) fn same_store(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }

    pub(crate) fn retain(
        &self,
        source_timestamp_ns: u64,
        pose: Isometry3<f64>,
    ) -> Result<(), String> {
        if source_timestamp_ns > i64::MAX as u64
            || !pose.to_homogeneous().iter().all(|value| value.is_finite())
        {
            return Err("PGO关键帧原始时间或位姿无效".into());
        }
        let mut poses = self.0.lock();
        if let Some(previous) = poses.get(&source_timestamp_ns) {
            return if previous == &pose {
                Ok(())
            } else {
                Err("同一PGO关键帧的原始位姿冲突".into())
            };
        }
        if poses.len() >= MAX_NATIVE_PGO_GRAPH_NODES as usize {
            return Err("PGO关键帧原始位姿超过既有图节点上限".into());
        }
        poses.insert(source_timestamp_ns, pose);
        Ok(())
    }

    pub(crate) fn snapshot_for(
        &self,
        trajectory: &[OptimizedKeyframe],
    ) -> Result<Vec<Isometry3<f64>>, String> {
        validate_optimized_keyframes(trajectory)?;
        let poses = self.0.lock();
        trajectory
            .iter()
            .map(|frame| {
                poses
                    .get(&(frame.source_timestamp_ns as u64))
                    .copied()
                    .ok_or_else(|| {
                        format!("PGO关键帧{}缺少精确原始位姿", frame.source_timestamp_ns)
                    })
            })
            .collect()
    }

    #[cfg(test)]
    pub(crate) fn len(&self) -> usize {
        self.0.lock().len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn trajectory(stamp: i64) -> OptimizedKeyframe {
        OptimizedKeyframe {
            source_timestamp_ns: stamp,
            rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
            ..Default::default()
        }
    }

    #[test]
    fn exact_keyframe_poses_are_shared_bounded_and_fail_closed() {
        let store = KeyframePoseStore::new();
        let clone = store.clone();
        assert!(store.same_store(&clone));
        assert!(!store.same_store(&KeyframePoseStore::new()));
        for stamp in 0..MAX_NATIVE_PGO_GRAPH_NODES as u64 {
            store
                .retain(stamp, Isometry3::translation(stamp as f64, 0.0, 0.0))
                .unwrap();
        }
        assert_eq!(clone.len(), MAX_NATIVE_PGO_GRAPH_NODES as usize);
        assert!(store
            .retain(MAX_NATIVE_PGO_GRAPH_NODES as u64, Isometry3::identity())
            .is_err());
        assert_eq!(store.len(), MAX_NATIVE_PGO_GRAPH_NODES as usize);
        assert_eq!(
            clone.snapshot_for(&[trajectory(12)]).unwrap(),
            vec![Isometry3::translation(12.0, 0.0, 0.0)]
        );
        assert!(store
            .snapshot_for(&[trajectory(MAX_NATIVE_PGO_GRAPH_NODES)])
            .is_err());
        assert!(store.retain(12, Isometry3::identity()).is_err());
        store
            .retain(12, Isometry3::translation(12.0, 0.0, 0.0))
            .unwrap();
        assert!(store.snapshot_for(&[trajectory(-1)]).is_err());
    }
}
