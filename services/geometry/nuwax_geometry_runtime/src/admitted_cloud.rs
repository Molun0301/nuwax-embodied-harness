//! 原拥有方已配对的真实点云；辅助相机不借用 LiDAR 身份。
use crate::correction_points::CorrectionPoints;
use crate::lcm_runtime::mapping_observation::RegisteredObservation;
use crate::lio_ingress::{AcceptedRegisteredCloud, AdmittedRegisteredScan};
use nalgebra::Point3;
use nuwax_map_chunks::LayerSource;
use std::sync::Arc;

/// 只持有准入对象的共享引用；PGO 不复制原始点或重造来源。
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum AdmittedCloudSource {
    Lidar(AdmittedRegisteredScan),
    Auxiliary(RegisteredObservation),
}
impl AdmittedCloudSource {
    pub(crate) fn lidar(&self) -> Option<&AdmittedRegisteredScan> {
        match self { Self::Lidar(scan) => Some(scan), Self::Auxiliary(_) => None }
    }
    pub(crate) fn timestamp_ns(&self) -> u64 {
        match self { Self::Lidar(scan) => scan.source_timestamp_ns(), Self::Auxiliary(scan) => scan.timestamp_ns() }
    }
    pub(crate) fn sequence(&self) -> Result<u64, String> {
        match self {
            Self::Lidar(scan) => u64::try_from(scan.source_identity().source_sequence)
                .ok().filter(|sequence| *sequence > 0).ok_or_else(|| "invalid admitted LiDAR sequence".into()),
            Self::Auxiliary(scan) => Ok(scan.source_sequence()),
        }
    }
    pub(crate) fn layer_source(&self) -> Arc<LayerSource> {
        match self {
            Self::Lidar(scan) => {
                let source = scan.source_identity();
                Arc::new(LayerSource::Lidar { backend: source.backend,
                    runtime_generation: source.runtime_generation,
                    sensor_frame_id: scan.sensor_frame_id().to_owned() })
            }
            Self::Auxiliary(scan) => Arc::clone(scan.layer_source()),
        }
    }
    pub(crate) fn body_points(&self) -> &CorrectionPoints {
        match self { Self::Lidar(scan) => scan.body_points(), Self::Auxiliary(scan) => scan.body_points() }
    }
    pub(crate) fn original_index(&self, index: usize) -> Option<u32> {
        match self {
            Self::Lidar(scan) => (index < scan.body_points().len()).then(|| u32::try_from(index).ok()).flatten(),
            Self::Auxiliary(scan) => scan.original_indices().get(index).copied(),
        }
    }
}
impl From<AdmittedRegisteredScan> for AdmittedCloudSource {
    fn from(source: AdmittedRegisteredScan) -> Self { Self::Lidar(source) }
}

/// 注册后的原始地图坐标只分配一次，提交期间借用；归档仅保留机体坐标。
#[derive(Clone, Debug)]
pub(crate) enum AcceptedMappingCloud {
    Lidar(AcceptedRegisteredCloud),
    Auxiliary(Arc<AcceptedAuxiliaryCloud>),
}
#[derive(Debug)]
pub(crate) struct AcceptedAuxiliaryCloud {
    source: RegisteredObservation,
    map_points: Vec<Point3<f64>>,
}
impl AcceptedMappingCloud {
    pub(crate) fn auxiliary(source: RegisteredObservation) -> Self {
        let map_from_body = source.map_from_body();
        let map_points = source.body_points().iter().map(|point| map_from_body * point).collect();
        Self::Auxiliary(Arc::new(AcceptedAuxiliaryCloud { source, map_points }))
    }
    pub(crate) fn map_points(&self) -> &[Point3<f64>] {
        match self { Self::Lidar(source) => source.map_points(), Self::Auxiliary(source) => &source.map_points }
    }
    pub(crate) fn source(&self) -> AdmittedCloudSource {
        match self {
            Self::Lidar(source) => AdmittedCloudSource::Lidar(source.scan().clone()),
            Self::Auxiliary(source) => AdmittedCloudSource::Auxiliary(source.source.clone()),
        }
    }
    pub(crate) fn lidar(&self) -> Option<&AcceptedRegisteredCloud> {
        match self { Self::Lidar(source) => Some(source), Self::Auxiliary(_) => None }
    }
}
impl From<AcceptedRegisteredCloud> for AcceptedMappingCloud {
    fn from(source: AcceptedRegisteredCloud) -> Self { Self::Lidar(source) }
}
