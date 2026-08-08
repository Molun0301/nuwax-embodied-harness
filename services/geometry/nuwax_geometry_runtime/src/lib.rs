//! Geometry 可复用库入口。

#[doc(hidden)]
pub mod mapping_backend;
mod offline_replay;
mod offline_replay_owner;

#[doc(hidden)]
pub mod __offline_replay_internal {
    pub use super::offline_replay_owner::{GeometryOfflineReplayOwner, OfflineGeometrySnapshotV1, OfflineReplayProcessingReceiptV1, ReplayBarrierIdentityV1};
}

pub use offline_replay::{
    OfflineBackendConfigV1, OfflineBackendKind, OfflineGeometryFacade, OfflineGeoreferenceV1,
    OfflinePointXyzV1, OfflinePoseV1, OfflineRaytracingConfigV1, OfflineReplayConfigV1,
    OfflineReplayFrameV1, OfflineReplayRequestV1, OfflineReplayValidationError,
    OfflineReplayValidator, OfflineVoxelConfigV1, ValidatedOfflineReplayFrameV1,
    ValidatedOfflineReplayRequestV1,
};
