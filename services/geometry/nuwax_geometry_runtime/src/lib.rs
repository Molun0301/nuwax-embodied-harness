//! Geometry 可复用库入口。

mod dynamic_scene;

#[doc(hidden)]
pub mod mapping_backend;
mod mapping_commit;
mod offline_asset_export;
mod offline_canonical_cloud;
mod offline_replay;
mod offline_replay_owner;
mod persistent_occupancy;

pub use offline_asset_export::{
    export_offline_candidate_assets_v1, AssetReceiptV1, CandidateAssetEncodingV1,
    CandidateAssetMemberV1, DeferredAssetMemberV1, OfflineAssetExportError,
    OfflineCandidateAssetsV1,
};
pub use offline_replay::{
    OfflineBackendConfigV1, OfflineBackendKind, OfflineGeometryFacade, OfflineGeoreferenceV1,
    OfflineMapSnapshotSeedV2, OfflinePointXyzV1, OfflinePoseV1, OfflineRaytracingConfigV1,
    OfflineReplayConfigV1, OfflineReplayFrameV1, OfflineReplayIdentityV2, OfflineReplayRequestV1,
    OfflineReplayValidationError, OfflineReplayValidator, OfflineVoxelConfigV1,
    ValidatedOfflineReplayFrameV1, ValidatedOfflineReplayRequestV1,
};
pub use offline_replay_owner::{
    GeometryOfflineReplayOwner, OfflineCanonicalCheckpointProjectionV1,
    OfflineCanonicalCloudAssetProjectionV1, OfflineCanonicalCloudReceiptV1,
    OfflineCanonicalCommitIdV1, OfflineCanonicalOccupancyAssetProjectionV1,
    OfflineCanonicalOccupancyReceiptV1, OfflineGeometrySnapshotV2,
    OfflineReplayProcessingReceiptV1, ReplayBarrierIdentityV1,
};
