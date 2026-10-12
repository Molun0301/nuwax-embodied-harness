//! Small assigned fixtures; no HTTP, generated identities, or digest helpers.

use std::sync::Arc;
use std::time::Duration;

use crate::{AssignedGeoreferenceV3, AssignedMapIdentityV3, CloudQueryIdentityV1, LatLng,
    MappingCloudRequest};

pub(crate) const ASSET_ID: &str = "11111111111111111111111111111111";
pub(crate) const OTHER_ASSET_ID: &str = "22222222222222222222222222222222";
pub(crate) const RAW: &[u8] = br#"{"schema_version":1,"map_name":"shanghai_lab","map_frame_id":"map/cloud","source_crs":"EPSG:4326","projected_crs":"EPSG:3857","axis_convention":"projected_x_east_y_north_z_up","anchor_lat_deg":31.2304,"anchor_lng_deg":121.4737,"anchor_easting_m":13522390.428774875,"anchor_northing_m":3662707.257050403,"map_origin_x_m":10.0,"map_origin_y_m":-4.0,"map_from_projected_yaw_rad":0.0,"altitude_policy":"map_plane_constant","map_plane_z_m":1.25}"#;

pub(crate) fn anchor() -> LatLng { LatLng { lat: 31.2304, lng: 121.4737 } }

pub(crate) fn request() -> MappingCloudRequest {
    MappingCloudRequest {
        query_identity: CloudQueryIdentityV1 {
            query_request_id: "assigned-request".into(),
            producer: "mapping-cloud-test".into(),
            producer_epoch: 1,
            source_timestamp_ns: 100,
        },
        query_text: "destination".into(),
        active_map: AssignedMapIdentityV3 {
            asset_id: ASSET_ID.into(),
            activation_request_id: "activate-1".into(),
            checkpoint_id: "33333333333333333333333333333333".into(),
            map_name: "shanghai_lab".into(),
            map_frame_id: "map/cloud".into(),
            runtime_generation: 1,
            lifecycle_epoch: 2,
            mapping_session_id: "mapping-session-1".into(),
            mapping_session_generation: 3,
            backend_kind: "voxel".into(),
            backend_generation: 4,
            committed_occupancy_revision: 5,
            committed_occupancy_boot_nonce: [6; 16],
            committed_occupancy_commit_sequence: 7,
            committed_occupancy_source_timestamp_ns: 8,
        },
        georeference: Arc::new(AssignedGeoreferenceV3::from_json(RAW, ASSET_ID).unwrap()),
        caller_timeout: Duration::from_secs(5),
        provider_timeout: Duration::from_secs(4),
    }
}
