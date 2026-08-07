//! Pure WGS84-to-map goal-intent adapter.
//!
//! This module deliberately has no transport, lease, planner, or command API.
//! Its output is a backend-local DTO that still requires the shared Batch02
//! goal arbiter to revalidate active-map identity and admit a canonical lease.

use thiserror::Error;

use crate::georeference::{
    canonical_zero, push_digest, push_f64, push_i64, push_text, push_u32, push_u64, valid_frame_id,
    GeoreferenceError, MapMetricPosition, Sha256Digest, VerifiedGeoreferenceV1,
    WEB_MERCATOR_MAX_LATITUDE_DEG,
};
use crate::LatLng;

/// Exact schema version for the backend-local mapping-cloud intent.
pub const MAPPING_CLOUD_GOAL_INTENT_SCHEMA_VERSION: u32 = 1;

const GOAL_INTENT_DIGEST_DOMAIN: &[u8] = b"nuwax.mapping-cloud.goal-intent.v1\0";
const MAX_QUERY_ID_BYTES: usize = 128;
const MAX_PRODUCER_BYTES: usize = 128;
const MAX_MAP_NAME_BYTES: usize = 255;

/// Immutable identity of one cloud-map query result.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CloudQueryIdentityV1 {
    /// Stable request identity used for replay/conflict correlation.
    pub query_request_id: String,
    /// Stable producer name.
    pub producer: String,
    /// Non-zero producer lifecycle epoch.
    pub producer_epoch: u64,
    /// Positive source timestamp in nanoseconds.
    pub source_timestamp_ns: i64,
}

impl CloudQueryIdentityV1 {
    fn validate(&self) -> Result<(), GoalAdapterError> {
        if !valid_identity_token(&self.query_request_id, MAX_QUERY_ID_BYTES) {
            return Err(GoalAdapterError::InvalidQueryRequestId);
        }
        if !valid_identity_token(&self.producer, MAX_PRODUCER_BYTES) {
            return Err(GoalAdapterError::InvalidProducer);
        }
        if self.producer_epoch == 0 {
            return Err(GoalAdapterError::UnpopulatedCounter("producer_epoch"));
        }
        if self.source_timestamp_ns <= 0 {
            return Err(GoalAdapterError::InvalidSourceTimestamp(
                self.source_timestamp_ns,
            ));
        }
        Ok(())
    }
}

/// Immutable active-map snapshot carried into shared goal arbitration.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ActiveMapIdentityV1 {
    /// Finalized map name.
    pub map_name: String,
    /// Exact map frame.
    pub map_frame_id: String,
    /// Top-level finalized asset manifest digest.
    pub asset_manifest_sha256: Sha256Digest,
    /// Finalized asset bundle digest.
    pub asset_bundle_sha256: Sha256Digest,
    /// Digest over exact georeference JSON bytes.
    pub georeference_sha256: Sha256Digest,
    /// Canonical activation identity.
    pub activation_identity_sha256: Sha256Digest,
    /// Non-zero Geometry runtime generation.
    pub runtime_generation: u64,
    /// Non-zero mapping session generation.
    pub mapping_session_generation: u64,
    /// Non-zero committed occupancy revision.
    pub committed_occupancy_revision: u64,
    /// Content digest for that committed occupancy revision.
    pub committed_occupancy_sha256: Sha256Digest,
}

impl ActiveMapIdentityV1 {
    fn validate(&self) -> Result<(), GoalAdapterError> {
        if !valid_map_name(&self.map_name) {
            return Err(GoalAdapterError::InvalidMapName);
        }
        if !valid_frame_id(&self.map_frame_id) {
            return Err(GoalAdapterError::InvalidMapFrame);
        }
        for (field, value) in [
            ("runtime_generation", self.runtime_generation),
            (
                "mapping_session_generation",
                self.mapping_session_generation,
            ),
            (
                "committed_occupancy_revision",
                self.committed_occupancy_revision,
            ),
        ] {
            if value == 0 {
                return Err(GoalAdapterError::UnpopulatedCounter(field));
            }
        }
        Ok(())
    }
}

/// Terminal heading policy carried by a mapping-cloud goal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TerminalHeadingPolicy {
    /// The query contains no heading; the shared arbiter must choose policy.
    Unspecified,
}

impl TerminalHeadingPolicy {
    const fn canonical_code(self) -> u32 {
        match self {
            Self::Unspecified => 0,
        }
    }
}

/// Backend-local metric goal intent awaiting shared arbitration.
#[derive(Debug, Clone, PartialEq)]
pub struct MappingCloudGoalIntentV1 {
    /// Exact DTO schema version.
    schema_version: u32,
    /// Cloud query identity and source time.
    query: CloudQueryIdentityV1,
    /// Original WGS84 query result after strict validation.
    source_wgs84: LatLng,
    /// Converted local metric position.
    map_position: MapMetricPosition,
    /// Heading remains unspecified; no quaternion is fabricated.
    terminal_heading: TerminalHeadingPolicy,
    /// Active-map identity snapshot to be revalidated by shared arbitration.
    active_map: ActiveMapIdentityV1,
    /// SHA-256 over the canonical intent body, excluding this field.
    payload_sha256: Sha256Digest,
}

impl MappingCloudGoalIntentV1 {
    /// Return the exact DTO schema version.
    #[must_use]
    pub const fn schema_version(&self) -> u32 {
        self.schema_version
    }

    /// Borrow the immutable query identity.
    #[must_use]
    pub const fn query(&self) -> &CloudQueryIdentityV1 {
        &self.query
    }

    /// Return the validated source WGS84 position.
    #[must_use]
    pub const fn source_wgs84(&self) -> LatLng {
        self.source_wgs84
    }

    /// Return the converted local metric position.
    #[must_use]
    pub const fn map_position(&self) -> MapMetricPosition {
        self.map_position
    }

    /// Return the frozen terminal heading policy.
    #[must_use]
    pub const fn terminal_heading(&self) -> TerminalHeadingPolicy {
        self.terminal_heading
    }

    /// Borrow the active-map snapshot that arbitration must revalidate.
    #[must_use]
    pub const fn active_map(&self) -> &ActiveMapIdentityV1 {
        &self.active_map
    }

    /// Return the canonical intent payload digest.
    #[must_use]
    pub const fn payload_sha256(&self) -> Sha256Digest {
        self.payload_sha256
    }

    /// Encode the validated intent body as a deterministic binary preimage.
    ///
    /// Exact order after the NUL-terminated domain is: schema `u32`, query ID
    /// and producer as length-prefixed UTF-8, producer epoch `u64`, source time
    /// `i64`, source lat/lng f64, map name/frame as length-prefixed UTF-8,
    /// metric XYZ f64, heading `u32`, four asset/activation digests, runtime and
    /// mapping generations `u64`, occupancy revision `u64`, and occupancy hash.
    /// All numbers are big-endian and floating zeroes use positive-zero bits.
    ///
    /// # Errors
    /// Rejects any invalid field before producing identity bytes.
    pub fn canonical_preimage(&self) -> Result<Vec<u8>, GoalAdapterError> {
        self.validate_body()?;
        Ok(encode_goal_intent_preimage(
            self.schema_version,
            &self.query,
            self.source_wgs84,
            self.map_position,
            self.terminal_heading,
            &self.active_map,
        ))
    }

    /// Revalidate all fields and the canonical self-digest.
    ///
    /// # Errors
    /// Rejects malformed identity, non-finite position data, or payload tampering.
    pub fn verify(&self) -> Result<(), GoalAdapterError> {
        let expected = Sha256Digest::of_bytes(&self.canonical_preimage()?);
        if expected != self.payload_sha256 {
            return Err(GoalAdapterError::PayloadDigestMismatch {
                expected,
                actual: self.payload_sha256,
            });
        }
        Ok(())
    }

    fn validate_body(&self) -> Result<(), GoalAdapterError> {
        if self.schema_version != MAPPING_CLOUD_GOAL_INTENT_SCHEMA_VERSION {
            return Err(GoalAdapterError::UnsupportedIntentSchema(
                self.schema_version,
            ));
        }
        self.query.validate()?;
        self.active_map.validate()?;
        validate_wgs84(self.source_wgs84)?;
        for (field, value) in [
            ("map_position.x_m", self.map_position.x_m),
            ("map_position.y_m", self.map_position.y_m),
            ("map_position.z_m", self.map_position.z_m),
        ] {
            if !value.is_finite() {
                return Err(GoalAdapterError::InvalidMapPosition(field));
            }
            if value == 0.0 && value.is_sign_negative() {
                return Err(GoalAdapterError::NegativeZero(field));
            }
        }
        Ok(())
    }
}

/// Convert a verified WGS84 query result into a sealed backend-local intent.
///
/// This function performs no I/O and has no lease, planner, or command side
/// effects. Its active-map snapshot must be compared again by the shared goal
/// arbiter immediately before canonical lease admission.
///
/// # Errors
/// Rejects invalid query/map identity, map/frame/georeference mismatches, or
/// any georeference conversion failure.
pub fn adapt_wgs84_goal_intent(
    georeference: &VerifiedGeoreferenceV1,
    query: CloudQueryIdentityV1,
    active_map: ActiveMapIdentityV1,
    source_wgs84: LatLng,
) -> Result<MappingCloudGoalIntentV1, GoalAdapterError> {
    query.validate()?;
    active_map.validate()?;
    let contract = georeference.contract();
    if active_map.map_name != contract.map_name {
        return Err(GoalAdapterError::MapNameMismatch {
            expected: contract.map_name.clone(),
            actual: active_map.map_name,
        });
    }
    if active_map.map_frame_id != contract.map_frame_id {
        return Err(GoalAdapterError::MapFrameMismatch {
            expected: contract.map_frame_id.clone(),
            actual: active_map.map_frame_id,
        });
    }
    if active_map.georeference_sha256 != georeference.raw_sha256() {
        return Err(GoalAdapterError::GeoreferenceDigestMismatch {
            expected: georeference.raw_sha256(),
            actual: active_map.georeference_sha256,
        });
    }
    let map_position = georeference.project_wgs84(source_wgs84)?;
    let source_wgs84 = LatLng {
        lat: canonical_zero(source_wgs84.lat),
        lng: canonical_zero(source_wgs84.lng),
    };
    let terminal_heading = TerminalHeadingPolicy::Unspecified;
    let payload_sha256 = Sha256Digest::of_bytes(&encode_goal_intent_preimage(
        MAPPING_CLOUD_GOAL_INTENT_SCHEMA_VERSION,
        &query,
        source_wgs84,
        map_position,
        terminal_heading,
        &active_map,
    ));
    Ok(MappingCloudGoalIntentV1 {
        schema_version: MAPPING_CLOUD_GOAL_INTENT_SCHEMA_VERSION,
        query,
        source_wgs84,
        map_position,
        terminal_heading,
        active_map,
        payload_sha256,
    })
}

/// Fail-closed mapping-cloud goal adaptation errors.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum GoalAdapterError {
    /// The backend-local intent schema was not exactly one.
    #[error("unsupported mapping-cloud goal intent schema {0}")]
    UnsupportedIntentSchema(u32),
    /// The stable query/request ID was not canonical bounded ASCII.
    #[error("query_request_id must be canonical bounded ASCII identity text")]
    InvalidQueryRequestId,
    /// The producer name was not canonical bounded ASCII.
    #[error("producer must be canonical bounded ASCII identity text")]
    InvalidProducer,
    /// A required generation/revision counter was zero.
    #[error("{0} must be non-zero")]
    UnpopulatedCounter(&'static str),
    /// Source time was absent or negative.
    #[error("source_timestamp_ns must be positive, got {0}")]
    InvalidSourceTimestamp(i64),
    /// Active map name was non-canonical or unbounded.
    #[error("active map_name must be trimmed, control-free, and bounded")]
    InvalidMapName,
    /// Active map frame was not canonical.
    #[error("active map_frame_id must use bounded slash-separated ASCII tokens")]
    InvalidMapFrame,
    /// WGS84 latitude was non-finite or outside Web Mercator.
    #[error("source WGS84 latitude is invalid: {0}")]
    InvalidLatitude(f64),
    /// WGS84 longitude was non-finite or outside its domain.
    #[error("source WGS84 longitude is invalid: {0}")]
    InvalidLongitude(f64),
    /// A source or local coordinate used negative-zero bits.
    #[error("{0} must encode zero as positive zero")]
    NegativeZero(&'static str),
    /// A local metric component was non-finite.
    #[error("{0} must be finite")]
    InvalidMapPosition(&'static str),
    /// The active map name did not match the verified georeference.
    #[error("active map name mismatch: expected {expected:?}, got {actual:?}")]
    MapNameMismatch {
        /// Georeference map name.
        expected: String,
        /// Active-map snapshot name.
        actual: String,
    },
    /// The active map frame did not match the verified georeference.
    #[error("active map frame mismatch: expected {expected:?}, got {actual:?}")]
    MapFrameMismatch {
        /// Georeference map frame.
        expected: String,
        /// Active-map snapshot frame.
        actual: String,
    },
    /// The active-map georeference digest did not match exact verified bytes.
    #[error("active georeference SHA-256 mismatch: expected {expected}, got {actual}")]
    GeoreferenceDigestMismatch {
        /// Exact verified raw-document digest.
        expected: Sha256Digest,
        /// Active-map snapshot digest.
        actual: Sha256Digest,
    },
    /// Georeference validation or conversion failed.
    #[error(transparent)]
    Georeference(#[from] GeoreferenceError),
    /// The DTO body no longer matched its canonical digest.
    #[error("mapping-cloud goal payload SHA-256 mismatch: expected {expected}, got {actual}")]
    PayloadDigestMismatch {
        /// Recomputed digest.
        expected: Sha256Digest,
        /// Supplied digest.
        actual: Sha256Digest,
    },
}

fn valid_identity_token(value: &str, maximum_bytes: usize) -> bool {
    !value.is_empty()
        && value.len() <= maximum_bytes
        && value.is_ascii()
        && value.as_bytes()[0].is_ascii_alphanumeric()
        && value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.' | b':' | b'/')
        })
}

fn encode_goal_intent_preimage(
    schema_version: u32,
    query: &CloudQueryIdentityV1,
    source_wgs84: LatLng,
    map_position: MapMetricPosition,
    terminal_heading: TerminalHeadingPolicy,
    active_map: &ActiveMapIdentityV1,
) -> Vec<u8> {
    let mut output = Vec::with_capacity(384);
    output.extend_from_slice(GOAL_INTENT_DIGEST_DOMAIN);
    push_u32(&mut output, schema_version);
    push_text(&mut output, &query.query_request_id);
    push_text(&mut output, &query.producer);
    push_u64(&mut output, query.producer_epoch);
    push_i64(&mut output, query.source_timestamp_ns);
    push_f64(&mut output, source_wgs84.lat);
    push_f64(&mut output, source_wgs84.lng);
    push_text(&mut output, &active_map.map_name);
    push_text(&mut output, &active_map.map_frame_id);
    push_f64(&mut output, map_position.x_m);
    push_f64(&mut output, map_position.y_m);
    push_f64(&mut output, map_position.z_m);
    push_u32(&mut output, terminal_heading.canonical_code());
    push_digest(&mut output, active_map.asset_manifest_sha256);
    push_digest(&mut output, active_map.asset_bundle_sha256);
    push_digest(&mut output, active_map.georeference_sha256);
    push_digest(&mut output, active_map.activation_identity_sha256);
    push_u64(&mut output, active_map.runtime_generation);
    push_u64(&mut output, active_map.mapping_session_generation);
    push_u64(&mut output, active_map.committed_occupancy_revision);
    push_digest(&mut output, active_map.committed_occupancy_sha256);
    output
}

fn valid_map_name(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= MAX_MAP_NAME_BYTES
        && value.trim() == value
        && !value.chars().any(char::is_control)
}

fn validate_wgs84(value: LatLng) -> Result<(), GoalAdapterError> {
    if !value.lat.is_finite()
        || !(-WEB_MERCATOR_MAX_LATITUDE_DEG..=WEB_MERCATOR_MAX_LATITUDE_DEG).contains(&value.lat)
    {
        return Err(GoalAdapterError::InvalidLatitude(value.lat));
    }
    if !value.lng.is_finite() || !(-180.0..=180.0).contains(&value.lng) {
        return Err(GoalAdapterError::InvalidLongitude(value.lng));
    }
    if value.lat == 0.0 && value.lat.is_sign_negative() {
        return Err(GoalAdapterError::NegativeZero("source_wgs84.lat"));
    }
    if value.lng == 0.0 && value.lng.is_sign_negative() {
        return Err(GoalAdapterError::NegativeZero("source_wgs84.lng"));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use crate::georeference::GeoreferenceBindingV1;

    use super::*;

    fn python_georeference_json() -> &'static [u8] {
        br#"{"altitude_policy":"map_plane_constant","anchor_easting_m":13522390.428774875,"anchor_lat_deg":31.2304,"anchor_lng_deg":121.4737,"anchor_northing_m":3662707.257050403,"axis_convention":"projected_x_east_y_north_z_up","map_frame_id":"map/cloud","map_from_projected_yaw_rad":0.0,"map_name":"shanghai_lab","map_origin_x_m":10.0,"map_origin_y_m":-4.0,"map_plane_z_m":1.25,"projected_crs":"EPSG:3857","schema_version":1,"source_crs":"EPSG:4326"}
"#
    }

    fn legacy_struct_order_georeference_json() -> &'static [u8] {
        br#"{"schema_version":1,"map_name":"shanghai_lab","map_frame_id":"map/cloud","source_crs":"EPSG:4326","projected_crs":"EPSG:3857","axis_convention":"projected_x_east_y_north_z_up","anchor_lat_deg":31.2304,"anchor_lng_deg":121.4737,"anchor_easting_m":13522390.428774875,"anchor_northing_m":3662707.257050403,"map_origin_x_m":10.0,"map_origin_y_m":-4.0,"map_from_projected_yaw_rad":0.0,"altitude_policy":"map_plane_constant","map_plane_z_m":1.25}"#
    }

    fn georeference_from_raw(raw: &[u8]) -> VerifiedGeoreferenceV1 {
        VerifiedGeoreferenceV1::from_json(raw, &GeoreferenceBindingV1::for_document(raw))
            .expect("verify Python-issued georeference")
    }

    fn georeference() -> VerifiedGeoreferenceV1 {
        georeference_from_raw(python_georeference_json())
    }

    fn decode_hex(value: &str) -> Vec<u8> {
        assert_eq!(value.len() % 2, 0, "hex fixture must contain pairs");
        value
            .as_bytes()
            .chunks_exact(2)
            .map(|pair| {
                let pair = std::str::from_utf8(pair).expect("hex fixture is ASCII");
                u8::from_str_radix(pair, 16).expect("hex fixture is valid")
            })
            .collect()
    }

    fn query() -> CloudQueryIdentityV1 {
        CloudQueryIdentityV1 {
            query_request_id: "query-20260731-0001".into(),
            producer: "nuwax-mapping-cloud".into(),
            producer_epoch: 7,
            source_timestamp_ns: 1_753_935_600_000_000_000,
        }
    }

    fn active_map(georeference: &VerifiedGeoreferenceV1) -> ActiveMapIdentityV1 {
        ActiveMapIdentityV1 {
            map_name: georeference.contract().map_name.clone(),
            map_frame_id: georeference.contract().map_frame_id.clone(),
            asset_manifest_sha256: Sha256Digest::of_bytes(b"asset manifest"),
            asset_bundle_sha256: Sha256Digest::of_bytes(b"asset bundle"),
            georeference_sha256: georeference.raw_sha256(),
            activation_identity_sha256: Sha256Digest::of_bytes(b"activation identity"),
            runtime_generation: 11,
            mapping_session_generation: 13,
            committed_occupancy_revision: 17,
            committed_occupancy_sha256: Sha256Digest::of_bytes(b"occupancy payload"),
        }
    }

    fn source(georeference: &VerifiedGeoreferenceV1) -> LatLng {
        LatLng {
            lat: georeference.contract().anchor_lat_deg + 0.001,
            lng: georeference.contract().anchor_lng_deg,
        }
    }

    #[test]
    fn adapter_seals_metric_unspecified_heading_intent() {
        let georeference = georeference();
        let intent = adapt_wgs84_goal_intent(
            &georeference,
            query(),
            active_map(&georeference),
            source(&georeference),
        )
        .expect("adapt valid goal");
        assert_eq!(
            intent.schema_version(),
            MAPPING_CLOUD_GOAL_INTENT_SCHEMA_VERSION
        );
        assert_eq!(
            intent.terminal_heading(),
            TerminalHeadingPolicy::Unspecified
        );
        assert_eq!(intent.map_position().x_m.to_bits(), 10.0_f64.to_bits());
        assert_eq!(intent.map_position().y_m.to_bits(), 0x405f_8bdc_6deb_0000);
        assert_eq!(intent.map_position().z_m, 1.25);
        assert_eq!(
            intent.active_map().georeference_sha256,
            georeference.raw_sha256()
        );
        assert_eq!(
            georeference.raw_sha256().to_lower_hex(),
            "bd79c32346a190c0ef227ffe0b27084cb0a71563297723d54ebe54e557d9a480"
        );
        intent.verify().expect("verify sealed intent");
    }

    #[test]
    fn canonical_preimage_is_deterministic_and_domain_separated() {
        let georeference = georeference();
        let intent = adapt_wgs84_goal_intent(
            &georeference,
            query(),
            active_map(&georeference),
            source(&georeference),
        )
        .expect("adapt canonical fixture");
        let first = intent.canonical_preimage().expect("canonical preimage");
        let second = intent.canonical_preimage().expect("stable preimage");
        assert_eq!(first, second);
        assert!(first.starts_with(GOAL_INTENT_DIGEST_DOMAIN));
        assert_eq!(first.len(), 358);
        let python_preimage = decode_hex(concat!(
            "6e757761782e6d617070696e672d636c6f75642e676f616c2d696e74656e742e763100",
            "000000010000001371756572792d32303236303733312d30303031000000136e757761",
            "782d6d617070696e672d636c6f7564000000000000000718573b22ace96000403f3b3d",
            "07c84b5e405e5e5119ce075f0000000c7368616e676861695f6c6162000000096d6170",
            "2f636c6f75644024000000000000405f8bdc6deb00003ff4000000000000000000000d",
            "8addb628b5adf9475633f32612b54a0ab2a6cacac69477d887d3c17dacbcdcd5557b0",
            "0692cc5f50e55dcacdffbbb62ff2f484b3850ffb0d2ea1bac082451a7bd79c32346a19",
            "0c0ef227ffe0b27084cb0a71563297723d54ebe54e557d9a4806c4e92d1a1bb00f51f",
            "70185443fab5343ee12fcbd939601ad827eadfba69fe6c000000000000000b00000000",
            "0000000d00000000000000118798e94238d24c318934c7524e826d728b0fd02f63668d",
            "fe6eaf1e5170621627"
        ));
        assert_eq!(first, python_preimage);
        assert_eq!(
            Sha256Digest::of_bytes(&first).to_lower_hex(),
            "e90ec247326f1ea7843571354d39475f5143a72dcf5851b741148b88658de8ce"
        );
        assert_ne!(
            Sha256Digest::of_bytes(&first).to_lower_hex(),
            "1c121051c8d5904901d17426c75fc37e473eed05503bff17e6dd005bfd489cd6"
        );
        assert_ne!(
            Sha256Digest::of_bytes(&first).to_lower_hex(),
            "14324f23c1b737135b8ef6556abffe3748c7e1e088413b4ef40377c29cbedd37"
        );
        let mut stale = intent.clone();
        stale.payload_sha256 = Sha256Digest::from_lower_hex(
            "1c121051c8d5904901d17426c75fc37e473eed05503bff17e6dd005bfd489cd6",
        )
        .expect("parse stale digest fixture");
        assert!(matches!(
            stale.verify(),
            Err(GoalAdapterError::PayloadDigestMismatch { actual, .. })
                if actual == stale.payload_sha256
        ));
        assert_eq!(intent.payload_sha256(), Sha256Digest::of_bytes(&first));
    }

    #[test]
    fn python_math_order_closes_the_reviewers_isolated_digest_vector() {
        let georeference = georeference_from_raw(legacy_struct_order_georeference_json());
        let intent = adapt_wgs84_goal_intent(
            &georeference,
            query(),
            active_map(&georeference),
            source(&georeference),
        )
        .expect("adapt isolated reviewer fixture");
        assert_eq!(intent.map_position().x_m.to_bits(), 10.0_f64.to_bits());
        assert_eq!(
            intent.payload_sha256().to_lower_hex(),
            "ad5eb407d7df2cc442ed78e3091d83bcdb8e95cb270c177fd31f6d858f60e2e1"
        );
        assert_ne!(
            intent.payload_sha256().to_lower_hex(),
            "1c121051c8d5904901d17426c75fc37e473eed05503bff17e6dd005bfd489cd6"
        );
    }

    #[test]
    fn same_request_replays_exactly_and_conflicting_payload_changes_digest() {
        let georeference = georeference();
        let first = adapt_wgs84_goal_intent(
            &georeference,
            query(),
            active_map(&georeference),
            source(&georeference),
        )
        .expect("adapt first goal");
        let replay = adapt_wgs84_goal_intent(
            &georeference,
            query(),
            active_map(&georeference),
            source(&georeference),
        )
        .expect("adapt replay");
        assert_eq!(first, replay);

        let mut changed_source = source(&georeference);
        changed_source.lng += 0.000_01;
        let conflict = adapt_wgs84_goal_intent(
            &georeference,
            query(),
            active_map(&georeference),
            changed_source,
        )
        .expect("adapt conflicting body");
        assert_ne!(first.payload_sha256, conflict.payload_sha256);

        let mut switched_map = active_map(&georeference);
        switched_map.runtime_generation += 1;
        let switched =
            adapt_wgs84_goal_intent(&georeference, query(), switched_map, source(&georeference))
                .expect("adapt new generation");
        assert_ne!(first.payload_sha256, switched.payload_sha256);
    }

    #[test]
    fn verify_rejects_tampering_without_resealing() {
        let georeference = georeference();
        let intent = adapt_wgs84_goal_intent(
            &georeference,
            query(),
            active_map(&georeference),
            source(&georeference),
        )
        .expect("adapt valid goal");

        let mut tampered = intent.clone();
        tampered.map_position.x_m += 1.0;
        assert!(matches!(
            tampered.verify(),
            Err(GoalAdapterError::PayloadDigestMismatch { .. })
        ));

        let mut tampered = intent.clone();
        tampered.active_map.committed_occupancy_revision += 1;
        assert!(matches!(
            tampered.verify(),
            Err(GoalAdapterError::PayloadDigestMismatch { .. })
        ));

        let mut malformed = intent.clone();
        malformed.map_position.y_m = f64::INFINITY;
        assert!(matches!(
            malformed.verify(),
            Err(GoalAdapterError::InvalidMapPosition("map_position.y_m"))
        ));

        let mut malformed = intent.clone();
        malformed.schema_version = 2;
        assert!(matches!(
            malformed.verify(),
            Err(GoalAdapterError::UnsupportedIntentSchema(2))
        ));

        let mut malformed = intent;
        malformed.map_position.x_m = -0.0;
        assert!(matches!(
            malformed.verify(),
            Err(GoalAdapterError::NegativeZero("map_position.x_m"))
        ));
    }

    #[test]
    fn adapter_rejects_map_frame_and_georeference_identity_mismatch() {
        let georeference = georeference();
        let mut map = active_map(&georeference);
        map.map_name = "other_map".into();
        assert!(matches!(
            adapt_wgs84_goal_intent(&georeference, query(), map, source(&georeference)),
            Err(GoalAdapterError::MapNameMismatch { .. })
        ));

        let mut map = active_map(&georeference);
        map.map_frame_id = "map/other".into();
        assert!(matches!(
            adapt_wgs84_goal_intent(&georeference, query(), map, source(&georeference)),
            Err(GoalAdapterError::MapFrameMismatch { .. })
        ));

        let mut map = active_map(&georeference);
        map.georeference_sha256 = Sha256Digest::of_bytes(b"other georeference");
        assert!(matches!(
            adapt_wgs84_goal_intent(&georeference, query(), map, source(&georeference)),
            Err(GoalAdapterError::GeoreferenceDigestMismatch { .. })
        ));
    }

    #[test]
    fn query_identity_is_strict_and_bounded() {
        let georeference = georeference();
        for invalid in ["", " query", "query value", "query@value"] {
            let mut query = query();
            query.query_request_id = invalid.into();
            assert!(matches!(
                adapt_wgs84_goal_intent(
                    &georeference,
                    query,
                    active_map(&georeference),
                    source(&georeference)
                ),
                Err(GoalAdapterError::InvalidQueryRequestId)
            ));
        }

        let mut invalid = query();
        invalid.query_request_id = "q".repeat(MAX_QUERY_ID_BYTES + 1);
        assert!(matches!(
            adapt_wgs84_goal_intent(
                &georeference,
                invalid,
                active_map(&georeference),
                source(&georeference)
            ),
            Err(GoalAdapterError::InvalidQueryRequestId)
        ));

        let mut invalid = query();
        invalid.producer = "cloud owner".into();
        assert!(matches!(
            adapt_wgs84_goal_intent(
                &georeference,
                invalid,
                active_map(&georeference),
                source(&georeference)
            ),
            Err(GoalAdapterError::InvalidProducer)
        ));

        let mut invalid = query();
        invalid.producer_epoch = 0;
        assert!(matches!(
            adapt_wgs84_goal_intent(
                &georeference,
                invalid,
                active_map(&georeference),
                source(&georeference)
            ),
            Err(GoalAdapterError::UnpopulatedCounter("producer_epoch"))
        ));

        let mut invalid = query();
        invalid.source_timestamp_ns = 0;
        assert!(matches!(
            adapt_wgs84_goal_intent(
                &georeference,
                invalid,
                active_map(&georeference),
                source(&georeference)
            ),
            Err(GoalAdapterError::InvalidSourceTimestamp(0))
        ));
    }

    #[test]
    fn active_map_identity_rejects_invalid_names_frames_and_zero_counters() {
        let georeference = georeference();
        let mut invalid = active_map(&georeference);
        invalid.map_name = " map".into();
        assert!(matches!(
            adapt_wgs84_goal_intent(&georeference, query(), invalid, source(&georeference)),
            Err(GoalAdapterError::InvalidMapName)
        ));

        let mut invalid = active_map(&georeference);
        invalid.map_frame_id = "/map".into();
        assert!(matches!(
            adapt_wgs84_goal_intent(&georeference, query(), invalid, source(&georeference)),
            Err(GoalAdapterError::InvalidMapFrame)
        ));

        for field in [
            "runtime_generation",
            "mapping_session_generation",
            "committed_occupancy_revision",
        ] {
            let mut invalid = active_map(&georeference);
            match field {
                "runtime_generation" => invalid.runtime_generation = 0,
                "mapping_session_generation" => invalid.mapping_session_generation = 0,
                "committed_occupancy_revision" => invalid.committed_occupancy_revision = 0,
                _ => unreachable!(),
            }
            assert!(matches!(
                adapt_wgs84_goal_intent(
                    &georeference,
                    query(),
                    invalid,
                    source(&georeference)
                ),
                Err(GoalAdapterError::UnpopulatedCounter(actual)) if actual == field
            ));
        }
    }

    #[test]
    fn hostile_wgs84_is_rejected_before_an_intent_exists() {
        let georeference = georeference();
        for source in [
            LatLng {
                lat: f64::NAN,
                lng: 0.0,
            },
            LatLng {
                lat: WEB_MERCATOR_MAX_LATITUDE_DEG + 1.0,
                lng: 0.0,
            },
            LatLng {
                lat: 0.0,
                lng: 181.0,
            },
            LatLng {
                lat: georeference.contract().anchor_lat_deg + 2.0,
                lng: georeference.contract().anchor_lng_deg,
            },
        ] {
            assert!(adapt_wgs84_goal_intent(
                &georeference,
                query(),
                active_map(&georeference),
                source
            )
            .is_err());
        }
    }
}
