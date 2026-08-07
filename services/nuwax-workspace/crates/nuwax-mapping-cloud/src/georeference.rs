//! Immutable georeference contract for mapping-cloud goal conversion.

use std::f64::consts::PI;
use std::fmt;

use serde::{Deserialize, Serialize};
use serde_json::value::RawValue;
use thiserror::Error;

use crate::coords::wgs84_to_web_mercator;
use crate::LatLng;

/// Exact schema version accepted by the georeference converter.
pub const GEOREFERENCE_SCHEMA_VERSION: u32 = 1;
/// Canonical bundle-relative path for the georeference document.
pub const GEOREFERENCE_CANONICAL_PATH: &str = "georeference/georeference-v1.json";
/// Exact geographic source coordinate reference system.
pub const GEOREFERENCE_SOURCE_CRS: &str = "EPSG:4326";
/// Exact projected coordinate reference system.
pub const GEOREFERENCE_PROJECTED_CRS: &str = "EPSG:3857";
/// Exact projected-axis convention.
pub const GEOREFERENCE_AXIS_CONVENTION: &str = "projected_x_east_y_north_z_up";
/// Exact altitude policy for v1.
pub const GEOREFERENCE_ALTITUDE_POLICY: &str = "map_plane_constant";
/// Maximum latitude representable by Web Mercator.
pub const WEB_MERCATOR_MAX_LATITUDE_DEG: f64 = 85.051_128_779_806_6;
/// Maximum projected offset accepted for a local cloud goal.
pub const MAX_LOCAL_GOAL_OFFSET_M: f64 = 100_000.0;
/// Maximum absolute map-frame coordinate accepted by the v1 converter.
pub const MAX_ABSOLUTE_MAP_COORDINATE_M: f64 = 1_000_000_000.0;

const MAX_IDENTITY_BYTES: usize = 255;
const MAX_GEOREFERENCE_JSON_BYTES: usize = 64 * 1024;
const MAX_JSON_NUMBER_TOKEN_BYTES: usize = 128;
const ANCHOR_PROJECTION_TOLERANCE_M: f64 = 1.0e-6;
const GEOREFERENCE_DIGEST_DOMAIN: &[u8] = b"nuwax.mapping-cloud.georeference.v1\0";

/// A populated SHA-256 digest used by immutable map identities.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct Sha256Digest([u8; 32]);

impl Sha256Digest {
    /// Validate an externally supplied 32-byte digest.
    ///
    /// # Errors
    /// Returns [`DigestError::Unpopulated`] for the all-zero absent sentinel.
    pub fn from_bytes(bytes: [u8; 32]) -> Result<Self, DigestError> {
        if bytes == [0; 32] {
            return Err(DigestError::Unpopulated);
        }
        Ok(Self(bytes))
    }

    /// Parse an exact lowercase 64-character hexadecimal digest.
    ///
    /// # Errors
    /// Rejects wrong lengths, uppercase/non-hex characters, and the all-zero sentinel.
    pub fn from_lower_hex(value: &str) -> Result<Self, DigestError> {
        if value.len() != 64 {
            return Err(DigestError::InvalidLength(value.len()));
        }
        let mut bytes = [0_u8; 32];
        for (index, pair) in value.as_bytes().chunks_exact(2).enumerate() {
            let high = lower_hex_nibble(pair[0]).ok_or(DigestError::InvalidLowerHex)?;
            let low = lower_hex_nibble(pair[1]).ok_or(DigestError::InvalidLowerHex)?;
            bytes[index] = (high << 4) | low;
        }
        Self::from_bytes(bytes)
    }

    /// Compute a populated SHA-256 digest over exact bytes.
    #[must_use]
    pub fn of_bytes(bytes: &[u8]) -> Self {
        let digest = sha256_bytes(bytes);
        debug_assert_ne!(
            digest, [0; 32],
            "SHA-256 unexpectedly matched absent sentinel"
        );
        Self(digest)
    }

    /// Borrow the raw digest bytes.
    #[must_use]
    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }

    /// Return the raw digest bytes.
    #[must_use]
    pub const fn into_bytes(self) -> [u8; 32] {
        self.0
    }

    /// Encode the digest as exact lowercase hexadecimal.
    #[must_use]
    pub fn to_lower_hex(self) -> String {
        const HEX: &[u8; 16] = b"0123456789abcdef";
        let mut output = String::with_capacity(64);
        for byte in self.0 {
            output.push(char::from(HEX[usize::from(byte >> 4)]));
            output.push(char::from(HEX[usize::from(byte & 0x0f)]));
        }
        output
    }
}

impl fmt::Debug for Sha256Digest {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_tuple("Sha256Digest")
            .field(&self.to_lower_hex())
            .finish()
    }
}

impl fmt::Display for Sha256Digest {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.to_lower_hex())
    }
}

/// Validation failures for externally supplied SHA-256 identities.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum DigestError {
    /// The textual digest was not exactly 64 bytes.
    #[error("SHA-256 hex must contain exactly 64 bytes, got {0}")]
    InvalidLength(usize),
    /// The textual digest contained uppercase or non-hex characters.
    #[error("SHA-256 hex must contain lowercase ASCII hexadecimal only")]
    InvalidLowerHex,
    /// The all-zero absent sentinel was supplied as an identity.
    #[error("SHA-256 digest must be populated")]
    Unpopulated,
}

/// Raw-file binding carried by a finalized map asset bundle.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GeoreferenceBindingV1 {
    /// Binding schema version; exact value is one.
    pub schema_version: u32,
    /// Exact bundle-relative document path.
    pub relative_path: String,
    /// SHA-256 over the exact document bytes.
    pub raw_sha256: Sha256Digest,
}

impl GeoreferenceBindingV1 {
    /// Build the canonical v1 binding for exact document bytes.
    #[must_use]
    pub fn for_document(raw_json: &[u8]) -> Self {
        Self {
            schema_version: GEOREFERENCE_SCHEMA_VERSION,
            relative_path: GEOREFERENCE_CANONICAL_PATH.to_owned(),
            raw_sha256: Sha256Digest::of_bytes(raw_json),
        }
    }

    fn validate(&self) -> Result<(), GeoreferenceError> {
        if self.schema_version != GEOREFERENCE_SCHEMA_VERSION {
            return Err(GeoreferenceError::UnsupportedBindingSchema(
                self.schema_version,
            ));
        }
        if self.relative_path != GEOREFERENCE_CANONICAL_PATH {
            return Err(GeoreferenceError::InvalidBindingPath(
                self.relative_path.clone(),
            ));
        }
        Ok(())
    }
}

/// Immutable georeference-v1 document stored in a finalized map asset bundle.
///
/// Version one converts each of its eight bounded JSON number tokens directly
/// to binary64 with IEEE 754 `roundTiesToEven`. It never converts through an
/// intermediate decimal approximation. Rust is a consumer of the Python Phase
/// 2 publisher's sorted-key, compact UTF-8 JSON with one trailing newline;
/// derived [`Serialize`] output is round-trip-only and is not canonical asset
/// publication bytes. The raw digest always binds the exact supplied document.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct GeoreferenceV1 {
    /// Exact schema version; must be one.
    pub schema_version: u32,
    /// Finalized map name.
    pub map_name: String,
    /// Right-handed map frame with positive Z up.
    pub map_frame_id: String,
    /// Exact geographic source CRS.
    pub source_crs: String,
    /// Exact projected CRS.
    pub projected_crs: String,
    /// Exact projected-axis convention.
    pub axis_convention: String,
    /// WGS84 anchor latitude in degrees.
    pub anchor_lat_deg: f64,
    /// WGS84 anchor longitude in degrees.
    pub anchor_lng_deg: f64,
    /// EPSG:3857 easting of the WGS84 anchor in metres.
    pub anchor_easting_m: f64,
    /// EPSG:3857 northing of the WGS84 anchor in metres.
    pub anchor_northing_m: f64,
    /// Map-frame X coordinate corresponding to the anchor.
    pub map_origin_x_m: f64,
    /// Map-frame Y coordinate corresponding to the anchor.
    pub map_origin_y_m: f64,
    /// Counter-clockwise rotation from projected axes into the map frame.
    pub map_from_projected_yaw_rad: f64,
    /// Exact altitude policy; v1 supports only a constant map plane.
    pub altitude_policy: String,
    /// Fixed map-frame Z coordinate for every converted goal.
    pub map_plane_z_m: f64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawGeoreferenceV1 {
    schema_version: u32,
    map_name: String,
    map_frame_id: String,
    source_crs: String,
    projected_crs: String,
    axis_convention: String,
    anchor_lat_deg: Box<RawValue>,
    anchor_lng_deg: Box<RawValue>,
    anchor_easting_m: Box<RawValue>,
    anchor_northing_m: Box<RawValue>,
    map_origin_x_m: Box<RawValue>,
    map_origin_y_m: Box<RawValue>,
    map_from_projected_yaw_rad: Box<RawValue>,
    altitude_policy: String,
    map_plane_z_m: Box<RawValue>,
}

impl RawGeoreferenceV1 {
    fn into_contract(self) -> Result<GeoreferenceV1, String> {
        Ok(GeoreferenceV1 {
            schema_version: self.schema_version,
            map_name: self.map_name,
            map_frame_id: self.map_frame_id,
            source_crs: self.source_crs,
            projected_crs: self.projected_crs,
            axis_convention: self.axis_convention,
            anchor_lat_deg: parse_json_f64("anchor_lat_deg", &self.anchor_lat_deg)?,
            anchor_lng_deg: parse_json_f64("anchor_lng_deg", &self.anchor_lng_deg)?,
            anchor_easting_m: parse_json_f64("anchor_easting_m", &self.anchor_easting_m)?,
            anchor_northing_m: parse_json_f64("anchor_northing_m", &self.anchor_northing_m)?,
            map_origin_x_m: parse_json_f64("map_origin_x_m", &self.map_origin_x_m)?,
            map_origin_y_m: parse_json_f64("map_origin_y_m", &self.map_origin_y_m)?,
            map_from_projected_yaw_rad: parse_json_f64(
                "map_from_projected_yaw_rad",
                &self.map_from_projected_yaw_rad,
            )?,
            altitude_policy: self.altitude_policy,
            map_plane_z_m: parse_json_f64("map_plane_z_m", &self.map_plane_z_m)?,
        })
    }
}

impl<'de> Deserialize<'de> for GeoreferenceV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        RawGeoreferenceV1::deserialize(deserializer)?
            .into_contract()
            .map_err(serde::de::Error::custom)
    }
}

fn parse_json_f64(field: &'static str, raw: &RawValue) -> Result<f64, String> {
    let token = raw.get();
    if token.len() > MAX_JSON_NUMBER_TOKEN_BYTES {
        return Err(format!(
            "{field} JSON number token exceeds {MAX_JSON_NUMBER_TOKEN_BYTES} bytes"
        ));
    }
    token
        .parse::<f64>()
        .map_err(|_| format!("{field} must be a JSON number representable as f64"))
}

impl GeoreferenceV1 {
    /// Validate every field and cross-check the projected anchor.
    ///
    /// # Errors
    /// Returns a typed error for unsupported labels, invalid coordinates, a
    /// non-canonical yaw, malformed identity text, or anchor inconsistency.
    pub fn validate(&self) -> Result<(), GeoreferenceError> {
        if self.schema_version != GEOREFERENCE_SCHEMA_VERSION {
            return Err(GeoreferenceError::UnsupportedSchema(self.schema_version));
        }
        validate_map_name(&self.map_name)?;
        validate_frame_id(&self.map_frame_id)?;
        validate_exact_label("source_crs", &self.source_crs, GEOREFERENCE_SOURCE_CRS)?;
        validate_exact_label(
            "projected_crs",
            &self.projected_crs,
            GEOREFERENCE_PROJECTED_CRS,
        )?;
        validate_exact_label(
            "axis_convention",
            &self.axis_convention,
            GEOREFERENCE_AXIS_CONVENTION,
        )?;
        validate_exact_label(
            "altitude_policy",
            &self.altitude_policy,
            GEOREFERENCE_ALTITUDE_POLICY,
        )?;

        validate_latitude("anchor_lat_deg", self.anchor_lat_deg)?;
        validate_longitude("anchor_lng_deg", self.anchor_lng_deg)?;
        for (field, value) in [
            ("anchor_easting_m", self.anchor_easting_m),
            ("anchor_northing_m", self.anchor_northing_m),
            ("map_origin_x_m", self.map_origin_x_m),
            ("map_origin_y_m", self.map_origin_y_m),
            (
                "map_from_projected_yaw_rad",
                self.map_from_projected_yaw_rad,
            ),
            ("map_plane_z_m", self.map_plane_z_m),
        ] {
            validate_finite_canonical(field, value)?;
        }
        for (field, value) in [
            ("map_origin_x_m", self.map_origin_x_m),
            ("map_origin_y_m", self.map_origin_y_m),
            ("map_plane_z_m", self.map_plane_z_m),
        ] {
            validate_map_coordinate(field, value)?;
        }
        if !(-PI..PI).contains(&self.map_from_projected_yaw_rad) {
            return Err(GeoreferenceError::YawOutOfRange(
                self.map_from_projected_yaw_rad,
            ));
        }

        let (expected_easting, expected_northing) =
            wgs84_to_web_mercator(self.anchor_lat_deg, self.anchor_lng_deg);
        if (expected_easting - self.anchor_easting_m).abs() > ANCHOR_PROJECTION_TOLERANCE_M
            || (expected_northing - self.anchor_northing_m).abs() > ANCHOR_PROJECTION_TOLERANCE_M
        {
            return Err(GeoreferenceError::AnchorProjectionMismatch {
                expected_easting,
                expected_northing,
                supplied_easting: self.anchor_easting_m,
                supplied_northing: self.anchor_northing_m,
            });
        }
        Ok(())
    }

    /// Encode the validated contract as a deterministic binary preimage.
    ///
    /// Strings use a big-endian `u32` byte length. Integers and finite f64 bit
    /// patterns are big-endian; all floating zeroes are canonical positive zero.
    ///
    /// # Errors
    /// Returns the same errors as [`Self::validate`].
    pub fn canonical_preimage(&self) -> Result<Vec<u8>, GeoreferenceError> {
        self.validate()?;
        let mut output = Vec::with_capacity(320);
        output.extend_from_slice(GEOREFERENCE_DIGEST_DOMAIN);
        push_u32(&mut output, self.schema_version);
        push_text(&mut output, &self.map_name);
        push_text(&mut output, &self.map_frame_id);
        push_text(&mut output, &self.source_crs);
        push_text(&mut output, &self.projected_crs);
        push_text(&mut output, &self.axis_convention);
        push_f64(&mut output, self.anchor_lat_deg);
        push_f64(&mut output, self.anchor_lng_deg);
        push_f64(&mut output, self.anchor_easting_m);
        push_f64(&mut output, self.anchor_northing_m);
        push_f64(&mut output, self.map_origin_x_m);
        push_f64(&mut output, self.map_origin_y_m);
        push_f64(&mut output, self.map_from_projected_yaw_rad);
        push_text(&mut output, &self.altitude_policy);
        push_f64(&mut output, self.map_plane_z_m);
        Ok(output)
    }

    /// Compute the typed-contract digest over [`Self::canonical_preimage`].
    ///
    /// # Errors
    /// Returns the same errors as [`Self::validate`].
    pub fn canonical_sha256(&self) -> Result<Sha256Digest, GeoreferenceError> {
        Ok(Sha256Digest::of_bytes(&self.canonical_preimage()?))
    }

    fn project(&self, position: LatLng) -> Result<MapMetricPosition, GeoreferenceError> {
        self.validate()?;
        validate_latitude("goal_lat_deg", position.lat)?;
        validate_longitude("goal_lng_deg", position.lng)?;
        let (easting, northing) = wgs84_to_web_mercator(position.lat, position.lng);
        let delta_easting = easting - self.anchor_easting_m;
        let delta_northing = northing - self.anchor_northing_m;
        if !delta_easting.is_finite() || !delta_northing.is_finite() {
            return Err(GeoreferenceError::NonFinite("projected goal"));
        }
        let offset_m = delta_easting.hypot(delta_northing);
        if offset_m > MAX_LOCAL_GOAL_OFFSET_M {
            return Err(GeoreferenceError::GoalTooFar {
                offset_m,
                maximum_m: MAX_LOCAL_GOAL_OFFSET_M,
            });
        }
        let (sin_yaw, cos_yaw) = self.map_from_projected_yaw_rad.sin_cos();
        let x_m = self.map_origin_x_m + cos_yaw * delta_easting - sin_yaw * delta_northing;
        let y_m = self.map_origin_y_m + sin_yaw * delta_easting + cos_yaw * delta_northing;
        if !x_m.is_finite() || !y_m.is_finite() {
            return Err(GeoreferenceError::NonFinite("map goal"));
        }
        validate_map_coordinate("map goal x", x_m)?;
        validate_map_coordinate("map goal y", y_m)?;
        Ok(MapMetricPosition {
            x_m: canonical_zero(x_m),
            y_m: canonical_zero(y_m),
            z_m: self.map_plane_z_m,
        })
    }
}

/// A validated georeference bound to exact finalized-asset bytes.
#[derive(Debug, Clone, PartialEq)]
pub struct VerifiedGeoreferenceV1 {
    contract: GeoreferenceV1,
    raw_sha256: Sha256Digest,
    canonical_sha256: Sha256Digest,
}

impl VerifiedGeoreferenceV1 {
    /// Parse, hash, and validate an exact georeference JSON document.
    ///
    /// Serde's struct decoder rejects duplicate and unknown fields. Filesystem
    /// path and symlink containment remain the asset repository owner's boundary.
    ///
    /// # Errors
    /// Rejects an invalid binding, empty/oversized/malformed JSON, a raw digest
    /// mismatch, or any invalid georeference field.
    pub fn from_json(
        raw_json: &[u8],
        binding: &GeoreferenceBindingV1,
    ) -> Result<Self, GeoreferenceError> {
        binding.validate()?;
        if raw_json.is_empty() {
            return Err(GeoreferenceError::EmptyDocument);
        }
        if raw_json.len() > MAX_GEOREFERENCE_JSON_BYTES {
            return Err(GeoreferenceError::DocumentTooLarge(raw_json.len()));
        }
        let actual_sha256 = Sha256Digest::of_bytes(raw_json);
        if actual_sha256 != binding.raw_sha256 {
            return Err(GeoreferenceError::RawDigestMismatch {
                expected: binding.raw_sha256,
                actual: actual_sha256,
            });
        }
        let contract: GeoreferenceV1 = serde_json::from_slice(raw_json)
            .map_err(|error| GeoreferenceError::InvalidJson(error.to_string()))?;
        contract.validate()?;
        let canonical_sha256 = contract.canonical_sha256()?;
        Ok(Self {
            contract,
            raw_sha256: actual_sha256,
            canonical_sha256,
        })
    }

    /// Borrow the validated typed contract.
    #[must_use]
    pub const fn contract(&self) -> &GeoreferenceV1 {
        &self.contract
    }

    /// Return the digest over the exact raw JSON bytes.
    #[must_use]
    pub const fn raw_sha256(&self) -> Sha256Digest {
        self.raw_sha256
    }

    /// Return the digest over the deterministic typed-contract preimage.
    #[must_use]
    pub const fn canonical_sha256(&self) -> Sha256Digest {
        self.canonical_sha256
    }

    /// Convert a WGS84 goal to the validated local metric map frame.
    ///
    /// # Errors
    /// Rejects invalid WGS84 input, non-finite output, or a target beyond the
    /// bounded local-goal radius.
    pub fn project_wgs84(&self, position: LatLng) -> Result<MapMetricPosition, GeoreferenceError> {
        self.contract.project(position)
    }
}

/// A finite local metric point in the georeference's map frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MapMetricPosition {
    /// Map-frame X in metres.
    pub x_m: f64,
    /// Map-frame Y in metres.
    pub y_m: f64,
    /// Map-frame Z in metres.
    pub z_m: f64,
}

/// Fail-closed georeference validation or conversion errors.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum GeoreferenceError {
    /// The document schema version was not exactly one.
    #[error("unsupported georeference schema version {0}")]
    UnsupportedSchema(u32),
    /// The asset binding schema version was not exactly one.
    #[error("unsupported georeference binding schema version {0}")]
    UnsupportedBindingSchema(u32),
    /// The bundle-relative path was not the canonical v1 path.
    #[error("invalid georeference binding path {0:?}")]
    InvalidBindingPath(String),
    /// The map name was empty, non-canonical, or unbounded.
    #[error("map_name must be trimmed, control-free, and at most 255 UTF-8 bytes")]
    InvalidMapName,
    /// The map frame did not use bounded slash-separated ASCII tokens.
    #[error("map_frame_id must use bounded slash-separated ASCII frame tokens")]
    InvalidFrameId,
    /// A fixed v1 label did not match its exact required value.
    #[error("{field} must equal {expected:?}, got {actual:?}")]
    UnsupportedLabel {
        /// Field name.
        field: &'static str,
        /// Exact required value.
        expected: &'static str,
        /// Supplied value.
        actual: String,
    },
    /// A floating-point field was NaN or infinite.
    #[error("{0} must be finite")]
    NonFinite(&'static str),
    /// A floating-point field used the non-canonical negative zero encoding.
    #[error("{0} must encode zero as positive zero")]
    NegativeZero(&'static str),
    /// A latitude was outside the Web Mercator domain.
    #[error("{field} {value} is outside Web Mercator latitude range")]
    LatitudeOutOfRange {
        /// Field name.
        field: &'static str,
        /// Supplied value.
        value: f64,
    },
    /// A longitude was outside the WGS84 domain.
    #[error("{field} {value} is outside longitude range")]
    LongitudeOutOfRange {
        /// Field name.
        field: &'static str,
        /// Supplied value.
        value: f64,
    },
    /// The projected-to-map yaw was not normalized to `[-pi, pi)`.
    #[error("map_from_projected_yaw_rad {0} must be in [-pi, pi)")]
    YawOutOfRange(f64),
    /// Stored projected anchor coordinates did not match the WGS84 anchor.
    #[error("stored projected anchor does not match WGS84 anchor")]
    AnchorProjectionMismatch {
        /// Recomputed EPSG:3857 easting.
        expected_easting: f64,
        /// Recomputed EPSG:3857 northing.
        expected_northing: f64,
        /// Stored EPSG:3857 easting.
        supplied_easting: f64,
        /// Stored EPSG:3857 northing.
        supplied_northing: f64,
    },
    /// The target was too far from the local georeference anchor.
    #[error("projected goal offset {offset_m}m exceeds local maximum {maximum_m}m")]
    GoalTooFar {
        /// Computed projected offset.
        offset_m: f64,
        /// Frozen maximum offset.
        maximum_m: f64,
    },
    /// A map-frame coordinate exceeded the bounded v1 metric domain.
    #[error("{field} {value}m exceeds absolute map coordinate maximum {maximum_m}m")]
    MapCoordinateOutOfRange {
        /// Field name.
        field: &'static str,
        /// Supplied coordinate.
        value: f64,
        /// Frozen absolute maximum.
        maximum_m: f64,
    },
    /// No document bytes were supplied.
    #[error("georeference document must not be empty")]
    EmptyDocument,
    /// The document exceeded the bounded parser input.
    #[error("georeference document is too large: {0} bytes")]
    DocumentTooLarge(usize),
    /// The exact document bytes did not match the asset binding.
    #[error("georeference raw SHA-256 mismatch: expected {expected}, got {actual}")]
    RawDigestMismatch {
        /// Bound digest.
        expected: Sha256Digest,
        /// Recomputed digest.
        actual: Sha256Digest,
    },
    /// The JSON was malformed, duplicated a field, or carried an unknown field.
    #[error("invalid georeference JSON: {0}")]
    InvalidJson(String),
}

fn validate_map_name(value: &str) -> Result<(), GeoreferenceError> {
    let valid = !value.is_empty()
        && value.len() <= MAX_IDENTITY_BYTES
        && value.trim() == value
        && !value.chars().any(char::is_control);
    if valid {
        Ok(())
    } else {
        Err(GeoreferenceError::InvalidMapName)
    }
}

pub(crate) fn valid_frame_id(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= MAX_IDENTITY_BYTES
        && value.trim() == value
        && value.split('/').all(|segment| {
            let mut characters = segment.chars();
            characters
                .next()
                .is_some_and(|first| first.is_ascii_alphabetic())
                && characters.all(|character| character.is_ascii_alphanumeric() || character == '_')
        })
}

fn validate_frame_id(value: &str) -> Result<(), GeoreferenceError> {
    if valid_frame_id(value) {
        Ok(())
    } else {
        Err(GeoreferenceError::InvalidFrameId)
    }
}

fn validate_exact_label(
    field: &'static str,
    actual: &str,
    expected: &'static str,
) -> Result<(), GeoreferenceError> {
    if actual == expected {
        Ok(())
    } else {
        Err(GeoreferenceError::UnsupportedLabel {
            field,
            expected,
            actual: actual.to_owned(),
        })
    }
}

fn validate_latitude(field: &'static str, value: f64) -> Result<(), GeoreferenceError> {
    validate_finite_canonical(field, value)?;
    if (-WEB_MERCATOR_MAX_LATITUDE_DEG..=WEB_MERCATOR_MAX_LATITUDE_DEG).contains(&value) {
        Ok(())
    } else {
        Err(GeoreferenceError::LatitudeOutOfRange { field, value })
    }
}

fn validate_longitude(field: &'static str, value: f64) -> Result<(), GeoreferenceError> {
    validate_finite_canonical(field, value)?;
    if (-180.0..=180.0).contains(&value) {
        Ok(())
    } else {
        Err(GeoreferenceError::LongitudeOutOfRange { field, value })
    }
}

fn validate_finite_canonical(field: &'static str, value: f64) -> Result<(), GeoreferenceError> {
    if !value.is_finite() {
        return Err(GeoreferenceError::NonFinite(field));
    }
    if value == 0.0 && value.is_sign_negative() {
        return Err(GeoreferenceError::NegativeZero(field));
    }
    Ok(())
}

fn validate_map_coordinate(field: &'static str, value: f64) -> Result<(), GeoreferenceError> {
    if value.abs() <= MAX_ABSOLUTE_MAP_COORDINATE_M {
        Ok(())
    } else {
        Err(GeoreferenceError::MapCoordinateOutOfRange {
            field,
            value,
            maximum_m: MAX_ABSOLUTE_MAP_COORDINATE_M,
        })
    }
}

pub(crate) fn canonical_zero(value: f64) -> f64 {
    if value == 0.0 {
        0.0
    } else {
        value
    }
}

pub(crate) fn push_u32(output: &mut Vec<u8>, value: u32) {
    output.extend_from_slice(&value.to_be_bytes());
}

pub(crate) fn push_u64(output: &mut Vec<u8>, value: u64) {
    output.extend_from_slice(&value.to_be_bytes());
}

pub(crate) fn push_i64(output: &mut Vec<u8>, value: i64) {
    output.extend_from_slice(&value.to_be_bytes());
}

pub(crate) fn push_f64(output: &mut Vec<u8>, value: f64) {
    output.extend_from_slice(&canonical_zero(value).to_bits().to_be_bytes());
}

pub(crate) fn push_text(output: &mut Vec<u8>, value: &str) {
    let length = u32::try_from(value.len()).expect("validated identity text must fit u32");
    push_u32(output, length);
    output.extend_from_slice(value.as_bytes());
}

pub(crate) fn push_digest(output: &mut Vec<u8>, value: Sha256Digest) {
    output.extend_from_slice(value.as_bytes());
}

fn lower_hex_nibble(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        _ => None,
    }
}

pub(crate) fn sha256_bytes(data: &[u8]) -> [u8; 32] {
    const H_INIT: [u32; 8] = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
        0x5be0cd19,
    ];
    const K: [u32; 64] = [
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
        0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
        0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
        0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
        0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
        0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
        0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
        0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
        0xc67178f2,
    ];

    let bit_len = (data.len() as u64).wrapping_mul(8);
    let mut padded = data.to_vec();
    padded.push(0x80);
    while padded.len() % 64 != 56 {
        padded.push(0);
    }
    padded.extend_from_slice(&bit_len.to_be_bytes());

    let mut h = H_INIT;
    for chunk in padded.chunks_exact(64) {
        let mut words = [0_u32; 64];
        for (index, word) in words.iter_mut().take(16).enumerate() {
            let offset = index * 4;
            *word = u32::from_be_bytes(
                chunk[offset..offset + 4]
                    .try_into()
                    .expect("four-byte SHA-256 word"),
            );
        }
        for index in 16..64 {
            let sigma0 = words[index - 15].rotate_right(7)
                ^ words[index - 15].rotate_right(18)
                ^ (words[index - 15] >> 3);
            let sigma1 = words[index - 2].rotate_right(17)
                ^ words[index - 2].rotate_right(19)
                ^ (words[index - 2] >> 10);
            words[index] = words[index - 16]
                .wrapping_add(sigma0)
                .wrapping_add(words[index - 7])
                .wrapping_add(sigma1);
        }

        let [mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut hh] = h;
        for (word, constant) in words.iter().zip(K) {
            let sigma1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let choose = (e & f) ^ ((!e) & g);
            let first = hh
                .wrapping_add(sigma1)
                .wrapping_add(choose)
                .wrapping_add(constant)
                .wrapping_add(*word);
            let sigma0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let majority = (a & b) ^ (a & c) ^ (b & c);
            let second = sigma0.wrapping_add(majority);
            hh = g;
            g = f;
            f = e;
            e = d.wrapping_add(first);
            d = c;
            c = b;
            b = a;
            a = first.wrapping_add(second);
        }
        for (target, value) in h.iter_mut().zip([a, b, c, d, e, f, g, hh]) {
            *target = target.wrapping_add(value);
        }
    }

    let mut digest = [0_u8; 32];
    for (index, value) in h.iter().enumerate() {
        digest[index * 4..index * 4 + 4].copy_from_slice(&value.to_be_bytes());
    }
    digest
}

#[cfg(test)]
mod tests {
    use std::f64::consts::FRAC_PI_2;

    use super::*;

    fn valid_contract() -> GeoreferenceV1 {
        let anchor_lat_deg = 31.230_4;
        let anchor_lng_deg = 121.473_7;
        GeoreferenceV1 {
            schema_version: GEOREFERENCE_SCHEMA_VERSION,
            map_name: "shanghai_lab".into(),
            map_frame_id: "map/cloud".into(),
            source_crs: GEOREFERENCE_SOURCE_CRS.into(),
            projected_crs: GEOREFERENCE_PROJECTED_CRS.into(),
            axis_convention: GEOREFERENCE_AXIS_CONVENTION.into(),
            anchor_lat_deg,
            anchor_lng_deg,
            anchor_easting_m: 13_522_390.428_774_875,
            anchor_northing_m: 3_662_707.257_050_403,
            map_origin_x_m: 10.0,
            map_origin_y_m: -4.0,
            map_from_projected_yaw_rad: FRAC_PI_2,
            altitude_policy: GEOREFERENCE_ALTITUDE_POLICY.into(),
            map_plane_z_m: 1.25,
        }
    }

    fn verified(contract: &GeoreferenceV1) -> VerifiedGeoreferenceV1 {
        let raw = serde_json::to_vec(contract).expect("serialize fixture");
        VerifiedGeoreferenceV1::from_json(&raw, &GeoreferenceBindingV1::for_document(&raw))
            .expect("verify fixture")
    }

    const FLOAT_TOKEN_CASES: [(&str, &str, &str, &str, u64); 8] = [
        (
            "anchor_lat_deg",
            "31.2304",
            "3.12304e1",
            "31.230400000000003",
            0x403f_3afb_7e90_ff98,
        ),
        (
            "anchor_lng_deg",
            "121.4737",
            "1.214737e2",
            "121.47370000000001",
            0x405e_5e51_19ce_0760,
        ),
        (
            "anchor_easting_m",
            "13522390.428774875",
            "1.3522390428774875e7",
            "13522390.428774877",
            0x4169_caba_cdb8_8617,
        ),
        (
            "anchor_northing_m",
            "3662707.257050403",
            "3.662707257050403e6",
            "3662707.2570504034",
            0x414b_f1b9_a0e7_0712,
        ),
        (
            "map_origin_x_m",
            "10.0",
            "1e1",
            "10.000000000000002",
            0x4024_0000_0000_0001,
        ),
        (
            "map_origin_y_m",
            "-4.0",
            "-4e0",
            "-3.9999999999999996",
            0xc00f_ffff_ffff_ffff,
        ),
        (
            "map_from_projected_yaw_rad",
            "1.5707963267948966",
            "1.5707963267948966e0",
            "1.5707963267948968",
            0x3ff9_21fb_5444_2d19,
        ),
        (
            "map_plane_z_m",
            "1.25",
            "1.25e0",
            "1.2500000000000002",
            0x3ff4_0000_0000_0001,
        ),
    ];

    fn python_contract_json() -> Vec<u8> {
        br#"{"altitude_policy":"map_plane_constant","anchor_easting_m":13522390.428774875,"anchor_lat_deg":31.2304,"anchor_lng_deg":121.4737,"anchor_northing_m":3662707.257050403,"axis_convention":"projected_x_east_y_north_z_up","map_frame_id":"map/cloud","map_from_projected_yaw_rad":1.5707963267948966,"map_name":"shanghai_lab","map_origin_x_m":10.0,"map_origin_y_m":-4.0,"map_plane_z_m":1.25,"projected_crs":"EPSG:3857","schema_version":1,"source_crs":"EPSG:4326"}
"#
        .to_vec()
    }

    fn replace_numeric_token(raw: &[u8], field: &str, old: &str, new: &str) -> Vec<u8> {
        let raw = std::str::from_utf8(raw).expect("ASCII JSON fixture");
        let needle = format!("\"{field}\":{old}");
        assert_eq!(raw.matches(&needle).count(), 1, "fixture field {field}");
        raw.replacen(&needle, &format!("\"{field}\":{new}"), 1)
            .into_bytes()
    }

    fn verify_raw(raw: &[u8]) -> Result<VerifiedGeoreferenceV1, GeoreferenceError> {
        VerifiedGeoreferenceV1::from_json(raw, &GeoreferenceBindingV1::for_document(raw))
    }

    fn numeric_field(contract: &GeoreferenceV1, field: &str) -> f64 {
        match field {
            "anchor_lat_deg" => contract.anchor_lat_deg,
            "anchor_lng_deg" => contract.anchor_lng_deg,
            "anchor_easting_m" => contract.anchor_easting_m,
            "anchor_northing_m" => contract.anchor_northing_m,
            "map_origin_x_m" => contract.map_origin_x_m,
            "map_origin_y_m" => contract.map_origin_y_m,
            "map_from_projected_yaw_rad" => contract.map_from_projected_yaw_rad,
            "map_plane_z_m" => contract.map_plane_z_m,
            _ => panic!("unexpected numeric field {field}"),
        }
    }

    #[test]
    fn json_number_tokens_follow_round_ties_to_even_matrix() {
        let cases = [
            (
                "13522390.428774873",
                0x4169_caba_cdb8_8615,
                "cb4487fe8869c84ded63e8d646a3d37bbe83cce619fe00620d71a67866a4a784",
                "f11b8ce353b093757095dd838f6e1f57689f6d4dcf9ebb9371b579a7e16ddca7",
            ),
            (
                "13522390.428774875",
                0x4169_caba_cdb8_8616,
                "05ed282beb2685b5f231b6a3c5566c635693bc0fc1cc93f17a76dabe73fbda53",
                "30afa770eef0100e3e22beffbd4624cfb5f033f6bb9c86bd9f779d7efc4cf4e8",
            ),
            (
                "13522390.4287748750",
                0x4169_caba_cdb8_8616,
                "b5eeb9dea5d91210669dc2884d861ef689b6b981bc9150392eda218693e4b935",
                "30afa770eef0100e3e22beffbd4624cfb5f033f6bb9c86bd9f779d7efc4cf4e8",
            ),
            (
                "1.3522390428774875e7",
                0x4169_caba_cdb8_8616,
                "1c38964694d951a7b65cb5df653ad64ccad102e3a59cb6f22bf5396f454d7596",
                "30afa770eef0100e3e22beffbd4624cfb5f033f6bb9c86bd9f779d7efc4cf4e8",
            ),
            (
                "13522390.428774877",
                0x4169_caba_cdb8_8617,
                "164b6d32dea36fafb1154fb9ee1c7cdf0a43bf754ccabea578f5f27b0b4c6ba4",
                "01f2871852564422ff24d02175813e163fc32d5d037b4ac1b911053167abbfc2",
            ),
        ];

        for (token, bits, raw_sha256, canonical_sha256) in cases {
            let raw = replace_numeric_token(
                &python_contract_json(),
                "anchor_easting_m",
                "13522390.428774875",
                token,
            );
            let verified = verify_raw(&raw).expect("verify correctly rounded token");
            assert_eq!(verified.contract().anchor_easting_m.to_bits(), bits);
            assert_eq!(verified.raw_sha256().to_lower_hex(), raw_sha256);
            assert_eq!(verified.canonical_sha256().to_lower_hex(), canonical_sha256);
        }
    }

    #[test]
    fn all_float_fields_accept_exponents_adjacent_ulps_and_long_tokens() {
        for (field, original, exponent, adjacent, adjacent_bits) in FLOAT_TOKEN_CASES {
            let exponent_raw =
                replace_numeric_token(&python_contract_json(), field, original, exponent);
            let exponent_contract = verify_raw(&exponent_raw).expect("verify exponent token");
            assert_eq!(
                numeric_field(exponent_contract.contract(), field).to_bits(),
                numeric_field(&valid_contract(), field).to_bits(),
                "exponent field {field}"
            );

            let adjacent_raw =
                replace_numeric_token(&python_contract_json(), field, original, adjacent);
            let adjacent_contract = verify_raw(&adjacent_raw).expect("verify adjacent ULP token");
            assert_eq!(
                numeric_field(adjacent_contract.contract(), field).to_bits(),
                adjacent_bits,
                "adjacent ULP field {field}"
            );

            let long_token = format!(
                "{original}{}",
                "0".repeat(MAX_JSON_NUMBER_TOKEN_BYTES - original.len())
            );
            assert_eq!(long_token.len(), MAX_JSON_NUMBER_TOKEN_BYTES);
            let long_raw =
                replace_numeric_token(&python_contract_json(), field, original, &long_token);
            let long_contract = verify_raw(&long_raw).expect("verify long decimal token");
            assert_eq!(
                numeric_field(long_contract.contract(), field).to_bits(),
                numeric_field(&valid_contract(), field).to_bits(),
                "long token field {field}"
            );
        }
    }

    #[test]
    fn all_float_fields_reject_negative_zero_invalid_types_and_duplicates() {
        for (field, original, _, _, _) in FLOAT_TOKEN_CASES {
            for token in ["-0.0", "-0"] {
                let negative_zero =
                    replace_numeric_token(&python_contract_json(), field, original, token);
                assert!(matches!(
                    verify_raw(&negative_zero),
                    Err(GeoreferenceError::NegativeZero(actual)) if actual == field
                ));
            }

            let invalid_type =
                replace_numeric_token(&python_contract_json(), field, original, "\"0\"");
            assert!(matches!(
                verify_raw(&invalid_type),
                Err(GeoreferenceError::InvalidJson(_))
            ));

            let duplicate = replace_numeric_token(
                &python_contract_json(),
                field,
                original,
                &format!("{original},\"{field}\":{original}"),
            );
            assert!(matches!(
                verify_raw(&duplicate),
                Err(GeoreferenceError::InvalidJson(_))
            ));
        }
    }

    #[test]
    fn float_tokens_reject_invalid_grammar_and_oversized_document() {
        for invalid in ["01", "+1", ".1", "1.", "1e", "NaN", "Infinity", "--1"] {
            let raw = replace_numeric_token(
                &python_contract_json(),
                "anchor_easting_m",
                "13522390.428774875",
                invalid,
            );
            assert!(matches!(
                verify_raw(&raw),
                Err(GeoreferenceError::InvalidJson(_))
            ));
        }

        let overlong_token = format!("10.{}", "0".repeat(126));
        assert_eq!(overlong_token.len(), MAX_JSON_NUMBER_TOKEN_BYTES + 1);
        let overlong = replace_numeric_token(
            &python_contract_json(),
            "map_origin_x_m",
            "10.0",
            &overlong_token,
        );
        assert!(matches!(
            verify_raw(&overlong),
            Err(GeoreferenceError::InvalidJson(_))
        ));

        let oversized_token = format!("1.{}1", "0".repeat(MAX_GEOREFERENCE_JSON_BYTES));
        let oversized = replace_numeric_token(
            &python_contract_json(),
            "map_origin_x_m",
            "10.0",
            &oversized_token,
        );
        assert!(matches!(
            verify_raw(&oversized),
            Err(GeoreferenceError::DocumentTooLarge(_))
        ));
    }

    #[test]
    fn number_token_extremes_match_python_phase2_acceptance() {
        let subnormal =
            replace_numeric_token(&python_contract_json(), "map_origin_x_m", "10.0", "5e-324");
        let verified = verify_raw(&subnormal).expect("minimum subnormal remains finite");
        assert_eq!(verified.contract().map_origin_x_m.to_bits(), 1);

        let maximum_finite = replace_numeric_token(
            &python_contract_json(),
            "map_origin_x_m",
            "10.0",
            "1.7976931348623157e308",
        );
        assert!(matches!(
            verify_raw(&maximum_finite),
            Err(GeoreferenceError::MapCoordinateOutOfRange {
                field: "map_origin_x_m",
                ..
            })
        ));

        let overflow =
            replace_numeric_token(&python_contract_json(), "map_origin_x_m", "10.0", "1e309");
        assert!(matches!(
            verify_raw(&overflow),
            Err(GeoreferenceError::NonFinite("map_origin_x_m"))
        ));
    }

    #[test]
    fn anchor_tolerance_matches_python_phase2_boundary_oracle() {
        let mut accepted = valid_contract();
        accepted.anchor_easting_m = 13_522_390.428_775_873;
        accepted
            .validate()
            .expect("Python accepts the upper in-tolerance binary64 value");

        let mut rejected = valid_contract();
        rejected.anchor_easting_m = 13_522_390.428_773_874;
        assert!(matches!(
            rejected.validate(),
            Err(GeoreferenceError::AnchorProjectionMismatch { .. })
        ));
    }

    #[test]
    fn serialization_uses_shortest_roundtrip_tokens_for_all_float_fields() {
        let mut exponent_raw = python_contract_json();
        for (field, original, exponent, _, _) in FLOAT_TOKEN_CASES {
            exponent_raw = replace_numeric_token(&exponent_raw, field, original, exponent);
        }
        let verified = verify_raw(&exponent_raw).expect("verify exponent document");
        let roundtrip_only = serde_json::to_vec(verified.contract()).expect("serialize contract");
        for (field, shortest, _, _, _) in FLOAT_TOKEN_CASES {
            let token = format!("\"{field}\":{shortest}");
            assert!(
                std::str::from_utf8(&roundtrip_only)
                    .expect("serialized JSON is UTF-8")
                    .contains(&token),
                "shortest token field {field}"
            );
        }
        assert!(!roundtrip_only.ends_with(b"\n"));
        assert_ne!(roundtrip_only, python_contract_json());
        let reparsed = verify_raw(&roundtrip_only).expect("reparse shortest-roundtrip document");
        assert_eq!(reparsed.contract(), verified.contract());
    }

    #[test]
    fn python_publisher_bytes_bind_unicode_and_key_order_without_recanonicalizing() {
        let canonical = String::from_utf8(python_contract_json())
            .expect("Python publisher fixture is UTF-8")
            .replace("shanghai_lab", "实验室🧭")
            .into_bytes();
        let verified = verify_raw(&canonical).expect("consume Python canonical JSON");
        assert_eq!(verified.contract().map_name, "实验室🧭");
        assert_eq!(
            verified.raw_sha256().to_lower_hex(),
            "f71ce016c61b7427cde577aa0ae10ecb1fe0579a745cb9802e90ba5d2b9c91af"
        );
        assert!(canonical.ends_with(b"\n"));

        let rust_roundtrip =
            serde_json::to_vec(verified.contract()).expect("serialize consumer DTO");
        assert_ne!(rust_roundtrip, canonical);
        let reordered = verify_raw(&rust_roundtrip).expect("key order is not semantic identity");
        assert_eq!(reordered.contract(), verified.contract());
        assert_eq!(reordered.canonical_sha256(), verified.canonical_sha256());
        assert_ne!(reordered.raw_sha256(), verified.raw_sha256());
    }

    #[test]
    fn sha256_and_strict_hex_match_public_vectors() {
        assert_eq!(
            Sha256Digest::of_bytes(b"").to_lower_hex(),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        );
        assert_eq!(
            Sha256Digest::of_bytes(b"abc").to_lower_hex(),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
        assert_eq!(
            Sha256Digest::from_lower_hex(
                "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
            )
            .expect("valid lowercase digest"),
            Sha256Digest::of_bytes(b"abc")
        );
        assert_eq!(
            Sha256Digest::from_lower_hex("aa"),
            Err(DigestError::InvalidLength(2))
        );
        assert_eq!(
            Sha256Digest::from_lower_hex(
                "BA7816BF8F01CFEA414140DE5DAE2223B00361A396177A9CB410FF61F20015AD"
            ),
            Err(DigestError::InvalidLowerHex)
        );
        assert_eq!(
            Sha256Digest::from_lower_hex(&"0".repeat(64)),
            Err(DigestError::Unpopulated)
        );
    }

    #[test]
    fn verified_contract_projects_with_ccw_rotation_and_constant_plane() {
        let contract = valid_contract();
        let verified = verified(&contract);
        let east_one_metre_lng = contract.anchor_lng_deg + 180.0 / 20_037_508.342_789_244;
        let point = verified
            .project_wgs84(LatLng {
                lat: contract.anchor_lat_deg,
                lng: east_one_metre_lng,
            })
            .expect("project local target");
        assert!((point.x_m - 10.0).abs() < 1.0e-8);
        assert!((point.y_m - -3.0).abs() < 1.0e-6);
        assert_eq!(point.z_m, 1.25);
        assert_eq!(verified.contract().map_name, contract.map_name);
        assert_eq!(verified.contract().map_frame_id, contract.map_frame_id);
        assert!(
            (verified.contract().anchor_northing_m - contract.anchor_northing_m).abs()
                < ANCHOR_PROJECTION_TOLERANCE_M
        );
        assert_ne!(verified.raw_sha256(), verified.canonical_sha256());
    }

    #[test]
    fn north_south_and_rotated_north_match_proj_oracles() {
        const NORTH_DELTA_M: f64 = 130.185_328_940;
        const SOUTH_DELTA_M: f64 = -130.183_951_231;

        let mut unrotated = valid_contract();
        unrotated.map_from_projected_yaw_rad = 0.0;
        let unrotated_verified = verified(&unrotated);
        let north = unrotated_verified
            .project_wgs84(LatLng {
                lat: unrotated.anchor_lat_deg + 0.001,
                lng: unrotated.anchor_lng_deg,
            })
            .expect("project north");
        assert!((north.x_m - unrotated.map_origin_x_m).abs() < 1e-6);
        assert!(
            (north.y_m - (unrotated.map_origin_y_m + NORTH_DELTA_M)).abs() < 1e-5,
            "north y={}",
            north.y_m
        );

        let south = unrotated_verified
            .project_wgs84(LatLng {
                lat: unrotated.anchor_lat_deg - 0.001,
                lng: unrotated.anchor_lng_deg,
            })
            .expect("project south");
        assert!((south.x_m - unrotated.map_origin_x_m).abs() < 1e-6);
        assert!(
            (south.y_m - (unrotated.map_origin_y_m + SOUTH_DELTA_M)).abs() < 1e-5,
            "south y={}",
            south.y_m
        );

        let rotated = valid_contract();
        let rotated_verified = verified(&rotated);
        let north = rotated_verified
            .project_wgs84(LatLng {
                lat: rotated.anchor_lat_deg + 0.001,
                lng: rotated.anchor_lng_deg,
            })
            .expect("project rotated north");
        assert!(
            (north.x_m - (rotated.map_origin_x_m - NORTH_DELTA_M)).abs() < 1e-5,
            "rotated north x={}",
            north.x_m
        );
        assert!((north.y_m - rotated.map_origin_y_m).abs() < 1e-6);
    }

    #[test]
    fn canonical_preimage_and_digest_are_deterministic() {
        let contract = valid_contract();
        let first = contract.canonical_preimage().expect("valid preimage");
        let second = contract.canonical_preimage().expect("stable preimage");
        assert_eq!(first, second);
        assert!(first.starts_with(GEOREFERENCE_DIGEST_DOMAIN));
        assert_eq!(first.len(), 214);
        assert_eq!(
            Sha256Digest::of_bytes(&first).to_lower_hex(),
            "30afa770eef0100e3e22beffbd4624cfb5f033f6bb9c86bd9f779d7efc4cf4e8"
        );
        assert_eq!(
            contract.canonical_sha256().expect("canonical digest"),
            Sha256Digest::of_bytes(&first)
        );
    }

    #[test]
    fn rejects_wrong_schema_labels_frame_and_map_name() {
        let mut cases = Vec::new();
        let mut value = valid_contract();
        value.schema_version = 2;
        cases.push(value);
        let mut value = valid_contract();
        value.source_crs = "epsg:4326".into();
        cases.push(value);
        let mut value = valid_contract();
        value.projected_crs = "EPSG:3395".into();
        cases.push(value);
        let mut value = valid_contract();
        value.axis_convention = "north_east_up".into();
        cases.push(value);
        let mut value = valid_contract();
        value.altitude_policy = "wgs84".into();
        cases.push(value);
        let mut value = valid_contract();
        value.map_frame_id = "/map".into();
        cases.push(value);
        let mut value = valid_contract();
        value.map_name = " map".into();
        cases.push(value);
        for value in cases {
            assert!(value.validate().is_err());
        }
    }

    #[test]
    fn rejects_nonfinite_negative_zero_ranges_yaw_and_anchor_mismatch() {
        let mut value = valid_contract();
        value.map_origin_x_m = f64::NAN;
        assert!(matches!(
            value.validate(),
            Err(GeoreferenceError::NonFinite("map_origin_x_m"))
        ));

        let mut value = valid_contract();
        value.map_plane_z_m = -0.0;
        assert!(matches!(
            value.validate(),
            Err(GeoreferenceError::NegativeZero("map_plane_z_m"))
        ));

        let mut value = valid_contract();
        value.anchor_lat_deg = WEB_MERCATOR_MAX_LATITUDE_DEG + 0.001;
        assert!(matches!(
            value.validate(),
            Err(GeoreferenceError::LatitudeOutOfRange { .. })
        ));

        let mut value = valid_contract();
        value.anchor_lng_deg = 180.001;
        assert!(matches!(
            value.validate(),
            Err(GeoreferenceError::LongitudeOutOfRange { .. })
        ));

        let mut value = valid_contract();
        value.map_from_projected_yaw_rad = PI;
        assert!(matches!(
            value.validate(),
            Err(GeoreferenceError::YawOutOfRange(_))
        ));

        let mut value = valid_contract();
        value.anchor_easting_m += 0.01;
        assert!(matches!(
            value.validate(),
            Err(GeoreferenceError::AnchorProjectionMismatch { .. })
        ));

        let mut value = valid_contract();
        value.map_origin_y_m = MAX_ABSOLUTE_MAP_COORDINATE_M + 1.0;
        assert!(matches!(
            value.validate(),
            Err(GeoreferenceError::MapCoordinateOutOfRange {
                field: "map_origin_y_m",
                ..
            })
        ));
    }

    #[test]
    fn raw_binding_rejects_path_schema_digest_and_bounded_json_failures() {
        let contract = valid_contract();
        let raw = serde_json::to_vec(&contract).expect("serialize fixture");
        let binding = GeoreferenceBindingV1::for_document(&raw);

        let mut wrong_path = binding.clone();
        wrong_path.relative_path = "../georeference.json".into();
        assert!(matches!(
            VerifiedGeoreferenceV1::from_json(&raw, &wrong_path),
            Err(GeoreferenceError::InvalidBindingPath(_))
        ));

        let mut wrong_schema = binding.clone();
        wrong_schema.schema_version = 2;
        assert!(matches!(
            VerifiedGeoreferenceV1::from_json(&raw, &wrong_schema),
            Err(GeoreferenceError::UnsupportedBindingSchema(2))
        ));

        let tampered = [raw.as_slice(), b" "].concat();
        assert!(matches!(
            VerifiedGeoreferenceV1::from_json(&tampered, &binding),
            Err(GeoreferenceError::RawDigestMismatch { .. })
        ));
        assert_eq!(
            VerifiedGeoreferenceV1::from_json(&[], &GeoreferenceBindingV1::for_document(&[])),
            Err(GeoreferenceError::EmptyDocument)
        );
        let oversized = vec![b' '; MAX_GEOREFERENCE_JSON_BYTES + 1];
        assert!(matches!(
            VerifiedGeoreferenceV1::from_json(
                &oversized,
                &GeoreferenceBindingV1::for_document(&oversized)
            ),
            Err(GeoreferenceError::DocumentTooLarge(_))
        ));
    }

    #[test]
    fn json_rejects_duplicate_and_unknown_fields() {
        let raw = br#"{
            "schema_version":1,
            "schema_version":1,
            "map_name":"m",
            "map_frame_id":"map",
            "source_crs":"EPSG:4326",
            "projected_crs":"EPSG:3857",
            "axis_convention":"projected_x_east_y_north_z_up",
            "anchor_lat_deg":0.0,
            "anchor_lng_deg":0.0,
            "anchor_easting_m":0.0,
            "anchor_northing_m":0.0,
            "map_origin_x_m":0.0,
            "map_origin_y_m":0.0,
            "map_from_projected_yaw_rad":0.0,
            "altitude_policy":"map_plane_constant",
            "map_plane_z_m":0.0
        }"#;
        assert!(matches!(
            VerifiedGeoreferenceV1::from_json(raw, &GeoreferenceBindingV1::for_document(raw)),
            Err(GeoreferenceError::InvalidJson(_))
        ));

        let mut object = serde_json::to_value(valid_contract())
            .expect("serialize fixture")
            .as_object()
            .expect("object fixture")
            .clone();
        object.insert("unknown".into(), serde_json::json!(true));
        let raw = serde_json::to_vec(&object).expect("serialize unknown field fixture");
        assert!(matches!(
            VerifiedGeoreferenceV1::from_json(&raw, &GeoreferenceBindingV1::for_document(&raw)),
            Err(GeoreferenceError::InvalidJson(_))
        ));
    }

    #[test]
    fn projection_rejects_nonfinite_out_of_domain_far_and_antimeridian_goals() {
        let verified_reference = verified(&valid_contract());
        for position in [
            LatLng {
                lat: f64::NAN,
                lng: 0.0,
            },
            LatLng {
                lat: WEB_MERCATOR_MAX_LATITUDE_DEG + 0.001,
                lng: 0.0,
            },
            LatLng {
                lat: 0.0,
                lng: 180.001,
            },
        ] {
            assert!(verified_reference.project_wgs84(position).is_err());
        }
        assert!(matches!(
            verified_reference.project_wgs84(LatLng {
                lat: valid_contract().anchor_lat_deg + 2.0,
                lng: valid_contract().anchor_lng_deg,
            }),
            Err(GeoreferenceError::GoalTooFar { .. })
        ));

        let mut antimeridian = valid_contract();
        antimeridian.anchor_lat_deg = 0.0;
        antimeridian.anchor_lng_deg = 179.9;
        (
            antimeridian.anchor_easting_m,
            antimeridian.anchor_northing_m,
        ) = wgs84_to_web_mercator(antimeridian.anchor_lat_deg, antimeridian.anchor_lng_deg);
        let verified = verified(&antimeridian);
        assert!(matches!(
            verified.project_wgs84(LatLng {
                lat: 0.0,
                lng: -179.9,
            }),
            Err(GeoreferenceError::GoalTooFar { .. })
        ));
    }
}
