//! Google Maps HTTP client.
//!
//! Network access is explicit. [`CloudMode::Live`] performs an HTTPS request
//! using `GOOGLE_MAPS_API_KEY` (or the legacy
//! `NUWAX_GOOGLE_MAPS_API_KEY`). [`CloudMode::Offline`] and the legacy
//! [`CloudMode::Mock`] never fabricate a location and return
//! [`CloudError::Offline`].

use serde::{de::DeserializeOwned, Deserialize, Serialize};
use thiserror::Error;

const GOOGLE_MAPS_BASE_URL: &str = "https://maps.googleapis.com/maps/api";

/// Common cloud-service error.
#[derive(Debug, Error)]
pub enum CloudError {
    /// A Google Maps live request was selected without an API key.
    #[error("missing API key (set GOOGLE_MAPS_API_KEY)")]
    ApiKeyMissing,
    /// The caller selected an explicit no-network mode.
    #[error("operation is unavailable in offline mode")]
    Offline,
    /// The HTTP client could not complete a request.
    #[error("HTTP error: {0}")]
    Http(String),
    /// The remote server returned a non-success status.
    #[error("HTTP status {status}: {message}")]
    HttpStatus {
        /// Numeric HTTP status code.
        status: u16,
        /// Response body or API error text.
        message: String,
    },
    /// A response or image could not be decoded or validated.
    #[error("parse error: {0}")]
    Parse(String),
    /// The requested cloud feature was compiled out.
    #[error("cloud map disabled (compile with --features cloud-enabled)")]
    Disabled,
}

/// Cloud call mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CloudMode {
    /// Do not perform network requests.
    Offline,
    /// Legacy spelling for [`CloudMode::Offline`].
    Mock,
    /// Perform a real HTTP request.
    Live,
}

/// Latitude and longitude in WGS84 degrees.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LatLng {
    /// Latitude in `[-90, 90]`.
    pub lat: f64,
    /// Longitude in `[-180, 180]`.
    pub lng: f64,
}

/// Selected address components from a geocoding response.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct GeocodeComponents {
    /// Street number and route.
    pub street: String,
    /// Locality or administrative area.
    pub city: String,
    /// Country name.
    pub country: String,
}

/// A decoded Google geocoding result.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GeocodeResult {
    /// Google-formatted address.
    pub address: String,
    /// Result location.
    pub location: LatLng,
    /// Normalized address components.
    pub components: GeocodeComponents,
}

/// A decoded Google place result.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Place {
    /// Place name.
    pub name: String,
    /// Formatted address or vicinity.
    pub address: String,
    /// First meaningful Google place type.
    pub category: String,
    /// Great-circle distance from the requested center in meters.
    pub distance_m: u32,
    /// Latitude.
    pub lat: f64,
    /// Longitude.
    pub lng: f64,
}

/// Compatibility alias retained for existing callers.
pub type GoogleMapsError = CloudError;

/// Geocode an address.
///
/// # Errors
/// Returns [`CloudError::Offline`] in offline mode, or propagates API,
/// transport, and decoding errors from the live client.
pub async fn geocode(address: &str, mode: CloudMode) -> Result<GeocodeResult, CloudError> {
    match mode {
        CloudMode::Live => GoogleMapsClient::from_env()?.geocode(address).await,
        CloudMode::Offline | CloudMode::Mock => Err(CloudError::Offline),
    }
}

/// Search for nearby places.
///
/// # Errors
/// Returns [`CloudError::Offline`] in offline mode, or propagates API,
/// transport, and decoding errors from the live client.
pub async fn places_nearby(
    center: LatLng,
    radius_m: u32,
    mode: CloudMode,
) -> Result<Vec<Place>, CloudError> {
    match mode {
        CloudMode::Live => {
            GoogleMapsClient::from_env()?
                .places_nearby(center, radius_m)
                .await
        }
        CloudMode::Offline | CloudMode::Mock => Err(CloudError::Offline),
    }
}

/// Google Maps REST client.
pub struct GoogleMapsClient {
    api_key: String,
    base_url: String,
    http: reqwest::Client,
}

impl GoogleMapsClient {
    /// Construct from `GOOGLE_MAPS_API_KEY` and optional
    /// `NUWAX_GOOGLE_MAPS_BASE_URL`.
    ///
    /// # Errors
    /// Returns [`CloudError::ApiKeyMissing`] when neither supported API-key
    /// environment variable is set or when it is empty.
    pub fn from_env() -> Result<Self, CloudError> {
        let api_key = std::env::var("GOOGLE_MAPS_API_KEY")
            .or_else(|_| std::env::var("NUWAX_GOOGLE_MAPS_API_KEY"))
            .map_err(|_| CloudError::ApiKeyMissing)?;
        if api_key.trim().is_empty() {
            return Err(CloudError::ApiKeyMissing);
        }
        let base_url = std::env::var("NUWAX_GOOGLE_MAPS_BASE_URL")
            .unwrap_or_else(|_| GOOGLE_MAPS_BASE_URL.to_string());
        Ok(Self::with_base_url(api_key, base_url))
    }

    /// Construct a client against the public Google Maps endpoint.
    #[must_use]
    pub fn new(api_key: impl Into<String>) -> Self {
        Self::with_base_url(api_key, GOOGLE_MAPS_BASE_URL)
    }

    /// Construct a client with an alternate endpoint, useful for proxies and
    /// deterministic local integration tests.
    #[must_use]
    pub fn with_base_url(api_key: impl Into<String>, base_url: impl Into<String>) -> Self {
        Self {
            api_key: api_key.into(),
            base_url: base_url.into().trim_end_matches('/').to_string(),
            http: reqwest::Client::builder()
                .timeout(std::time::Duration::from_secs(30))
                .build()
                .expect("reqwest client configuration is valid"),
        }
    }

    /// Perform a live geocoding request and decode its first result.
    ///
    /// # Errors
    /// Returns transport, HTTP, Google API, or response-decoding errors.
    pub async fn geocode(&self, address: &str) -> Result<GeocodeResult, CloudError> {
        let response: GoogleResponse<GooglePlaceResult> =
            self.request_json("geocode/json", &[('a', address)]).await?;
        let item = response
            .results
            .into_iter()
            .next()
            .ok_or_else(|| CloudError::Parse("Google geocode returned no results".into()))?;
        Ok(decode_geocode_result(item))
    }

    /// Perform a live nearby-place request.
    ///
    /// # Errors
    /// Returns transport, HTTP, Google API, or response-decoding errors.
    pub async fn places_nearby(
        &self,
        center: LatLng,
        radius_m: u32,
    ) -> Result<Vec<Place>, CloudError> {
        let location = format!("{},{}", center.lat, center.lng);
        let radius = radius_m.to_string();
        let response: GoogleResponse<GooglePlaceResult> = self
            .request_json(
                "place/nearbysearch/json",
                &[('l', location.as_str()), ('r', radius.as_str())],
            )
            .await?;
        Ok(response
            .results
            .into_iter()
            .map(|item| decode_place(item, center))
            .collect())
    }

    /// Perform a live text-search request.
    ///
    /// # Errors
    /// Returns transport, HTTP, Google API, or response-decoding errors.
    pub async fn text_search(&self, query: &str) -> Result<Vec<Place>, CloudError> {
        let response: GoogleResponse<GooglePlaceResult> = self
            .request_json("place/textsearch/json", &[('q', query)])
            .await?;
        let origin = LatLng { lat: 0.0, lng: 0.0 };
        Ok(response
            .results
            .into_iter()
            .map(|item| decode_place(item, origin))
            .collect())
    }

    /// Masked key suffix for diagnostics.
    #[must_use]
    pub fn api_key_fingerprint(&self) -> String {
        let key = &self.api_key;
        if key.len() <= 4 {
            "***".to_string()
        } else {
            format!("***{}", &key[key.len() - 4..])
        }
    }

    async fn request_json<T: DeserializeOwned>(
        &self,
        endpoint: &str,
        abbreviated_query: &[(char, &str)],
    ) -> Result<GoogleResponse<T>, CloudError> {
        let mut request = self
            .http
            .get(format!("{}/{}", self.base_url, endpoint))
            .query(&[("key", self.api_key.as_str())]);
        for (key, value) in abbreviated_query {
            let name = match key {
                'a' => "address",
                'l' => "location",
                'q' => "query",
                'r' => "radius",
                _ => unreachable!("internal query key"),
            };
            request = request.query(&[(name, *value)]);
        }
        let response = request
            .send()
            .await
            .map_err(|error| CloudError::Http(error.to_string()))?;
        let status = response.status();
        let body = response
            .text()
            .await
            .map_err(|error| CloudError::Http(error.to_string()))?;
        if !status.is_success() {
            return Err(CloudError::HttpStatus {
                status: status.as_u16(),
                message: body,
            });
        }
        let decoded: GoogleResponse<T> =
            serde_json::from_str(&body).map_err(|error| CloudError::Parse(error.to_string()))?;
        if decoded.status != "OK" && decoded.status != "ZERO_RESULTS" {
            return Err(CloudError::HttpStatus {
                status: status.as_u16(),
                message: decoded
                    .error_message
                    .clone()
                    .unwrap_or_else(|| decoded.status.clone()),
            });
        }
        Ok(decoded)
    }
}

#[derive(Debug, Deserialize)]
struct GoogleResponse<T> {
    results: Vec<T>,
    status: String,
    #[serde(default)]
    error_message: Option<String>,
}

#[derive(Debug, Deserialize)]
struct GooglePlaceResult {
    #[serde(default)]
    formatted_address: String,
    #[serde(default)]
    vicinity: String,
    #[serde(default)]
    name: String,
    #[serde(default)]
    types: Vec<String>,
    geometry: GoogleGeometry,
    #[serde(default)]
    address_components: Vec<GoogleAddressComponent>,
}

#[derive(Debug, Deserialize)]
struct GoogleGeometry {
    location: LatLng,
}

#[derive(Debug, Deserialize)]
struct GoogleAddressComponent {
    long_name: String,
    #[serde(default)]
    types: Vec<String>,
}

fn decode_geocode_result(item: GooglePlaceResult) -> GeocodeResult {
    let mut street_number = String::new();
    let mut route = String::new();
    let mut city = String::new();
    let mut country = String::new();
    for component in item.address_components {
        if component.types.iter().any(|value| value == "street_number") {
            street_number = component.long_name;
        } else if component.types.iter().any(|value| value == "route") {
            route = component.long_name;
        } else if component.types.iter().any(|value| value == "locality")
            || (city.is_empty()
                && component
                    .types
                    .iter()
                    .any(|value| value == "administrative_area_level_1"))
        {
            city = component.long_name;
        } else if component.types.iter().any(|value| value == "country") {
            country = component.long_name;
        }
    }
    let street = [street_number, route]
        .into_iter()
        .filter(|value| !value.is_empty())
        .collect::<Vec<_>>()
        .join(" ");
    GeocodeResult {
        address: item.formatted_address,
        location: item.geometry.location,
        components: GeocodeComponents {
            street,
            city,
            country,
        },
    }
}

fn decode_place(item: GooglePlaceResult, center: LatLng) -> Place {
    let location = item.geometry.location;
    let category = item
        .types
        .iter()
        .find(|value| value.as_str() != "point_of_interest" && value.as_str() != "establishment")
        .cloned()
        .unwrap_or_else(|| "unknown".to_string());
    Place {
        name: item.name,
        address: if item.formatted_address.is_empty() {
            item.vicinity
        } else {
            item.formatted_address
        },
        category,
        distance_m: haversine_meters(center, location)
            .round()
            .clamp(0.0, f64::from(u32::MAX)) as u32,
        lat: location.lat,
        lng: location.lng,
    }
}

fn haversine_meters(a: LatLng, b: LatLng) -> f64 {
    const EARTH_RADIUS_M: f64 = 6_371_000.0;
    let lat1 = a.lat.to_radians();
    let lat2 = b.lat.to_radians();
    let delta_lat = (b.lat - a.lat).to_radians();
    let delta_lng = (b.lng - a.lng).to_radians();
    let sin_lat = (delta_lat / 2.0).sin();
    let sin_lng = (delta_lng / 2.0).sin();
    let h = sin_lat * sin_lat + lat1.cos() * lat2.cos() * sin_lng * sin_lng;
    2.0 * EARTH_RADIUS_M * h.sqrt().asin()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn explicit_construct_and_fingerprint() {
        let client = GoogleMapsClient::new("abc12345678");
        assert_eq!(client.api_key_fingerprint(), "***5678");
    }

    #[tokio::test]
    async fn offline_mode_does_not_fabricate_results() {
        assert!(matches!(
            geocode("Tokyo", CloudMode::Offline).await,
            Err(CloudError::Offline)
        ));
        assert!(matches!(
            places_nearby(LatLng { lat: 0.0, lng: 0.0 }, 100, CloudMode::Mock).await,
            Err(CloudError::Offline)
        ));
    }
}
