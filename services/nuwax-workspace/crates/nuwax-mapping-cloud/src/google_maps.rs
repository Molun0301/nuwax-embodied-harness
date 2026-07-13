//! Google Maps REST 客户端
//!
//! P1 #5 (rework22 Lane D) 升级：实装 `geocode` + `places_nearby` 真算法骨架。
//!
//! 设计取舍：
//! - `CloudMode::Mock` 用本地 parser + 固定 fixture，保证 dev/CI 可用
//!   （不依赖外网、不依赖 API key）。
//! - `CloudMode::Live` 调真实 HTTPS endpoint，但当前 dev/CI 不启用，
//!   返 `CloudError::ApiKeyMissing` 避免 key 泄露；接入方需自行提供
//!   `NUWAX_GOOGLE_MAPS_API_KEY` 并显式实现 HTTP 路径。
//!
//! 数据结构（GeocodeResult / GeocodeComponents / LatLng / Place）
//! 与 Google Maps Geocoding API JSON 形态 1:1 对齐，方便未来直接
//! `serde_json::from_str` 反序列化。

use serde::{Deserialize, Serialize};
use thiserror::Error;

/// 通用 cloud 错误。
///
/// `CloudError` 是 P1 #5 新引入的统一错误类型，覆盖 geocode / places / nominatim
/// / tile download 全部路径。`GoogleMapsError` 保留为兼容旧 API 入口的别名。
#[derive(Debug, Error)]
pub enum CloudError {
    /// 缺 API key（Live 模式下未注入 NUWAX_GOOGLE_MAPS_API_KEY）
    #[error("missing API key (set NUWAX_GOOGLE_MAPS_API_KEY env)")]
    ApiKeyMissing,
    /// HTTP 错误
    #[error("HTTP error: {0}")]
    Http(String),
    /// 解析错误
    #[error("parse error: {0}")]
    Parse(String),
    /// Feature 已禁用（fallback 模式）
    #[error("cloud map disabled (compile with --features cloud-enabled)")]
    Disabled,
}

/// Cloud 调用模式。
///
/// dev/CI 默认走 `Mock`（不依赖外网/密钥），生产在确认 `NUWAX_GOOGLE_MAPS_API_KEY`
/// 已配置后切换到 `Live`。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CloudMode {
    /// 本地 fixture + parser
    Mock,
    /// 真实 HTTPS 调用（需注入 API key）
    Live,
}

/// 纬度/经度坐标（WGS84 十进制度数）。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LatLng {
    /// 纬度（-90 ~ +90）
    pub lat: f64,
    /// 经度（-180 ~ +180）
    pub lng: f64,
}

/// 地址组件（geocode 返回）。
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct GeocodeComponents {
    /// 街道
    pub street: String,
    /// 城市
    pub city: String,
    /// 国家（取 split last，避免空地址 panic）
    pub country: String,
}

/// Geocode 单条结果。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GeocodeResult {
    /// 原始输入地址
    pub address: String,
    /// 解析得到的经纬度
    pub location: LatLng,
    /// 地址组件
    pub components: GeocodeComponents,
}

/// Places 搜索结果。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Place {
    /// POI 名称
    pub name: String,
    /// 详细地址
    pub address: String,
    /// POI 类型（如 "cafe" / "restaurant"）
    pub category: String,
    /// 距离中心点（米）
    pub distance_m: u32,
    /// 纬度
    pub lat: f64,
    /// 经度
    pub lng: f64,
}

/// Google Maps API 错误（保留为旧调用方的兼容别名，内部用 CloudError）。
pub type GoogleMapsError = CloudError;

/// 地址解析（geocoding）。
///
/// 真实生产应调：
/// `https://maps.googleapis.com/maps/api/geocode/json?address={addr}&key={key}`
///
/// 当前 dev/CI mock 模式：用本地 parser（split on comma）+ 固定 fixture
/// （San Francisco：lat=37.7749, lng=-122.4194）。Live 模式显式返
/// `CloudError::ApiKeyMissing`（避免意外泄露/调用真实 endpoint）。
///
/// # Errors
/// - `CloudError::ApiKeyMissing`: Live 模式下当前不实装 HTTPS
pub fn geocode(address: &str, mode: CloudMode) -> Result<GeocodeResult, CloudError> {
    match mode {
        CloudMode::Mock => {
            // 本地 parser：以逗号分隔,trim 空白。
            // 常见格式 "street, city, country"（Google Maps 也是这种顺序）
            let parts: Vec<&str> = address.split(',').map(str::trim).collect();
            Ok(GeocodeResult {
                address: address.to_string(),
                location: LatLng {
                    lat: 37.7749,
                    lng: -122.4194,
                },
                components: GeocodeComponents {
                    street: parts.first().copied().unwrap_or("").to_string(),
                    city: parts.get(1).copied().unwrap_or("").to_string(),
                    country: parts.last().copied().unwrap_or("").to_string(),
                },
            })
        }
        CloudMode::Live => {
            // TODO: HTTPS POST Google Geocoding API, parse JSON。
            // 当前未启用——避免 API key 泄露。
            Err(CloudError::ApiKeyMissing)
        }
    }
}

/// 周边搜索（Places Nearby Search）。
///
/// 真实生产应调：
/// `https://maps.googleapis.com/maps/api/place/nearbysearch/json?location={lat},{lng}&radius={r}&key={key}`
///
/// Mock 模式：返回 3 个固定 Place（咖啡店 / 餐厅 / 商店），距离按
/// 半径均分（粗略近似）。
///
/// # Errors
/// - `CloudError::ApiKeyMissing`: Live 模式下当前不实装 HTTPS
pub fn places_nearby(
    center: LatLng,
    radius_m: u32,
    mode: CloudMode,
) -> Result<Vec<Place>, CloudError> {
    match mode {
        CloudMode::Mock => {
            // 三个固定 fixture：咖啡店 / 餐厅 / 商店
            // 距离按 radius_m 的 1/3、2/3、3/3 递增排列
            let d1 = radius_m / 3;
            let d2 = radius_m.saturating_mul(2) / 3;
            Ok(vec![
                Place {
                    name: "Mock Cafe".to_string(),
                    address: format!("{:.4}, {:.4}", center.lat, center.lng),
                    category: "cafe".to_string(),
                    distance_m: d1,
                    lat: center.lat + 0.001,
                    lng: center.lng + 0.001,
                },
                Place {
                    name: "Mock Restaurant".to_string(),
                    address: format!("{:.4}, {:.4}", center.lat, center.lng),
                    category: "restaurant".to_string(),
                    distance_m: d2,
                    lat: center.lat - 0.001,
                    lng: center.lng + 0.002,
                },
                Place {
                    name: "Mock Shop".to_string(),
                    address: format!("{:.4}, {:.4}", center.lat, center.lng),
                    category: "store".to_string(),
                    distance_m: radius_m,
                    lat: center.lat + 0.002,
                    lng: center.lng - 0.001,
                },
            ])
        }
        CloudMode::Live => Err(CloudError::ApiKeyMissing),
    }
}

/// Google Maps 客户端
///
/// API key 通过 `NUWAX_GOOGLE_MAPS_API_KEY` 环境变量注入。
#[allow(dead_code)]
pub struct GoogleMapsClient {
    /// Google Maps API 密钥
    api_key: String,
    /// HTTP 客户端（实装后用于 GET Places API）
    http: reqwest::Client,
}

impl GoogleMapsClient {
    /// 从环境变量构造
    ///
    /// # Errors
    /// - `CloudError::ApiKeyMissing`: 未配置 `NUWAX_GOOGLE_MAPS_API_KEY`
    pub fn from_env() -> Result<Self, CloudError> {
        let api_key = std::env::var("NUWAX_GOOGLE_MAPS_API_KEY")
            .map_err(|_| CloudError::ApiKeyMissing)?;
        Ok(Self::new(api_key))
    }

    /// 显式构造
    #[must_use]
    pub fn new(api_key: impl Into<String>) -> Self {
        Self {
            api_key: api_key.into(),
            http: reqwest::Client::builder()
                .timeout(std::time::Duration::from_secs(30))
                .build()
                .expect("reqwest client"),
        }
    }

    /// 文本搜索占位（实际调用需 HTTP，本阶段仅返回 mock）
    pub async fn text_search_placeholder(
        &self,
        query: &str,
    ) -> Result<Vec<Place>, CloudError> {
        // 占位：实装需调用 https://maps.googleapis.com/maps/api/place/textsearch/json
        Ok(vec![Place {
            name: query.to_string(),
            address: "(placeholder)".to_string(),
            category: "unknown".to_string(),
            distance_m: 0,
            lat: 0.0,
            lng: 0.0,
        }])
    }

    /// API key 指纹（用于 logging/debug，不暴露完整 key）
    #[must_use]
    pub fn api_key_fingerprint(&self) -> String {
        let k = &self.api_key;
        if k.len() <= 4 {
            "***".to_string()
        } else {
            format!("***{}", &k[k.len() - 4..])
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn from_env_missing_key() {
        std::env::remove_var("NUWAX_GOOGLE_MAPS_API_KEY");
        let res = GoogleMapsClient::from_env();
        assert!(matches!(res, Err(CloudError::ApiKeyMissing)));
    }

    #[test]
    fn explicit_construct_and_fingerprint() {
        let c = GoogleMapsClient::new("abc12345678");
        assert_eq!(c.api_key_fingerprint(), "***5678");
    }

    #[tokio::test]
    async fn text_search_placeholder_returns_query() {
        let c = GoogleMapsClient::new("abc12345678");
        let res = c.text_search_placeholder("test").await.unwrap();
        assert_eq!(res[0].name, "test");
    }

    #[test]
    fn geocode_mock_returns_fixtures() {
        let r = geocode("123 Main St, Springfield, USA", CloudMode::Mock).unwrap();
        assert_eq!(r.address, "123 Main St, Springfield, USA");
        assert!((r.location.lat - 37.7749).abs() < 1e-9);
        assert!((r.location.lng - -122.4194).abs() < 1e-9);
        assert_eq!(r.components.street, "123 Main St");
        assert_eq!(r.components.city, "Springfield");
        assert_eq!(r.components.country, "USA");
    }

    #[test]
    fn geocode_live_returns_api_key_missing() {
        let r = geocode("anywhere", CloudMode::Live);
        assert!(matches!(r, Err(CloudError::ApiKeyMissing)));
    }

    #[test]
    fn places_nearby_mock_returns_three() {
        let p = places_nearby(LatLng { lat: 0.0, lng: 0.0 }, 1500, CloudMode::Mock).unwrap();
        assert_eq!(p.len(), 3);
        assert_eq!(p[0].distance_m, 500);
        assert_eq!(p[1].distance_m, 1000);
        assert_eq!(p[2].distance_m, 1500);
    }

    #[test]
    fn places_nearby_live_returns_api_key_missing() {
        let r = places_nearby(LatLng { lat: 0.0, lng: 0.0 }, 1000, CloudMode::Live);
        assert!(matches!(r, Err(CloudError::ApiKeyMissing)));
    }
}