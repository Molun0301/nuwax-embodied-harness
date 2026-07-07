//! Google Maps REST 客户端

use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Debug, Error)]
/// Google Maps API 错误
/// Google Maps 客户端错误
pub enum GoogleMapsError {
    /// 缺 API key
    #[error("missing API key (set NUWAX_GOOGLE_MAPS_API_KEY env)")]
    MissingApiKey,
    /// HTTP 错误
    #[error("HTTP error: {0}")]
    Http(String),
    /// 解析错误
    #[error("parse error: {0}")]
    Parse(String),
}

/// Place 查询结果
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Place {
    /// 名称
    pub name: String,
    /// 地址
    pub address: String,
    /// 纬度
    pub lat: f64,
    /// 经度
    pub lng: f64,
}

/// Google Maps 客户端
///
/// API key 通过 `NUWAX_GOOGLE_MAPS_API_KEY` 环境变量注入。
#[allow(dead_code)] // http 字段为占位骨架预留，实装 GET Places API 时启用
pub struct GoogleMapsClient {
    /// Google Maps API 密钥
    api_key: String,
    /// HTTP 客户端（实装后用于 GET Places API）
    http: reqwest::Client,
}

impl GoogleMapsClient {
    /// 从环境变量构造
    pub fn from_env() -> Result<Self, GoogleMapsError> {
        let api_key = std::env::var("NUWAX_GOOGLE_MAPS_API_KEY")
            .map_err(|_| GoogleMapsError::MissingApiKey)?;
        Ok(Self::new(api_key))
    }

    /// 显式构造
    #[must_use]
    /// 用显式 API 密钥构造
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
    ) -> Result<Vec<Place>, GoogleMapsError> {
        // 占位：实装需调用 https://maps.googleapis.com/maps/api/place/textsearch/json
        Ok(vec![Place {
            name: query.to_string(),
            address: "(placeholder)".to_string(),
            lat: 0.0,
            lng: 0.0,
        }])
    }

    /// API key 引用（用于 logging/debug，不暴露完整 key）
    #[must_use]
    /// API key 指纹（用于日志脱敏）
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
        assert!(matches!(res, Err(GoogleMapsError::MissingApiKey)));
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
}
